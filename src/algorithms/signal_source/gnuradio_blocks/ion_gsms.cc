/*!
 * \file ion_gsms.cc
 * \brief GNU Radio block that reads a Block from a file following ION's GNSS-SDR metadata standard
 * \author Víctor Castillo Agüero, 2024. victorcastilloaguero(at)gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "ion_gsms.h"
#include "gnuradio/block.h"
#include <algorithm>
#include <cmath>

using namespace std::string_literals;

IONGSMSFileSource::IONGSMSFileSource(
    const fs::path& metadata_filepath,
    const GnssMetadata::File& file,
    const GnssMetadata::Block& block,
    const std::vector<std::string>& stream_ids)
    : gr::sync_block(
          "ion_gsms_file_source",
          gr::io_signature::make(0, 0, 0),
          make_output_signature(block, stream_ids)),
      file_stream_(metadata_filepath.parent_path() / file.Url().Value()),
      io_buffer_offset_(0),
      maximum_item_rate_(0),
      chunk_cycle_length_(0)
{
    fs::path data_filepath = metadata_filepath.parent_path() / file.Url().Value();
    std::size_t block_offset = file.Offset();
    file_stream_.seekg(file.Offset() + block_offset + block.SizeHeader());

    std::size_t output_stream_offset = 0;
    for (const auto& chunk : block.Chunks())
        {
            chunk_data_.emplace_back(std::make_shared<IONGSMSChunkData>(chunk, stream_ids, output_stream_offset));
            chunk_cycle_length_ += chunk.CountWords() * chunk.SizeWord();
            const std::size_t out_count = chunk_data_.back()->output_stream_count();
            output_stream_offset += out_count;
            for (std::size_t i = 0; i < out_count; ++i)
                {
                    output_stream_item_sizes_.push_back(chunk_data_.back()->output_stream_item_size(i));
                    maximum_item_rate_ = std::max(chunk_data_.back()->output_stream_item_rate(i), maximum_item_rate_);
                }
        }
    output_stream_count_ = output_stream_offset;
}


int IONGSMSFileSource::work(
    int noutput_items,
    gr_vector_const_void_star& input_items __attribute__((unused)),
    gr_vector_void_star& output_items)
{
    const std::size_t max_sample_output = std::floor((noutput_items - 1.0) / maximum_item_rate_);
    io_buffer_.resize(max_sample_output * chunk_cycle_length_);
    io_buffer_offset_ = 0;
    file_stream_.read(io_buffer_.data(), sizeof(decltype(io_buffer_)::value_type) * io_buffer_.size());

    items_produced_.clear();
    items_produced_.resize(output_items.size());

    while (io_buffer_offset_ < io_buffer_.size())
        {
            for (auto& c : chunk_data_)
                {
                    auto* chunk = c.get();
                    io_buffer_offset_ += chunk->read_from_buffer(reinterpret_cast<uint8_t*>(io_buffer_.data()), io_buffer_offset_);
                    chunk->write_to_output(output_items, items_produced_);
                }
        }

    for (std::size_t i = 0; i < items_produced_.size(); ++i)
        {
            produce(i, items_produced_[i]);
        }

    return WORK_CALLED_PRODUCE;
}


std::size_t IONGSMSFileSource::output_stream_count() const
{
    return output_stream_count_;
}


std::size_t IONGSMSFileSource::output_stream_item_size(std::size_t stream_index) const
{
    return output_stream_item_sizes_[stream_index];
}


gr::io_signature::sptr IONGSMSFileSource::make_output_signature(const GnssMetadata::Block& block, const std::vector<std::string>& stream_ids)
{
    int nstreams = 0;
    std::vector<int> item_sizes{};

    for (const auto& chunk : block.Chunks())
        {
            for (const auto& lump : chunk.Lumps())
                {
                    for (const auto& stream : lump.Streams())
                        {
                            bool found = false;
                            for (const auto& stream_id : stream_ids)
                                {
                                    if (stream_id == stream.Id())
                                        {
                                            found = true;
                                            break;
                                        }
                                }
                            if (found)
                                {
                                    ++nstreams;
                                    std::size_t sample_bitsize = stream.Packedbits() / stream.RateFactor();
                                    if (stream.Packedbits() >= 2 * stream.RateFactor() * stream.Quantization())
                                        {
                                            // Samples have 'Complex' format
                                            sample_bitsize /= 2;
                                        }
                                    item_sizes.push_back(bits_to_item_size(sample_bitsize));
                                }
                        }
                }
        }

    return gr::io_signature::makev(
        nstreams,
        nstreams,
        item_sizes);
}
