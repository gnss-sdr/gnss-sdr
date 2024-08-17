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
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "gnuradio/block.h"
#include "ion_gsms.h"
#include <memory>
#include <algorithm>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>

#include <utility>
#endif

using namespace std::string_literals;

IONGSMSFileSource::IONGSMSFileSource(
    const ConfigurationInterface* configuration,
    const std::string& role,
    const std::filesystem::path& metadata_filepath,
    const GnssMetadata::File& file,
    const GnssMetadata::Block& block,
    const std::vector<std::string>& stream_ids)
    : gr::sync_block(
          "ion_gsms_file_source",
          gr::io_signature::make(0, 0, 0),
          make_output_signature(block, stream_ids)),
      file_metadata_(file),
      block_metadata_(block),
      io_buffer_offset_(0),
      maximum_item_rate_(0),
      chunk_cycle_length_(0)
{
    std::filesystem::path data_filepath = metadata_filepath.parent_path() / file.Url().Value();
    fd_ = std::fopen(data_filepath.c_str(), "rb");
    std::size_t block_offset = file.Offset();
    std::fseek(fd_, file.Offset() + block_offset + block.SizeHeader(), SEEK_SET);

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

IONGSMSFileSource::~IONGSMSFileSource()
{
    std::fclose(fd_);
}

int IONGSMSFileSource::work(
    int noutput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    const std::size_t max_sample_output = std::floor((noutput_items-1.0) / maximum_item_rate_);
    io_buffer_.resize(max_sample_output * chunk_cycle_length_);
    io_buffer_offset_ = 0;
    std::fread(io_buffer_.data(), sizeof(decltype(io_buffer_)::value_type), io_buffer_.size(), fd_);

    items_produced_.resize(output_items.size());
    for (int i = 0; i < items_produced_.size(); ++i)
        {
            items_produced_[i] = 0;
        }

    while (io_buffer_offset_ < io_buffer_.size())
        {
            for (auto& c : chunk_data_)
                {
                    auto* chunk = c.get();
                    io_buffer_offset_ += chunk->read_from_buffer(io_buffer_.data(), io_buffer_offset_);
                    chunk->write_to_output(output_items, items_produced_);
                }
        }

    for (int i = 0; i < items_produced_.size(); ++i)
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
    std::vector<size_t> item_sizes{};

    for (const auto& chunk : block.Chunks())
        {
            for (const auto& lump : chunk.Lumps())
                {
                    for (const auto& stream : lump.Streams())
                        {
                            if (std::ranges::any_of(stream_ids.begin(), stream_ids.end(), [&](const std::string& it) {
                                    return stream.Id() == it;
                                }))
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

    return gr::io_signature::make(
        nstreams,
        nstreams,
        item_sizes);
}


