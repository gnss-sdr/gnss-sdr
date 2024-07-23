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

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>

#include <utility>
#endif


IONGSMSFileSource::IONGSMSFileSource(
    const std::filesystem::path& metadata_filepath,
    const GnssMetadata::File& file,
    const GnssMetadata::Block& block,
    const std::vector<std::string>& stream_ids)
    : gr::sync_block(
          "ion_gsms_file_source",
          gr::io_signature::make(0, 0, 0),
          make_output_signature(block, stream_ids)),
      file_metadata_(file),
      block_metadata_(block)
{
    std::filesystem::path data_filepath = metadata_filepath.parent_path() / file.Url().Value();
    fd_ = std::fopen(data_filepath.c_str(), "rb");
    std::size_t block_offset = file.Offset();
    std::fseek(fd_, file.Offset() + block_offset + block.SizeHeader(), SEEK_SET);

    std::size_t output_stream_offset = 0;
    for (const auto& chunk : block.Chunks())
        {
            chunk_data_.emplace_back(std::make_shared<IONGSMSChunkData>(chunk, stream_ids, output_stream_offset));
            const std::size_t out_count = chunk_data_.back()->output_stream_count();
            output_stream_offset += out_count;
            for (std::size_t i = 0; i < out_count; ++i)
                {
                    output_stream_item_sizes_.push_back(chunk_data_.back()->output_stream_item_size(i));
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
    // for (int i = 0; i < noutput_items; ++i)
        // {
            for (auto& c : chunk_data_)
                {
                    c->read_from_file(fd_);
                    c->write_to_output(output_items, [&](int output, int nitems) {
                        produce(output, nitems);
                    });
                }
        // }
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


