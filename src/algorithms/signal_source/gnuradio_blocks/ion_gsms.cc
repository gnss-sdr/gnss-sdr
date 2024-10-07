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
#include <cstdlib>
#include <iostream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

IONGSMSFileSource::IONGSMSFileSource(
    const fs::path& metadata_filepath,
    const GnssMetadata::File& file,
    const GnssMetadata::Block& block,
    const std::vector<std::string>& stream_ids)
    : gr::sync_block(
          "ion_gsms_file_source",
          gr::io_signature::make(0, 0, 0),
          make_output_signature(block, stream_ids)),
      file_stream_(metadata_filepath.parent_path() / file.Url().Value(), std::ios::in | std::ios::binary),
      io_buffer_offset_(0),
      maximum_item_rate_(0),
      chunk_cycle_length_(0)
{
    fs::path data_filepath = metadata_filepath.parent_path() / file.Url().Value();
    std::size_t block_offset = file.Offset();

    if (!file_stream_.is_open())
        {
            LOG(WARNING) << "ION_GSMS_Signal_Source - Unable to open the samples file: " << (data_filepath).c_str();
            std::cerr << "ION_GSMS_Signal_Source - Unable to open the samples file: " << (data_filepath).c_str() << std::endl;
            std::cout << "GNSS-SDR program ended.\n";
            exit(1);
        }

    // Skip offset and block header
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
                    output_stream_item_rates_.push_back(chunk_data_.back()->output_stream_item_rate(i));
                    maximum_item_rate_ = std::max(chunk_data_.back()->output_stream_item_rate(i), maximum_item_rate_);
                }
        }
    output_stream_count_ = output_stream_offset;

    output_stream_total_sample_counts_.resize(output_stream_count_);

    std::size_t cycle_count = block.Cycles();
    if (cycle_count == 0)
        {
            // Read the whole file
            const std::size_t file_size = fs::file_size(data_filepath);
            cycle_count = std::floor((file_size - block_offset - block.SizeHeader()) / chunk_cycle_length_);
        }

    for (std::size_t i = 0; i < output_stream_count_; ++i)
        {
            output_stream_total_sample_counts_[i] = cycle_count * output_stream_item_rates_[i];
        }
}


std::size_t IONGSMSFileSource::output_stream_count() const
{
    return output_stream_count_;
}


std::size_t IONGSMSFileSource::output_stream_item_size(std::size_t stream_index) const
{
    return output_stream_item_sizes_[stream_index];
}


std::size_t IONGSMSFileSource::output_stream_total_sample_count(std::size_t stream_index) const
{
    return output_stream_total_sample_counts_[stream_index];
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


int IONGSMSFileSource::work(
    int noutput_items,
    gr_vector_const_void_star& input_items __attribute__((unused)),
    gr_vector_void_star& output_items)
{
    // Compute the maximum number of samples that will be copied across all output buffer.
    // If there are more than one output buffer (multichannel set up), the one with the most samples will be used as the maximum.
    //
    // Complex samples produce 2 items each (I and Q). In order to account for them, we subtract 1 from `noutput_items` and
    // then floor the division. During testing, not doing this caused `max_sample_output` to oscillate between two values, thus
    // resizing the `io_buffer_` on each call to `work()`.
    const std::size_t max_sample_output = std::floor((noutput_items - 1.0) / maximum_item_rate_);

    // Resize the IO buffer to fit exactly the maximum amount of samples that will be outputted.
    io_buffer_.resize(max_sample_output * chunk_cycle_length_);

    // We will be walking the IO buffer with this variable.
    io_buffer_offset_ = 0;

    // Read samples from file into IO buffer
    const std::size_t bytes_to_read = io_buffer_.size();
    file_stream_.read(io_buffer_.data(), bytes_to_read);

    // Reset `items_produced_` vector. This vector will accumulate the amount of items produced for each output stream.
    items_produced_.clear();
    items_produced_.resize(output_items.size());

    // Walk the IO buffer one chunk cycle at a time. See ION documentation for a definition of chunk and chunk cycle.
    while (io_buffer_offset_ < bytes_to_read)
        {
            // Iterate chunks within a chunk cycle
            for (auto& chunk : chunk_data_)
                {
                    // Copy chunk into a separate buffer where the samples will be shifted from.
                    const std::size_t bytes_copied = chunk->read_from_buffer(reinterpret_cast<uint8_t*>(io_buffer_.data()), io_buffer_offset_);

                    // Advance IO buffer offset
                    io_buffer_offset_ += bytes_copied;

                    // Shift samples into output buffers following the appropriate unpacking strategy for this chunk.
                    chunk->write_to_output(output_items, items_produced_);
                }
        }

    // Call `produce(int, int)` with the appropriate item count for each output stream.
    for (std::size_t i = 0; i < items_produced_.size(); ++i)
        {
            produce(i, items_produced_[i]);
        }

    return WORK_CALLED_PRODUCE;
}
