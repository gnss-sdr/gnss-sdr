/*!
 * \file ion_gsms.cc
 * \brief GNU Radio block that reads a Block from a file following ION's GNSS-SDR metadata standard
 * \author Víctor Castillo Agüero, 2024. victorcastilloaguero(at)gmail.com
 * \author Carles Fernandez, 2026 carles.fernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
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
#include <stdexcept>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

IONGSMSFileSource::IONGSMSFileSource(
    const fs::path& metadata_filepath,
    const GnssMetadata::File& file,
    const GnssMetadata::Block& block,
    const std::size_t block_start_offset,
    const std::vector<std::string>& stream_ids)
    : gr::sync_block(
          "ion_gsms_file_source",
          gr::io_signature::make(0, 0, 0),
          make_output_signature(block, stream_ids)),
      file_stream_(metadata_filepath.parent_path() / file.Url().Value(), std::ios::in | std::ios::binary),
      io_buffer_offset_(0),
      maximum_item_rate_(0),
      chunk_cycle_length_(0),
      cycles_remaining_(0)
{
    fs::path data_filepath = metadata_filepath.parent_path() / file.Url().Value();
    const auto output_stream_ids = block_output_stream_ids(block, stream_ids);

    if (!file_stream_.is_open())
        {
            LOG(WARNING) << "ION_GSMS_Signal_Source - Unable to open the samples file: " << (data_filepath).c_str();
            std::cerr << "ION_GSMS_Signal_Source - Unable to open the samples file: " << (data_filepath).c_str() << std::endl;
            std::cout << "GNSS-SDR program ended.\n";
            exit(1);
        }

    // Skip to this block's sample payload, after the lane offset and block header.
    file_stream_.seekg(static_cast<std::streamoff>(block_start_offset + block.SizeHeader()), std::ios::beg);

    output_stream_count_ = output_stream_ids.size();
    output_stream_item_sizes_.assign(output_stream_count_, 0);
    output_stream_item_rates_.assign(output_stream_count_, 0);

    for (const auto& chunk : block.Chunks())
        {
            chunk_data_.emplace_back(std::make_shared<IONGSMSChunkData>(chunk, output_stream_ids, 0));
            chunk_cycle_length_ += chunk.CountWords() * chunk.SizeWord();
            for (std::size_t i = 0; i < output_stream_count_; ++i)
                {
                    const auto chunk_item_rate = chunk_data_.back()->output_stream_item_rate(i);
                    if (chunk_item_rate == 0)
                        {
                            continue;
                        }

                    const auto chunk_item_size = chunk_data_.back()->output_stream_item_size(i);
                    if (output_stream_item_sizes_[i] != 0 && output_stream_item_sizes_[i] != chunk_item_size)
                        {
                            throw std::runtime_error("ION_GSMS_Signal_Source stream appears with inconsistent output item sizes");
                        }
                    output_stream_item_sizes_[i] = chunk_item_size;
                    output_stream_item_rates_[i] += chunk_item_rate;
                    maximum_item_rate_ = std::max(output_stream_item_rates_[i], maximum_item_rate_);
                }
        }

    output_stream_total_sample_counts_.resize(output_stream_count_);

    std::size_t cycle_count = block.Cycles();
    if (cycle_count == 0)
        {
            cycle_count = infer_cycle_count_from_file(data_filepath, block, block_start_offset, chunk_cycle_length_);
        }
    cycles_remaining_ = cycle_count;

    for (std::size_t i = 0; i < output_stream_count_; ++i)
        {
            output_stream_total_sample_counts_[i] = cycle_count * output_stream_item_rates_[i];
        }
}


bool IONGSMSFileSource::block_contains_stream_id(const GnssMetadata::Block& block, const std::string& stream_id)
{
    for (const auto& chunk : block.Chunks())
        {
            for (const auto& lump : chunk.Lumps())
                {
                    for (const auto& stream : lump.Streams())
                        {
                            if (stream.Id() == stream_id)
                                {
                                    return true;
                                }
                        }
                }
        }

    return false;
}


std::vector<std::string> IONGSMSFileSource::block_output_stream_ids(const GnssMetadata::Block& block, const std::vector<std::string>& stream_ids)
{
    std::vector<std::string> output_stream_ids;
    for (const auto& stream_id : stream_ids)
        {
            if (std::find(output_stream_ids.begin(), output_stream_ids.end(), stream_id) == output_stream_ids.end() &&
                block_contains_stream_id(block, stream_id))
                {
                    output_stream_ids.push_back(stream_id);
                }
        }

    return output_stream_ids;
}


int IONGSMSFileSource::output_item_size_for_stream_id(const GnssMetadata::Block& block, const std::string& stream_id)
{
    int item_size = 0;
    for (const auto& chunk : block.Chunks())
        {
            for (const auto& lump : chunk.Lumps())
                {
                    for (const auto& stream : lump.Streams())
                        {
                            if (stream.Id() != stream_id)
                                {
                                    continue;
                                }

                            const auto current_item_size = static_cast<int>(IONGSMSChunkData::stream_output_item_size(stream));
                            if (item_size != 0 && item_size != current_item_size)
                                {
                                    throw std::runtime_error("ION_GSMS_Signal_Source stream appears with inconsistent output item sizes");
                                }
                            item_size = current_item_size;
                        }
                }
        }

    return item_size;
}


std::size_t IONGSMSFileSource::infer_cycle_count_from_file(
    const fs::path& data_filepath,
    const GnssMetadata::Block& block,
    const std::size_t block_start_offset,
    const std::size_t chunk_cycle_length)
{
    if (chunk_cycle_length == 0)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source block has zero-length chunk cycle");
        }

    const std::size_t file_size = fs::file_size(data_filepath);
    const std::size_t payload_start = block_start_offset + block.SizeHeader();
    if (file_size <= payload_start)
        {
            return 0;
        }

    std::size_t payload_bytes = file_size - payload_start;
    if (payload_bytes <= block.SizeFooter())
        {
            return 0;
        }
    payload_bytes -= block.SizeFooter();
    return payload_bytes / chunk_cycle_length;
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
    const auto output_stream_ids = block_output_stream_ids(block, stream_ids);
    std::vector<int> item_sizes{};

    for (const auto& stream_id : output_stream_ids)
        {
            const auto item_size = output_item_size_for_stream_id(block, stream_id);
            if (item_size == 0)
                {
                    throw std::runtime_error("ION_GSMS_Signal_Source requested stream is not present in block");
                }
            item_sizes.push_back(item_size);
        }

    return gr::io_signature::makev(
        static_cast<int>(item_sizes.size()),
        static_cast<int>(item_sizes.size()),
        item_sizes);
}


int IONGSMSFileSource::work(
    int noutput_items,
    gr_vector_const_void_star& input_items __attribute__((unused)),
    gr_vector_void_star& output_items)
{
    if (cycles_remaining_ == 0)
        {
            return WORK_DONE;
        }
    if (noutput_items <= 0 || maximum_item_rate_ == 0 || chunk_cycle_length_ == 0)
        {
            return 0;
        }

    // Compute the maximum number of samples that will be copied across all output buffer.
    // If there are more than one output buffer (multichannel set up), the one with the most samples will be used as the maximum.
    const std::size_t max_sample_output = static_cast<std::size_t>(noutput_items) / maximum_item_rate_;
    if (max_sample_output == 0)
        {
            return 0;
        }
    const std::size_t cycles_to_read = std::min(max_sample_output, cycles_remaining_);

    // Resize the IO buffer to fit exactly the maximum amount of samples that will be outputted.
    io_buffer_.resize(cycles_to_read * chunk_cycle_length_);

    // We will be walking the IO buffer with this variable.
    io_buffer_offset_ = 0;

    // Read samples from file into IO buffer
    const std::size_t bytes_to_read = io_buffer_.size();
    file_stream_.read(io_buffer_.data(), bytes_to_read);
    const auto bytes_read = static_cast<std::size_t>(file_stream_.gcount());
    const std::size_t cycles_read = bytes_read / chunk_cycle_length_;
    const std::size_t bytes_to_decode = cycles_read * chunk_cycle_length_;
    if (cycles_read == 0)
        {
            cycles_remaining_ = 0;
            return WORK_DONE;
        }
    cycles_remaining_ = cycles_read >= cycles_remaining_ ? 0 : cycles_remaining_ - cycles_read;
    if (bytes_read < bytes_to_read)
        {
            cycles_remaining_ = 0;
        }

    // Reset `items_produced_` vector. This vector will accumulate the amount of items produced for each output stream.
    items_produced_.clear();
    items_produced_.resize(output_items.size());

    // Walk the IO buffer one chunk cycle at a time. See ION documentation for a definition of chunk and chunk cycle.
    while (io_buffer_offset_ < bytes_to_decode)
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
