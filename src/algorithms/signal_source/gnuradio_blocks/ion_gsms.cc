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
    : IONGSMSFileSource(
          std::vector<SegmentDescriptor>{make_segment_descriptor(metadata_filepath, file, block, block_start_offset)},
          stream_ids)
{
}


IONGSMSFileSource::IONGSMSFileSource(
    const std::vector<SegmentDescriptor>& segments,
    const std::vector<std::string>& stream_ids)
    : gr::sync_block(
          "ion_gsms_file_source",
          gr::io_signature::make(0, 0, 0),
          make_output_signature(segments, stream_ids)),
      io_buffer_offset_(0),
      maximum_item_rate_(0),
      chunk_cycle_length_(0),
      cycles_remaining_(0),
      current_segment_index_(0),
      output_stream_ids_(segment_output_stream_ids(segments, stream_ids))
{
    output_stream_count_ = output_stream_ids_.size();
    output_stream_item_sizes_.assign(output_stream_count_, 0);
    output_stream_item_rates_.assign(output_stream_count_, 0);
    output_stream_total_sample_counts_.assign(output_stream_count_, 0);

    if (segments.empty())
        {
            throw std::runtime_error("ION_GSMS_Signal_Source requires at least one source segment");
        }

    for (const auto& descriptor : segments)
        {
            if (descriptor.block == nullptr)
                {
                    throw std::runtime_error("ION_GSMS_Signal_Source source segment has no block metadata");
                }

            SegmentData segment;
            segment.data_filepath = descriptor.data_filepath;
            segment.block = descriptor.block;
            segment.block_start_offset = descriptor.block_start_offset;
            segment.output_stream_item_rates.assign(output_stream_count_, 0);

            for (const auto& chunk : descriptor.block->Chunks())
                {
                    segment.chunk_data.emplace_back(std::make_shared<IONGSMSChunkData>(chunk, output_stream_ids_, 0));
                    segment.chunk_cycle_length += chunk.CountWords() * chunk.SizeWord();
                    for (std::size_t i = 0; i < output_stream_count_; ++i)
                        {
                            const auto chunk_item_rate = segment.chunk_data.back()->output_stream_item_rate(i);
                            if (chunk_item_rate == 0)
                                {
                                    continue;
                                }

                            const auto chunk_item_size = segment.chunk_data.back()->output_stream_item_size(i);
                            if (output_stream_item_sizes_[i] != 0 && output_stream_item_sizes_[i] != chunk_item_size)
                                {
                                    throw std::runtime_error("ION_GSMS_Signal_Source stream appears with inconsistent output item sizes");
                                }
                            output_stream_item_sizes_[i] = chunk_item_size;
                            segment.output_stream_item_rates[i] += chunk_item_rate;
                            segment.maximum_item_rate = std::max(segment.output_stream_item_rates[i], segment.maximum_item_rate);
                        }
                }

            std::size_t cycle_count = descriptor.block->Cycles();
            if (cycle_count == 0)
                {
                    if (!descriptor.block_extends_to_eof)
                        {
                            throw std::runtime_error(
                                "ION_GSMS_Signal_Source block has cycles=0 before the final lane block; "
                                "refusing EOF-based cycle inference because later blocks would be unreachable");
                        }
                    const std::string warning = "ION_GSMS_Signal_Source block at offset " + std::to_string(descriptor.block_start_offset) +
                                                " in " + segment.data_filepath.string() +
                                                " has cycles=0; inferring cycle count from EOF. This is a non-standard metadata extension supported only for the final block in a lane.";
                    LOG(WARNING) << warning;
                    std::cerr << "Warning: " << warning << std::endl;
                    cycle_count = infer_cycle_count_from_file(segment.data_filepath, *descriptor.block, descriptor.block_start_offset, segment.chunk_cycle_length);
                }
            segment.cycle_count = cycle_count;

            for (std::size_t i = 0; i < output_stream_count_; ++i)
                {
                    output_stream_item_rates_[i] = std::max(output_stream_item_rates_[i], segment.output_stream_item_rates[i]);
                    output_stream_total_sample_counts_[i] += cycle_count * segment.output_stream_item_rates[i];
                }
            maximum_item_rate_ = std::max(maximum_item_rate_, segment.maximum_item_rate);
            segments_.push_back(std::move(segment));
        }

    for (std::size_t i = 0; i < output_stream_count_; ++i)
        {
            if (output_stream_item_sizes_[i] == 0)
                {
                    throw std::runtime_error("ION_GSMS_Signal_Source requested stream is not present in source segments");
                }
        }

    current_segment_index_ = 0;
    advance_to_next_segment();
}


IONGSMSFileSource::SegmentDescriptor IONGSMSFileSource::make_segment_descriptor(
    const fs::path& metadata_filepath,
    const GnssMetadata::File& file,
    const GnssMetadata::Block& block,
    const std::size_t block_start_offset)
{
    return {metadata_filepath.parent_path() / file.Url().Value(), &block, block_start_offset, true};
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


std::vector<std::string> IONGSMSFileSource::segment_output_stream_ids(const std::vector<SegmentDescriptor>& segments, const std::vector<std::string>& stream_ids)
{
    std::vector<std::string> output_stream_ids;
    for (const auto& stream_id : stream_ids)
        {
            for (const auto& segment : segments)
                {
                    if (segment.block != nullptr && block_contains_stream_id(*segment.block, stream_id))
                        {
                            output_stream_ids.push_back(stream_id);
                            break;
                        }
                }
        }

    return output_stream_ids;
}


int IONGSMSFileSource::output_item_size_for_stream_id(const std::vector<SegmentDescriptor>& segments, const std::string& stream_id)
{
    int item_size = 0;
    for (const auto& segment : segments)
        {
            if (segment.block == nullptr)
                {
                    continue;
                }
            const auto current_item_size = output_item_size_for_stream_id(*segment.block, stream_id);
            if (current_item_size == 0)
                {
                    continue;
                }
            if (item_size != 0 && item_size != current_item_size)
                {
                    throw std::runtime_error("ION_GSMS_Signal_Source stream appears with inconsistent output item sizes");
                }
            item_size = current_item_size;
        }

    return item_size;
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


gr::io_signature::sptr IONGSMSFileSource::make_output_signature(const std::vector<SegmentDescriptor>& segments, const std::vector<std::string>& stream_ids)
{
    const auto output_stream_ids = segment_output_stream_ids(segments, stream_ids);
    if (output_stream_ids.empty())
        {
            throw std::runtime_error("ION_GSMS_Signal_Source requested streams are not present in source segments");
        }
    std::vector<int> item_sizes{};

    for (const auto& stream_id : output_stream_ids)
        {
            const auto item_size = output_item_size_for_stream_id(segments, stream_id);
            if (item_size == 0)
                {
                    throw std::runtime_error("ION_GSMS_Signal_Source requested stream is not present in source segments");
                }
            item_sizes.push_back(item_size);
        }

    return gr::io_signature::makev(
        static_cast<int>(item_sizes.size()),
        static_cast<int>(item_sizes.size()),
        item_sizes);
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


bool IONGSMSFileSource::advance_to_next_segment()
{
    file_stream_.close();
    while (current_segment_index_ < segments_.size())
        {
            auto& segment = segments_[current_segment_index_];
            if (segment.cycle_count == 0)
                {
                    ++current_segment_index_;
                    continue;
                }

            file_stream_.open(segment.data_filepath, std::ios::in | std::ios::binary);
            if (!file_stream_.is_open())
                {
                    LOG(WARNING) << "ION_GSMS_Signal_Source - Unable to open the samples file: " << segment.data_filepath.c_str();
                    std::cerr << "ION_GSMS_Signal_Source - Unable to open the samples file: " << segment.data_filepath.c_str() << std::endl;
                    std::cout << "GNSS-SDR program ended.\n";
                    exit(1);
                }

            file_stream_.seekg(static_cast<std::streamoff>(segment.block_start_offset + segment.block->SizeHeader()), std::ios::beg);
            cycles_remaining_ = segment.cycle_count;
            chunk_cycle_length_ = segment.chunk_cycle_length;
            maximum_item_rate_ = segment.maximum_item_rate;
            return true;
        }

    cycles_remaining_ = 0;
    chunk_cycle_length_ = 0;
    maximum_item_rate_ = 0;
    return false;
}


int IONGSMSFileSource::work(
    int noutput_items,
    gr_vector_const_void_star& input_items __attribute__((unused)),
    gr_vector_void_star& output_items)
{
    if (cycles_remaining_ == 0)
        {
            if (!advance_to_next_segment())
                {
                    return WORK_DONE;
                }
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
    auto& segment = segments_[current_segment_index_];

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
            ++current_segment_index_;
            return 0;
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
            for (auto& chunk : segment.chunk_data)
                {
                    // Copy chunk into a separate buffer where the samples will be shifted from.
                    const std::size_t bytes_copied = chunk->read_from_buffer(reinterpret_cast<uint8_t*>(io_buffer_.data()), io_buffer_offset_);

                    // Advance IO buffer offset
                    io_buffer_offset_ += bytes_copied;

                    // Shift samples into output buffers following the appropriate unpacking strategy for this chunk.
                    chunk->write_to_output(output_items, items_produced_);
                }
        }

    if (cycles_remaining_ == 0)
        {
            ++current_segment_index_;
        }

    // Call `produce(int, int)` with the appropriate item count for each output stream.
    for (std::size_t i = 0; i < items_produced_.size(); ++i)
        {
            produce(i, items_produced_[i]);
        }

    return WORK_CALLED_PRODUCE;
}
