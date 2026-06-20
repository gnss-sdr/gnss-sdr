/*!
 * \file ion_gsms_signal_source.h
 * \brief GNSS-SDR Signal Source that reads sample streams following ION's GNSS-SDR metadata standard
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

#include "ion_gsms_signal_source.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_valve.h"
#include <gnuradio/blocks/copy.h>
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

using namespace std::string_literals;

IONGSMSSignalSource::IONGSMSSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "ION_GSMS_Signal_Source"s),
      stream_ids_(parse_comma_list(configuration->property(role + ".streams"s, ""s))),
      metadata_filepath_(configuration->property(role + ".metadata_filename"s, "./example_capture_metadata.sdrx"s)),
      minimum_tail_s_(kMinimumTailSeconds),
      sampling_frequency_(configuration->property(role + ".sampling_frequency"s, configuration->property("GNSS-SDR.internal_fs_sps"s, int64_t(0)))),
      in_streams_(in_streams),
      out_streams_(out_streams)
{
    if (in_streams_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_streams_ <= 0)
        {
            LOG(ERROR) << "A signal source does not have an output stream";
        }
    if (stream_ids_.empty())
        {
            throw std::runtime_error("ION_GSMS_Signal_Source requires at least one stream id in the .streams property");
        }

    // Parse XML metadata file
    load_metadata();

    // Make source vector
    sources_ = make_stream_sources(stream_ids_);
    if (sources_.empty())
        {
            throw std::runtime_error("ION_GSMS_Signal_Source no configured streams were found in the metadata");
        }

    for (const auto& source : sources_)
        {
            for (std::size_t i = 0; i < source->output_stream_count(); ++i)
                {
                    copy_blocks_.emplace_back(gr::blocks::copy::make(source->output_stream_item_size(i)));
                    valves_.emplace_back(gnss_sdr_make_valve(source->output_stream_item_size(i), valve_sample_count(source->output_stream_total_sample_count(i)), queue));
                }
        }
}


void IONGSMSSignalSource::load_metadata()
{
    metadata_ = std::make_shared<GnssMetadata::Metadata>();
    try
        {
            GnssMetadata::XmlProcessor xml_proc;
            if (!xml_proc.Load(metadata_filepath_.c_str(), false, *metadata_))
                {
                    LOG(WARNING) << "Could not load XML metadata file " << metadata_filepath_;
                    std::cerr << "Could not load XML metadata file " << metadata_filepath_ << std::endl;
                    std::cout << "GNSS-SDR program ended.\n";
                    exit(1);
                }
        }
    catch (GnssMetadata::ApiException& e)
        {
            LOG(WARNING) << "API Exception while loading XML metadata file: " << std::to_string(e.Error());
            std::cerr << "Could not load XML metadata file " << metadata_filepath_ << " : " << std::to_string(e.Error()) << std::endl;
            std::cout << "GNSS-SDR program ended.\n";
            exit(1);
        }
    catch (std::exception& e)
        {
            LOG(WARNING) << "Exception while loading XML metadata file: " << e.what();
            std::cerr << "Could not load XML metadata file " << metadata_filepath_ << " : " << e.what() << std::endl;
            std::cout << "GNSS-SDR program ended.\n";
            exit(1);
        }
}


std::vector<std::string> IONGSMSSignalSource::parse_comma_list(const std::string& str)
{
    std::vector<std::string> list{};
    std::size_t prev_comma_at{0};

    while (prev_comma_at < str.size())
        {
            std::size_t comma_at = str.find_first_of(',', prev_comma_at);
            if (comma_at == std::string::npos)
                {
                    comma_at = str.size();
                }
            auto value = str.substr(prev_comma_at, (comma_at - prev_comma_at));
            const auto first_non_space = value.find_first_not_of(" \t\n\r");
            if (first_non_space != std::string::npos)
                {
                    const auto last_non_space = value.find_last_not_of(" \t\n\r");
                    value = value.substr(first_non_space, last_non_space - first_non_space + 1);
                    if (std::find(list.begin(), list.end(), value) == list.end())
                        {
                            list.push_back(value);
                        }
                }
            prev_comma_at = comma_at + 1;
        }

    return list;
}


bool IONGSMSSignalSource::block_contains_stream(const GnssMetadata::Block& block, const std::vector<std::string>& stream_ids)
{
    for (const auto& chunk : block.Chunks())
        {
            for (const auto& lump : chunk.Lumps())
                {
                    for (const auto& stream : lump.Streams())
                        {
                            if (std::find(stream_ids.begin(), stream_ids.end(), stream.Id()) != stream_ids.end())
                                {
                                    return true;
                                }
                        }
                }
        }

    return false;
}


std::size_t IONGSMSSignalSource::chunk_cycle_bytes(const GnssMetadata::Block& block)
{
    std::size_t bytes = 0;
    for (const auto& chunk : block.Chunks())
        {
            bytes += chunk.CountWords() * chunk.SizeWord();
        }
    return bytes;
}


std::size_t IONGSMSSignalSource::infer_block_cycles(
    const fs::path& data_filepath,
    const GnssMetadata::Block& block,
    const std::size_t block_start_offset)
{
    if (block.Cycles() != 0)
        {
            return block.Cycles();
        }

    const std::size_t cycle_bytes = chunk_cycle_bytes(block);
    if (cycle_bytes == 0)
        {
            return 0;
        }

    const std::size_t file_size = fs::file_size(data_filepath);
    if (file_size <= block_start_offset)
        {
            return 0;
        }

    const std::size_t remaining_bytes = file_size - block_start_offset;
    const std::size_t block_overhead = block.SizeHeader() + block.SizeFooter();
    if (remaining_bytes <= block_overhead)
        {
            return 0;
        }

    return (remaining_bytes - block_overhead) / cycle_bytes;
}


std::size_t IONGSMSSignalSource::block_storage_bytes(
    const fs::path& data_filepath,
    const GnssMetadata::Block& block,
    const std::size_t block_start_offset)
{
    const std::size_t cycle_count = infer_block_cycles(data_filepath, block, block_start_offset);
    return block.SizeHeader() + cycle_count * chunk_cycle_bytes(block) + block.SizeFooter();
}


std::vector<IONGSMSFileSource::sptr> IONGSMSSignalSource::make_stream_sources(const std::vector<std::string>& stream_ids) const
{
    std::vector<IONGSMSFileSource::sptr> sources{};
    for (const auto& file : metadata_->Files())
        {
            const fs::path data_filepath = fs::path(metadata_filepath_).parent_path() / file.Url().Value();
            for (const auto& lane : metadata_->Lanes())
                {
                    if (lane.Id() == file.Lane().Id())
                        {
                            std::size_t block_start_offset = file.Offset();
                            for (const auto& block : lane.Blocks())
                                {
                                    if (block_contains_stream(block, stream_ids))
                                        {
                                            auto source = gnss_make_shared<IONGSMSFileSource>(
                                                metadata_filepath_,
                                                file,
                                                block,
                                                block_start_offset,
                                                stream_ids);

                                            sources.push_back(source);
                                        }
                                    block_start_offset += block_storage_bytes(data_filepath, block, block_start_offset);
                                }
                            break;
                        }
                }
        }

    return sources;
}


std::uint64_t IONGSMSSignalSource::valve_sample_count(std::uint64_t total_sample_count) const
{
    if (total_sample_count == 0 || sampling_frequency_ <= 0)
        {
            return total_sample_count;
        }

    const auto tail_samples = static_cast<std::uint64_t>(std::ceil(minimum_tail_s_ * static_cast<double>(sampling_frequency_)));
    if (total_sample_count <= tail_samples)
        {
            std::cout << "Warning: ION_GSMS_Signal_Source stream has " << total_sample_count
                      << " samples, which is shorter than the configured " << tail_samples
                      << " sample receiver tail. Setting the valve to process 1 sample.\n";
            return 1;
        }

    return total_sample_count - tail_samples;
}


void IONGSMSSignalSource::connect(gr::top_block_sptr top_block)
{
    std::size_t cumulative_index = 0;
    for (const auto& source : sources_)
        {
            for (std::size_t i = 0; i < source->output_stream_count(); ++i, ++cumulative_index)
                {
                    top_block->connect(source, i, copy_blocks_[cumulative_index], 0);
                    top_block->connect(copy_blocks_[cumulative_index], 0, valves_[cumulative_index], 0);
                }
        }
}


void IONGSMSSignalSource::disconnect(gr::top_block_sptr top_block)
{
    std::size_t cumulative_index = 0;
    for (const auto& source : sources_)
        {
            for (std::size_t i = 0; i < source->output_stream_count(); ++i, ++cumulative_index)
                {
                    top_block->disconnect(source, i, copy_blocks_[cumulative_index], 0);
                    top_block->disconnect(copy_blocks_[cumulative_index], 0, valves_[cumulative_index], 0);
                }
        }
}


gr::basic_block_sptr IONGSMSSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    // return gr_basic_block_sptr();
    return IONGSMSFileSource::sptr();
}


gr::basic_block_sptr IONGSMSSignalSource::get_right_block()
{
    return get_right_block(0);
}


gr::basic_block_sptr IONGSMSSignalSource::get_right_block(int RF_channel)
{
    if (RF_channel < 0 || RF_channel >= static_cast<int>(copy_blocks_.size()))
        {
            LOG(WARNING) << "'RF_channel' out of bounds while trying to get signal source right block.";
            if (valves_.empty())
                {
                    return {};
                }
            return valves_[0];
        }
    return valves_[RF_channel];
}
