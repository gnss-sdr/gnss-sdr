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
#include <iterator>
#include <limits>
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

    for (const auto& source_data : sources_)
        {
            const auto& source = source_data.source;
            for (std::size_t i = 0; i < source->output_stream_count(); ++i)
                {
                    copy_blocks_.emplace_back(gr::blocks::copy::make(source->output_stream_item_size(i)));
                    valves_.emplace_back(gnss_sdr_make_valve(source->output_stream_item_size(i), valve_sample_count(source->output_stream_total_sample_count(i), source_data.sampling_frequency), queue));
                }
        }
}


const char* IONGSMSSignalSource::file_uri_scheme()
{
    return "file://";
}


bool IONGSMSSignalSource::starts_with(const std::string& value, const std::string& prefix)
{
    return value.compare(0, prefix.size(), prefix) == 0;
}


fs::path IONGSMSSignalSource::absolute_path_key(const fs::path& path)
{
    try
        {
            return fs::canonical(path);
        }
    catch (const fs::filesystem_error&)
        {
            return fs::absolute(path);
        }
}


fs::path IONGSMSSignalSource::resolve_local_metadata_uri(const fs::path& metadata_path, const std::string& uri)
{
    if (uri.empty())
        {
            throw std::runtime_error("ION_GSMS metadata include URI is empty");
        }

    fs::path include_path;
    const std::string scheme(file_uri_scheme());
    if (starts_with(uri, scheme))
        {
            include_path = fs::path(uri.substr(scheme.size()));
        }
    else if (uri.find("://") != std::string::npos)
        {
            throw std::runtime_error(
                "ION_GSMS metadata include URI '" + uri + "' is not a local file URI");
        }
    else
        {
            include_path = fs::path(uri);
        }

    if (include_path.is_absolute())
        {
            return include_path;
        }

    return metadata_path.parent_path() / include_path;
}


void IONGSMSSignalSource::load_metadata()
{
    metadata_ = std::make_shared<GnssMetadata::Metadata>();
    metadata_files_.clear();
    try
        {
            std::vector<std::string> include_stack;
            load_metadata_file(fs::path(metadata_filepath_), *metadata_, include_stack);
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


void IONGSMSSignalSource::load_metadata_file(
    const fs::path& metadata_path,
    GnssMetadata::Metadata& metadata,
    std::vector<std::string>& include_stack)
{
    const auto metadata_key = absolute_path_key(metadata_path).string();
    if (std::find(include_stack.begin(), include_stack.end(), metadata_key) != include_stack.end())
        {
            throw std::runtime_error("ION_GSMS metadata include cycle detected at " + metadata_path.string());
        }

    include_stack.push_back(metadata_key);
    try
        {
            GnssMetadata::XmlProcessor xml_proc;
            const auto metadata_path_string = metadata_path.string();
            if (!xml_proc.Load(metadata_path_string.c_str(), false, metadata))
                {
                    throw std::runtime_error("Could not load XML metadata file " + metadata_path_string);
                }

            const auto metadata_directory = metadata_path.parent_path();
            for (const auto& file : metadata.Files())
                {
                    metadata_files_.push_back({metadata_files_.size(), true, file, metadata_directory});
                }

            const std::vector<GnssMetadata::AnyUri> includes(metadata.Includes().begin(), metadata.Includes().end());
            metadata.Includes().clear();
            for (const auto& include : includes)
                {
                    GnssMetadata::Metadata included_metadata;
                    load_metadata_file(
                        resolve_local_metadata_uri(metadata_path, include.Value()),
                        included_metadata,
                        include_stack);
                    metadata.Splice(included_metadata);
                }
        }
    catch (...)
        {
            include_stack.pop_back();
            throw;
        }
    include_stack.pop_back();
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
    const std::size_t block_start_offset,
    const bool block_extends_to_eof)
{
    if (block.Cycles() != 0)
        {
            return block.Cycles();
        }

    if (!block_extends_to_eof)
        {
            throw std::runtime_error(
                "ION_GSMS_Signal_Source block has cycles=0 before the final lane block; "
                "refusing EOF-based cycle inference because later blocks would be unreachable");
        }
    LOG(WARNING) << "ION_GSMS_Signal_Source block at offset " << block_start_offset
                 << " in " << data_filepath.string()
                 << " has cycles=0; inferring cycle count from EOF. This is a non-standard metadata extension supported only for the final block in a lane.";

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
    const std::size_t block_start_offset,
    const bool block_extends_to_eof)
{
    const std::size_t cycle_count = infer_block_cycles(data_filepath, block, block_start_offset, block_extends_to_eof);
    return block.SizeHeader() + cycle_count * chunk_cycle_bytes(block) + block.SizeFooter();
}


std::vector<IONGSMSSignalSource::StreamSourceData> IONGSMSSignalSource::make_stream_sources(const std::vector<std::string>& stream_ids) const
{
    std::vector<StreamSourceData> sources{};
    const auto files = ordered_metadata_files();
    for (const auto& stream_id : stream_ids)
        {
            std::vector<IONGSMSFileSource::SegmentDescriptor> segments{};
            std::int64_t stream_sampling_frequency = 0;
            for (const auto& metadata_file : files)
                {
                    if (!metadata_file.has_file)
                        {
                            continue;
                        }
                    const auto& file = metadata_file.file;
                    const fs::path data_filepath = metadata_file.metadata_directory / file.Url().Value();
                    for (const auto& lane : metadata_->Lanes())
                        {
                            if (lane.Id() == file.Lane().Id())
                                {
                                    std::size_t block_start_offset = file.Offset();
                                    const auto& blocks = lane.Blocks();
                                    for (auto block_iter = blocks.begin(); block_iter != blocks.end(); ++block_iter)
                                        {
                                            const auto& block = *block_iter;
                                            const bool block_extends_to_eof = std::next(block_iter) == blocks.end();
                                            if (block_contains_stream(block, std::vector<std::string>{stream_id}))
                                                {
                                                    segments.push_back({data_filepath, &block, block_start_offset, block_extends_to_eof});
                                                    stream_sampling_frequency = reconcile_sampling_frequency(
                                                        stream_sampling_frequency,
                                                        stream_sampling_frequency_hz(lane, block, stream_id),
                                                        stream_id);
                                                }
                                            if (!block_extends_to_eof)
                                                {
                                                    block_start_offset += block_storage_bytes(data_filepath, block, block_start_offset, block_extends_to_eof);
                                                }
                                        }
                                    break;
                                }
                        }
                }
            if (segments.empty())
                {
                    throw std::runtime_error("ION_GSMS_Signal_Source requested stream '" + stream_id + "' was not found in the metadata");
                }
            if (stream_sampling_frequency == 0)
                {
                    stream_sampling_frequency = sampling_frequency_;
                }
            sources.push_back({gnss_make_shared<IONGSMSFileSource>(segments, std::vector<std::string>{stream_id}), stream_sampling_frequency});
        }

    return sources;
}


std::vector<IONGSMSSignalSource::MetadataFileData> IONGSMSSignalSource::ordered_metadata_files() const
{
    std::vector<MetadataFileData> files = metadata_files_;
    if (files.empty())
        {
            const auto metadata_directory = fs::path(metadata_filepath_).parent_path();
            for (const auto& file : metadata_->Files())
                {
                    files.push_back({files.size(), true, file, metadata_directory});
                }
        }

    std::vector<MetadataFileData> ordered_files{};
    std::vector<MetadataFileData> remaining_files = files;
    auto find_by_url = [](const std::vector<MetadataFileData>& candidates, const std::string& url) {
        return std::find_if(candidates.begin(), candidates.end(), [&url](const auto& file) {
            return file.has_file && file.file.Url().Value() == url;
        });
    };

    auto append_chain = [&ordered_files, &remaining_files, &find_by_url](MetadataFileData file_data) {
        while (file_data.has_file)
            {
                ordered_files.push_back(file_data);
                remaining_files.erase(std::remove_if(remaining_files.begin(), remaining_files.end(), [&file_data](const auto& candidate) {
                    return candidate.has_file && candidate.sequence == file_data.sequence;
                }),
                    remaining_files.end());
                const auto next = file_data.file.Next().Value();
                if (next.empty())
                    {
                        file_data = {};
                    }
                else
                    {
                        const auto next_file = find_by_url(remaining_files, next);
                        file_data = next_file == remaining_files.end() ? MetadataFileData{} : *next_file;
                    }
            }
    };

    for (const auto& fileset : metadata_->FileSets())
        {
            for (const auto& file_url : fileset.FileUrls())
                {
                    const auto first_file = find_by_url(remaining_files, file_url.Value());
                    if (first_file != remaining_files.end())
                        {
                            append_chain(*first_file);
                        }
                }
        }

    while (!remaining_files.empty())
        {
            auto first_file = std::find_if(remaining_files.begin(), remaining_files.end(), [&remaining_files, &find_by_url](const auto& file) {
                if (!file.has_file)
                    {
                        return false;
                    }
                const auto previous = file.file.Previous().Value();
                return previous.empty() || find_by_url(remaining_files, previous) == remaining_files.end();
            });
            if (first_file == remaining_files.end())
                {
                    first_file = remaining_files.begin();
                }

            append_chain(*first_file);
        }

    return ordered_files;
}


const GnssMetadata::System* IONGSMSSignalSource::resolve_system(const GnssMetadata::System& system) const
{
    if (!system.IsReference() && system.BaseFrequency().toHertz() > 0.0)
        {
            return &system;
        }

    for (const auto& metadata_system : metadata_->Systems())
        {
            if (metadata_system.Id() == system.Id())
                {
                    return &metadata_system;
                }
        }

    return system.IsReference() ? nullptr : &system;
}


double IONGSMSSignalSource::lane_base_frequency_hz(const GnssMetadata::Lane& lane) const
{
    double base_frequency = 0.0;
    for (const auto& lane_system : lane.Systems())
        {
            const auto* system = resolve_system(lane_system);
            if (system == nullptr)
                {
                    continue;
                }

            const double system_base_frequency = system->BaseFrequency().toHertz();
            if (system_base_frequency <= 0.0)
                {
                    continue;
                }
            if (base_frequency > 0.0 && std::llround(base_frequency) != std::llround(system_base_frequency))
                {
                    throw std::runtime_error("ION_GSMS_Signal_Source lane references systems with inconsistent freqbase values");
                }
            base_frequency = system_base_frequency;
        }

    return base_frequency;
}


std::int64_t IONGSMSSignalSource::stream_sampling_frequency_hz(
    const GnssMetadata::Lane& lane,
    const GnssMetadata::Block& block,
    const std::string& stream_id) const
{
    const double base_frequency = lane_base_frequency_hz(lane);
    if (base_frequency <= 0.0)
        {
            return 0;
        }

    std::size_t rate_factor = 0;
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
                            if (rate_factor != 0 && rate_factor != stream.RateFactor())
                                {
                                    throw std::runtime_error("ION_GSMS_Signal_Source stream '" + stream_id + "' appears with inconsistent ratefactor values");
                                }
                            rate_factor = stream.RateFactor();
                        }
                }
        }

    if (rate_factor == 0)
        {
            return 0;
        }

    const double sampling_frequency = base_frequency * static_cast<double>(rate_factor);
    if (sampling_frequency <= 0.0 || sampling_frequency > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
        {
            throw std::runtime_error("ION_GSMS_Signal_Source stream '" + stream_id + "' has an unsupported sampling frequency");
        }

    return static_cast<std::int64_t>(std::llround(sampling_frequency));
}


std::int64_t IONGSMSSignalSource::reconcile_sampling_frequency(
    const std::int64_t current_frequency,
    const std::int64_t candidate_frequency,
    const std::string& stream_id)
{
    if (candidate_frequency <= 0)
        {
            return current_frequency;
        }
    if (current_frequency > 0 && current_frequency != candidate_frequency)
        {
            throw std::runtime_error("ION_GSMS_Signal_Source stream '" + stream_id + "' appears with inconsistent sampling frequencies");
        }
    return candidate_frequency;
}


std::uint64_t IONGSMSSignalSource::valve_sample_count(std::uint64_t total_sample_count, const std::int64_t sampling_frequency) const
{
    if (total_sample_count == 0 || sampling_frequency <= 0)
        {
            return total_sample_count;
        }

    const auto tail_samples = static_cast<std::uint64_t>(std::ceil(minimum_tail_s_ * static_cast<double>(sampling_frequency)));
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
    for (const auto& source_data : sources_)
        {
            const auto& source = source_data.source;
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
    for (const auto& source_data : sources_)
        {
            const auto& source = source_data.source;
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
    if (RF_channel < 0 || RF_channel >= static_cast<int>(valves_.size()))
        {
            LOG(WARNING) << "'RF_channel' out of bounds while trying to get signal source right block.";
            return {};
        }
    return valves_[RF_channel];
}


size_t IONGSMSSignalSource::getRfChannels() const
{
    return valves_.size();
}
