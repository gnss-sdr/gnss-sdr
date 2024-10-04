/*!
 * \file ion_gsms_signal_source.h
 * \brief GNSS-SDR Signal Source that reads sample streams following ION's GNSS-SDR metadata standard
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

#include "ion_gsms_signal_source.h"
#include "gnss_sdr_flags.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_valve.h"
#include <gnuradio/blocks/copy.h>
#include <cstdlib>
#include <iostream>
#include <unordered_set>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

namespace
{
std::vector<std::string> parse_comma_list(const std::string& str)
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
            list.emplace_back(str.substr(prev_comma_at, (comma_at - prev_comma_at)));
            prev_comma_at = comma_at + 1;
        }

    return list;
}
}  // anonymous namespace


IONGSMSSignalSource::IONGSMSSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "ION_GSMS_Signal_Source"s),
      stream_ids_(parse_comma_list(configuration->property(role + ".streams"s, ""s))),
      metadata_filepath_(configuration->property(role + ".metadata_filename"s, "./example_capture_metadata.sdrx"s)),
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

    // Parse XML metadata file
    load_metadata();

    // Make source vector
    sources_ = make_stream_sources(stream_ids_);

    for (const auto& source : sources_)
        {
            for (std::size_t i = 0; i < source->output_stream_count(); ++i)
                {
                    copy_blocks_.emplace_back(gr::blocks::copy::make(source->output_stream_item_size(i)));
                    valves_.emplace_back(gnss_sdr_make_valve(source->output_stream_item_size(i), source->output_stream_total_sample_count(i), queue));
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


std::vector<IONGSMSFileSource::sptr> IONGSMSSignalSource::make_stream_sources(const std::vector<std::string>& stream_ids) const
{
    std::vector<IONGSMSFileSource::sptr> sources{};
    for (const auto& file : metadata_->Files())
        {
            for (const auto& lane : metadata_->Lanes())
                {
                    if (lane.Id() == file.Lane().Id())
                        {
                            for (const auto& block : lane.Blocks())
                                {
                                    bool block_done = false;
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
                                                                    auto source = gnss_make_shared<IONGSMSFileSource>(
                                                                        metadata_filepath_,
                                                                        file,
                                                                        block,
                                                                        stream_ids);

                                                                    sources.push_back(source);

                                                                    // This file source will take care of any other matching streams in this block
                                                                    // We can skip the rest of this block
                                                                    block_done = true;
                                                                    break;
                                                                }
                                                        }

                                                    if (block_done)
                                                        {
                                                            break;
                                                        }
                                                }
                                            if (block_done)
                                                {
                                                    break;
                                                }
                                        }
                                }
                            break;
                        }
                }
        }

    return sources;
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
            return valves_[0];
        }
    return valves_[RF_channel];
}
