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
#include <gnuradio/blocks/copy.h>
#include <string>
#include <unordered_set>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

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


IONGSMSSignalSource::IONGSMSSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams,
    Concurrent_Queue<pmt::pmt_t>* queue __attribute__((unused)))
    : SignalSourceBase(configuration, role, "ION_GSMS_Signal_Source"s),
      stream_ids_(parse_comma_list(configuration->property(role + ".streams"s, ""s))),
      metadata_(configuration->property(role + ".metadata_filename"s, "../data/example_capture_metadata.sdrx"s)),
      timestamp_clock_offset_ms_(configuration->property(role + ".timestamp_clock_offset_ms"s, 0.0)),
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

    sources_ = metadata_.make_stream_sources(stream_ids_);

    for (const auto& source : sources_)
        {
            for (std::size_t i = 0; i < source->output_stream_count(); ++i)
                {
                    copy_blocks_.push_back(gr::blocks::copy::make(source->output_stream_item_size(i)));
                }
        }
}


void IONGSMSSignalSource::connect(gr::top_block_sptr top_block)
{
    std::size_t cumulative_index = 0;
    for (const auto& source : sources_)
        {
            for (std::size_t i = 0; i < source->output_stream_count(); ++i, ++cumulative_index)
                {
                    top_block->connect(source, i, copy_blocks_[cumulative_index], 0);
                }
        }
}


void IONGSMSSignalSource::disconnect(gr::top_block_sptr top_block)
{
    for (const auto& source : sources_)
        {
            top_block->disconnect(source);
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
            return copy_blocks_[0];
        }
    return copy_blocks_[RF_channel];
}
