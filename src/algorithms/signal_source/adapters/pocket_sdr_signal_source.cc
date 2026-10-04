/*!
 * \file pocket_sdr_signal_source.cc
 * \brief Signal source for Pocket SDR front-ends
 * \author Minhaj Ahmad, 2026. mahmad12(at)crimson.ua.edu
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

#include "pocket_sdr_signal_source.h"
#include "configuration_interface.h"
#include "gnss_frequencies.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_valve.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

PocketSdrSignalSource::PocketSdrSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_stream,
    unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "Pocket_SDR_Signal_Source"s),
      item_type_(configuration->property(role + ".item_type", std::string("gr_complex"))),
      dump_filename_(configuration->property(role + ".dump_filename", std::string("./data/signal_source.dat"))),
      conf_file_(configuration->property(role + ".conf_file", std::string())),
      sampling_frequency_(configuration->property(role + ".sampling_frequency", 0.0)),
      freq_(configuration->property(role + ".freq", FREQ1)),
      samples_(configuration->property(role + ".samples", int64_t(0))),
      in_stream_(in_stream),
      out_stream_(out_stream),
      channel_(configuration->property(role + ".channel", 1)),
      dump_(configuration->property(role + ".dump", false))
{
    if (item_type_ != "gr_complex")
        {
            LOG(WARNING) << item_type_ << " unrecognized item type. Using gr_complex.";
            item_type_ = std::string("gr_complex");
        }
    item_size_ = sizeof(gr_complex);

    // 1. Make the driver instance: the device is configured with a standard
    // PocketSDR configuration file (pocket_conf format); an empty conf_file
    // keeps the current device settings
    try
        {
            pocketsdr_source_ = gr::pocketsdr::pocketsdr_source::make(conf_file_, std::vector<int>{channel_});
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << "Exception creating gr-pocketsdr source: " << e.what();
            throw std::invalid_argument("Wrong Pocket SDR arguments or device not found");
        }

    // 2. Read back the actual settings from the device
    const double actual_sample_rate = pocketsdr_source_->get_sample_rate();
    std::cout << "Actual RX Rate: " << actual_sample_rate << " [SPS]...\n";
    LOG(INFO) << "Actual RX Rate: " << actual_sample_rate << " [SPS]...";

    if (sampling_frequency_ > 0.0 && std::fabs(actual_sample_rate - sampling_frequency_) > 1.0)
        {
            std::cerr << "Warning: device sampling rate " << actual_sample_rate
                      << " SPS does not match " << role << ".sampling_frequency="
                      << sampling_frequency_ << " SPS in the configuration file\n";
            LOG(WARNING) << "Device sampling rate " << actual_sample_rate
                         << " SPS does not match configured " << sampling_frequency_ << " SPS";
        }

    const double actual_center_freq = pocketsdr_source_->get_center_freq(0);
    std::cout << "Actual RX Freq: " << actual_center_freq << " [Hz]...\n";
    LOG(INFO) << "Actual RX Freq: " << actual_center_freq << " [Hz]...";

    if (std::fabs(actual_center_freq - freq_) > 1.0)
        {
            std::cerr << "Warning: device LO frequency " << actual_center_freq
                      << " Hz does not match " << role << ".freq=" << freq_
                      << " Hz in the configuration file\n";
            LOG(WARNING) << "Device LO frequency " << actual_center_freq
                         << " Hz does not match configured " << freq_ << " Hz";
        }

    if (samples_ != 0)
        {
            DLOG(INFO) << "Send STOP signal after " << samples_ << " samples";
            valve_ = gnss_sdr_make_valve(item_size_, samples_, queue);
            DLOG(INFO) << "valve(" << valve_->unique_id() << ")";
        }

    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }

    if (in_stream_ > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void PocketSdrSignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->connect(pocketsdr_source_, 0, valve_, 0);
            DLOG(INFO) << "connected pocketsdr source to valve";
            if (dump_)
                {
                    top_block->connect(valve_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected valve to file sink";
                }
        }
    else
        {
            if (dump_)
                {
                    top_block->connect(pocketsdr_source_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected pocketsdr source to file sink";
                }
        }
}


void PocketSdrSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->disconnect(pocketsdr_source_, 0, valve_, 0);
            if (dump_)
                {
                    top_block->disconnect(valve_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (dump_)
                {
                    top_block->disconnect(pocketsdr_source_, 0, file_sink_, 0);
                }
        }
}


gr::basic_block_sptr PocketSdrSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return {};
}


gr::basic_block_sptr PocketSdrSignalSource::get_right_block()
{
    if (samples_ != 0)
        {
            return valve_;
        }
    return pocketsdr_source_;
}
