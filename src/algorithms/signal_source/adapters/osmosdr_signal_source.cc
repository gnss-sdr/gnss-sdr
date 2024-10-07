/*!
 * \file osmosdr_signal_source.cc
 * \brief Signal source for the Realtek RTL2832U USB dongle DVB-T receiver
 * (see https://osmocom.org/projects/rtl-sdr/wiki for more information)
 * \author Javier Arribas, 2012. jarribas(at)cttc.es
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

#include "osmosdr_signal_source.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_valve.h"
#include <boost/exception/diagnostic_information.hpp>
#include <gnuradio/blocks/file_sink.h>
#include <iostream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


using namespace std::string_literals;


OsmosdrSignalSource::OsmosdrSignalSource(const ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_stream, unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "Osmosdr_Signal_Source"s),
      item_type_(configuration->property(role + ".item_type", std::string("gr_complex"))),
      dump_filename_(configuration->property(role + ".dump_filename", std::string("./data/signal_source.dat"))),
      osmosdr_args_(configuration->property(role + ".osmosdr_args", std::string())),
      antenna_(configuration->property(role + ".antenna", std::string())),
      sample_rate_(configuration->property(role + ".sampling_frequency", 2.0e6)),
      freq_(configuration->property(role + ".freq", GPS_L1_FREQ_HZ)),
      gain_(configuration->property(role + ".gain", 40.0)),
      if_gain_(configuration->property(role + ".if_gain", 40.0)),
      rf_gain_(configuration->property(role + ".rf_gain", 40.0)),
      if_bw_(configuration->property(role + ".if_bw", 0.0)),
      samples_(configuration->property(role + ".samples", static_cast<int64_t>(0))),
      in_stream_(in_stream),
      out_stream_(out_stream),
      AGC_enabled_(configuration->property(role + ".AGC_enabled", true)),
      dump_(configuration->property(role + ".dump", false))
{
    if (item_type_ == "short")
        {
            item_size_ = sizeof(int16_t);
        }
    else if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            // 1. Make the driver instance
            OsmosdrSignalSource::driver_instance();

            // For LimeSDR: Set RX antenna
            if (!antenna_.empty())
                {
                    osmosdr_source_->set_antenna(antenna_, 0);
                    std::cout << "Set RX Antenna: " << osmosdr_source_->get_antenna(0) << '\n';
                    LOG(INFO) << "Set RX Antenna: " << osmosdr_source_->get_antenna(0);
                }

            // 2 set sampling rate
            osmosdr_source_->set_sample_rate(sample_rate_);
            std::cout << "Actual RX Rate: " << osmosdr_source_->get_sample_rate() << " [SPS]...\n";
            LOG(INFO) << "Actual RX Rate: " << osmosdr_source_->get_sample_rate() << " [SPS]...";

            // 3. set rx frequency
            osmosdr_source_->set_center_freq(freq_);
            std::cout << "Actual RX Freq: " << osmosdr_source_->get_center_freq() << " [Hz]...\n";
            LOG(INFO) << "Actual RX Freq: " << osmosdr_source_->get_center_freq() << " [Hz]...";

            // TODO: Assign the remnant IF from the PLL tune error
            std::cout << "PLL Frequency tune error: " << osmosdr_source_->get_center_freq() - freq_ << " [Hz]...\n";
            LOG(INFO) << "PLL Frequency tune error: " << osmosdr_source_->get_center_freq() - freq_ << " [Hz]...\n";

            // 4. set rx gain
            if (this->AGC_enabled_ == true)
                {
                    osmosdr_source_->set_gain_mode(true);
                    std::cout << "AGC enabled\n";
                    LOG(INFO) << "AGC enabled";
                }
            else
                {
                    osmosdr_source_->set_gain_mode(false);
                    osmosdr_source_->set_gain(gain_, 0);
                    osmosdr_source_->set_if_gain(rf_gain_, 0);
                    osmosdr_source_->set_bb_gain(if_gain_, 0);
                    if (!osmosdr_args_.empty() && (osmosdr_args_.find("bladerf") != std::string::npos))
                        {
                            std::cout << "Actual LNA Gain: " << osmosdr_source_->get_gain("LNA", 0) << " dB...\n";
                            std::cout << "Actual VGA1 Gain: " << osmosdr_source_->get_gain("VGA1", 0) << " dB...\n";
                            std::cout << "Actual VGA2 Gain: " << osmosdr_source_->get_gain("VGA2", 0) << " dB...\n";
                        }
                    else
                        {
                            if (!osmosdr_args_.empty() && (osmosdr_args_.find("xtrx") != std::string::npos))
                                {
                                    osmosdr_source_->set_gain(gain_, "LNA", 0);
                                    osmosdr_source_->set_gain(rf_gain_, "TIA", 0);
                                    osmosdr_source_->set_gain(if_gain_, "PGA", 0);
                                    std::cout << "Actual XTRX LNA Gain: " << osmosdr_source_->get_gain("LNA", 0) << " dB...\n";
                                    std::cout << "Actual XTRX TIA Gain: " << osmosdr_source_->get_gain("TIA", 0) << " dB...\n";
                                    std::cout << "Actual XTRX PGA Gain: " << osmosdr_source_->get_gain("PGA", 0) << " dB...\n";
                                }
                            else
                                {
                                    std::cout << "Actual RX Gain: " << osmosdr_source_->get_gain() << " dB...\n";
                                    LOG(INFO) << "Actual RX Gain: " << osmosdr_source_->get_gain() << " dB...";
                                }
                        }
                }

            // 5. set bandwidth
            if (if_bw_ > 0.0)
                {
                    osmosdr_source_->set_bandwidth(if_bw_, 0);
                }

            // Get actual bandwidth
            std::cout << "Actual Bandwidth: " << osmosdr_source_->get_bandwidth(0) << " [Hz]...\n";
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type. Using short.";
            item_size_ = sizeof(int16_t);
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


void OsmosdrSignalSource::driver_instance()
{
    try
        {
            if (!osmosdr_args_.empty())
                {
                    std::cout << "OsmoSdr arguments: " << osmosdr_args_ << '\n';
                    LOG(INFO) << "OsmoSdr arguments: " << osmosdr_args_;
                }
            osmosdr_source_ = osmosdr::source::make(osmosdr_args_);
        }
    catch (const boost::exception& e)
        {
            LOG(WARNING) << "Boost exception: " << boost::diagnostic_information(e);
            throw std::invalid_argument("Wrong OsmoSdr arguments");
        }
}


void OsmosdrSignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->connect(osmosdr_source_, 0, valve_, 0);
            DLOG(INFO) << "connected osmosdr source to valve";
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
                    top_block->connect(osmosdr_source_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected osmosdr source to file sink";
                }
        }
}


void OsmosdrSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->disconnect(osmosdr_source_, 0, valve_, 0);
            if (dump_)
                {
                    top_block->disconnect(valve_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (dump_)
                {
                    top_block->disconnect(osmosdr_source_, 0, file_sink_, 0);
                }
        }
}


gr::basic_block_sptr OsmosdrSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return {};
}


gr::basic_block_sptr OsmosdrSignalSource::get_right_block()
{
    if (samples_ != 0)
        {
            return valve_;
        }
    else
        {
            return osmosdr_source_;
        }
}
