/*!
 * \file rtl_tcp_signal_source.cc
 * \brief Signal source for the Realtek RTL2832U USB dongle DVB-T receiver
 *        over TCP.
 * (see https://osmocom.org/projects/rtl-sdr/wiki for more information)
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
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

#include "rtl_tcp_signal_source.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_valve.h"
#include <boost/exception/diagnostic_information.hpp>
#include <glog/logging.h>
#include <cstdint>
#include <iostream>
#include <utility>

using namespace std::string_literals;

RtlTcpSignalSource::RtlTcpSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_stream,
    unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "RtlTcp_Signal_Source"s),
      samples_(configuration->property(role + ".samples", static_cast<uint64_t>(0))),
      rf_gain_(configuration->property(role + ".rf_gain", 40.0)),
      sample_rate_(configuration->property(role + ".sampling_frequency", 2000000)),
      freq_(configuration->property(role + ".freq", static_cast<int>(GPS_L1_FREQ_HZ))),
      gain_(configuration->property(role + ".gain", 40)),
      if_gain_(configuration->property(role + ".if_gain", 40)),
      in_stream_(in_stream),
      out_stream_(out_stream),
      AGC_enabled_(configuration->property(role + ".AGC_enabled", true)),
      flip_iq_(configuration->property(role + ".flip_iq", false)),
      dump_(configuration->property(role + ".dump", false))
{
    // DUMP PARAMETERS
    const std::string default_dump_file("./data/signal_source.dat");
    const std::string default_item_type("gr_complex");
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    // rtl_tcp PARAMETERS
    const std::string default_address("127.0.0.1");
    const int16_t default_port = 1234;
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    address_ = configuration->property(role + ".address", default_address);
    port_ = configuration->property(role + ".port", default_port);

    if (item_type_ == "short")
        {
            item_size_ = sizeof(int16_t);
        }
    else if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            // 1. Make the gr block
            MakeBlock();

            // 2 set sampling rate
            signal_source_->set_sample_rate(sample_rate_);

            // 3. set rx frequency
            signal_source_->set_frequency(freq_);

            // 4. set rx gain
            signal_source_->set_agc_mode(true);

            if (this->AGC_enabled_ == true)
                {
                    std::cout << "AGC enabled\n";
                    LOG(INFO) << "AGC enabled";
                    signal_source_->set_agc_mode(true);
                }
            else
                {
                    std::cout << "AGC disabled\n";
                    LOG(INFO) << "AGC disabled";
                    signal_source_->set_agc_mode(false);

                    std::cout << "Setting gain to " << gain_ << '\n';
                    LOG(INFO) << "Setting gain to " << gain_;
                    signal_source_->set_gain(gain_);

                    std::cout << "Setting IF gain to " << if_gain_ << '\n';
                    LOG(INFO) << "Setting IF gain to " << if_gain_;
                    signal_source_->set_if_gain(if_gain_);
                }
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type. Using short.";
            item_size_ = sizeof(int16_t);
        }

    if (samples_ != 0ULL)
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


void RtlTcpSignalSource::MakeBlock()
{
    try
        {
            std::cout << "Connecting to " << address_ << ":" << port_ << '\n';
            LOG(INFO) << "Connecting to " << address_ << ":" << port_;
            signal_source_ = rtl_tcp_make_signal_source_c(address_, port_, flip_iq_);
        }
    catch (const boost::exception& e)
        {
            LOG(WARNING) << "Boost exception: " << boost::diagnostic_information(e);
            throw std::runtime_error("Failure connecting to the device");
        }
}


void RtlTcpSignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ != 0ULL)
        {
            top_block->connect(signal_source_, 0, valve_, 0);
            DLOG(INFO) << "connected rtl tcp source to valve";
            if (dump_)
                {
                    top_block->connect(valve_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected valve to file sink";
                }
        }
    else if (dump_)
        {
            top_block->connect(signal_source_, 0, file_sink_, 0);
            DLOG(INFO) << "connected rtl tcp source to file sink";
        }
}


void RtlTcpSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (samples_ != 0ULL)
        {
            top_block->disconnect(signal_source_, 0, valve_, 0);
            if (dump_)
                {
                    top_block->disconnect(valve_, 0, file_sink_, 0);
                }
        }
    else if (dump_)
        {
            top_block->disconnect(signal_source_, 0, file_sink_, 0);
        }
}


gr::basic_block_sptr RtlTcpSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return {};
}


gr::basic_block_sptr RtlTcpSignalSource::get_right_block()
{
    if (samples_ != 0ULL)
        {
            return valve_;
        }
    return signal_source_;
}
