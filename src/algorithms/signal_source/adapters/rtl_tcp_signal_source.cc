/*!
 * \file rtl_tcp_signal_source.cc
 * \brief Signal source for the Realtek RTL2832U USB dongle DVB-T receiver
 *        over TCP.
 * (see http://sdr.osmocom.org/trac/wiki/rtl-sdr for more information)
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "rtl_tcp_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_valve.h"
#include "GPS_L1_CA.h"
#include <boost/format.hpp>
#include <glog/logging.h>
#include <iostream>


using google::LogMessage;


RtlTcpSignalSource::RtlTcpSignalSource(ConfigurationInterface* configuration,
    std::string role, unsigned int in_stream, unsigned int out_stream,
    boost::shared_ptr<gr::msg_queue> queue) : role_(role), in_stream_(in_stream), out_stream_(out_stream), queue_(queue)
{
    // DUMP PARAMETERS
    std::string empty = "";
    std::string default_dump_file = "./data/signal_source.dat";
    std::string default_item_type = "gr_complex";
    samples_ = configuration->property(role + ".samples", 0);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename",
        default_dump_file);

    // rtl_tcp PARAMETERS
    std::string default_address = "127.0.0.1";
    short default_port = 1234;
    AGC_enabled_ = configuration->property(role + ".AGC_enabled", true);
    freq_ = configuration->property(role + ".freq", GPS_L1_FREQ_HZ);
    gain_ = configuration->property(role + ".gain", 40.0);
    rf_gain_ = configuration->property(role + ".rf_gain", 40.0);
    if_gain_ = configuration->property(role + ".if_gain", 40.0);
    sample_rate_ = configuration->property(role + ".sampling_frequency", 2.0e6);
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    address_ = configuration->property(role + ".address", default_address);
    port_ = configuration->property(role + ".port", default_port);
    flip_iq_ = configuration->property(role + ".flip_iq", false);

    if (item_type_.compare("short") == 0)
        {
            item_size_ = sizeof(short);
        }
    else if (item_type_.compare("gr_complex") == 0)
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
                    std::cout << "AGC enabled" << std::endl;
                    LOG(INFO) << "AGC enabled";
                    signal_source_->set_agc_mode(true);
                }
            else
                {
                    std::cout << "AGC disabled" << std::endl;
                    LOG(INFO) << "AGC disabled";
                    signal_source_->set_agc_mode(false);

                    std::cout << "Setting gain to " << gain_ << std::endl;
                    LOG(INFO) << "Setting gain to " << gain_;
                    signal_source_->set_gain(gain_);

                    std::cout << "Setting IF gain to " << if_gain_ << std::endl;
                    LOG(INFO) << "Setting IF gain to " << if_gain_;
                    signal_source_->set_if_gain(if_gain_);
                }
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type. Using short.";
            item_size_ = sizeof(short);
        }

    if (samples_ != 0)
        {
            DLOG(INFO) << "Send STOP signal after " << samples_ << " samples";
            valve_ = gnss_sdr_make_valve(item_size_, samples_, queue_);
            DLOG(INFO) << "valve(" << valve_->unique_id() << ")";
        }

    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }
}


RtlTcpSignalSource::~RtlTcpSignalSource()
{
}


void RtlTcpSignalSource::MakeBlock()
{
    try
        {
            std::cout << "Connecting to " << address_ << ":" << port_ << std::endl;
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
    if (samples_)
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
    if (samples_)
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
    return gr::basic_block_sptr();
}


gr::basic_block_sptr RtlTcpSignalSource::get_right_block()
{
    if (samples_ != 0)
        {
            return valve_;
        }
    else
        {
            return signal_source_;
        }
}
