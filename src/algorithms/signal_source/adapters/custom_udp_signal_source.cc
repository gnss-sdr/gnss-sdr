/*!
 * \file udp_signal_source.cc
 *
 * \brief Receives ip frames containing samples in UDP frame encapsulation
 * using a high performance packet capture library (libpcap)
 * \author Javier Arribas jarribas (at) cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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


#include "custom_udp_signal_source.h"
#include "configuration_interface.h"
#include "GPS_L1_CA.h"
#include <boost/format.hpp>
#include <glog/logging.h>
#include <iostream>


using google::LogMessage;


CustomUDPSignalSource::CustomUDPSignalSource(ConfigurationInterface* configuration,
    std::string role, unsigned int in_stream, unsigned int out_stream,
    boost::shared_ptr<gr::msg_queue> queue) : role_(role), in_stream_(in_stream), out_stream_(out_stream), queue_(queue)
{
    // DUMP PARAMETERS
    std::string empty = "";
    std::string default_dump_file = "./data/signal_source.dat";
    std::string default_item_type = "gr_complex";
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename",
        default_dump_file);

    // network PARAMETERS
    std::string default_capture_device = "eth0";
    std::string default_address = "127.0.0.1";
    int default_port = 1234;
    std::string address = configuration->property(role + ".origin_address", default_address);
    std::string capture_device = configuration->property(role + ".capture_device", default_capture_device);
    int port = configuration->property(role + ".port", default_port);
    int payload_bytes = configuration->property(role + ".payload_bytes", 1024);


    RF_channels_ = configuration->property(role + ".RF_channels", 1);
    channels_in_udp_ = configuration->property(role + ".channels_in_udp", 1);
    IQ_swap_ = configuration->property(role + ".IQ_swap", false);

    std::string default_sample_type = "cbyte";
    std::string sample_type = configuration->property(role + ".sample_type", default_sample_type);
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    //output item size is always gr_complex
    item_size_ = sizeof(gr_complex);

    udp_gnss_rx_source_ = gr_complex_ip_packet_source::make(capture_device,
        address,
        port,
        payload_bytes,
        channels_in_udp_,
        sample_type,
        item_size_,
        IQ_swap_);

    if (channels_in_udp_ >= RF_channels_)
        {
            for (int n = 0; n < channels_in_udp_; n++)
                {
                    null_sinks_.push_back(gr::blocks::null_sink::make(sizeof(gr_complex)));
                }
        }
    else
        {
            std::cout << "Configuration error: RF_channels<channels_in_use" << std::endl;
            exit(0);
        }


    if (dump_)
        {
            for (int n = 0; n < channels_in_udp_; n++)
                {
                    DLOG(INFO) << "Dumping output into file " << (dump_filename_ + "c_h" + std::to_string(n) + ".bin");
                    file_sink_.push_back(gr::blocks::file_sink::make(item_size_, (dump_filename_ + "_ch" + std::to_string(n) + ".bin").c_str()));
                }
        }
}


CustomUDPSignalSource::~CustomUDPSignalSource()
{
}


void CustomUDPSignalSource::connect(gr::top_block_sptr top_block)
{
    //connect null sinks to unused streams
    for (int n = 0; n < channels_in_udp_; n++)
        {
            top_block->connect(udp_gnss_rx_source_, n, null_sinks_.at(n), 0);
        }
    DLOG(INFO) << "connected udp_source to null_sinks to enable the use of spare channels" << std::endl;

    if (dump_)
        {
            for (int n = 0; n < channels_in_udp_; n++)
                {
                    top_block->connect(udp_gnss_rx_source_, n, file_sink_.at(n), 0);
                    DLOG(INFO) << "connected source to file sink";
                }
        }
}


void CustomUDPSignalSource::disconnect(gr::top_block_sptr top_block)
{
    //disconnect null sinks to unused streams
    for (int n = 0; n < channels_in_udp_; n++)
        {
            top_block->disconnect(udp_gnss_rx_source_, n, null_sinks_.at(n), 0);
        }
    if (dump_)
        {
            for (int n = 0; n < channels_in_udp_; n++)
                {
                    top_block->disconnect(udp_gnss_rx_source_, n, file_sink_.at(n), 0);
                    DLOG(INFO) << "disconnected source to file sink";
                }
        }
    DLOG(INFO) << "disconnected udp_source" << std::endl;
}


gr::basic_block_sptr CustomUDPSignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::block_sptr();
}


gr::basic_block_sptr CustomUDPSignalSource::get_right_block()
{
    return udp_gnss_rx_source_;
}

gr::basic_block_sptr CustomUDPSignalSource::get_right_block(__attribute__((unused)) int RF_channel)
{
    return udp_gnss_rx_source_;
}
