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

#include "udp_signal_source.h"
#include "configuration_interface.h"
#include "GPS_L1_CA.h"
#include <boost/format.hpp>
#include <glog/logging.h>
#include <iostream>


using google::LogMessage;


UDPSignalSource::UDPSignalSource(ConfigurationInterface* configuration,
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

    // rtl_tcp PARAMETERS
    std::string default_address = "127.0.0.1";
    int default_port = 1234;

    RF_channels_ = configuration->property(role + ".RF_channels", 1);
    channels_in_udp_= configuration->property(role + ".channels_in_udp", 1);
    IQ_swap_= configuration->property(role + ".IQ_swap", false);

    std::string default_sample_type = "cbyte";
    std::string sample_type = configuration->property(role + ".sample_type", default_sample_type);

    item_type_ = configuration->property(role + ".item_type", default_item_type);
    std::string address = configuration->property(role + ".address", default_address);
    int port = configuration->property(role + ".port", default_port);
    int payload_bytes = configuration->property(role + ".payload_bytes", 1024);

    if (sample_type.compare("cbyte")==0)
    {
        udp_gnss_rx_source_ = make_udp_gnss_rx_source(sizeof(char), address, port, payload_bytes, true);
        demux_=gr::blocks::deinterleave::make(sizeof(char),1);
    }else{
        std::cout<<"WARNING: Requested UDP sample type unsuported, setting sample type to cbyte\n";
        udp_gnss_rx_source_ = make_udp_gnss_rx_source(sizeof(char), address, port, payload_bytes, true);
        demux_=gr::blocks::deinterleave::make(sizeof(char),1);
    }

    //create I, Q -> gr_complex type conversion blocks
    for (int n = 0; n < (channels_in_udp_ * 2); n++)
        {
            char_to_float_.push_back(gr::blocks::char_to_float::make());
        }

    for (int n = 0; n < channels_in_udp_; n++)
        {
            float_to_complex_.push_back(gr::blocks::float_to_complex::make());
        }

    if (channels_in_udp_>=RF_channels_)
    {
        for (int n = 0; n < (channels_in_udp_-RF_channels_); n++)
        {
            null_sinks_.push_back(gr::blocks::null_sink::make(sizeof(gr_complex)));
        }
    }else
    {
        std::cout<<"Configuration error: RF_channels<channels_in_use"<<std::endl;
        exit(0);
    }

    //output item size is always gr_complex
    item_size_ = sizeof(gr_complex);
    if (dump_)
        {
            for (int n = 0; n < channels_in_udp_; n++)
            {
                DLOG(INFO) << "Dumping output into file " << (dump_filename_+"c_h"+std::to_string(n)+".bin");
                file_sink_.push_back(gr::blocks::file_sink::make(item_size_, (dump_filename_+"_ch"+std::to_string(n)+".bin").c_str()));
            }
        }

}


UDPSignalSource::~UDPSignalSource()
{
}


void UDPSignalSource::connect(gr::top_block_sptr top_block)
{
    top_block->connect(udp_gnss_rx_source_,0, demux_,0);
    DLOG(INFO)<<"connected udp_source to demux"<<std::endl;
    for (int n = 0; n < (channels_in_udp_ * 2); n++)
        {

            top_block->connect(demux_, n, char_to_float_.at(n), 0);
            DLOG(INFO) << "connected demux to char_to_float CH" << n;
        }
    for (int n = 0; n < channels_in_udp_; n++)
        {
            if (!IQ_swap_)
            {
                top_block->connect(char_to_float_.at(n * 2), 0, float_to_complex_.at(n), 0);
                top_block->connect(char_to_float_.at(n * 2 + 1), 0, float_to_complex_.at(n), 1);
            }
            else
            {
                top_block->connect(char_to_float_.at(n * 2), 0, float_to_complex_.at(n), 1);
                top_block->connect(char_to_float_.at(n * 2 + 1), 0, float_to_complex_.at(n), 0);
            }
            DLOG(INFO) << "connected char_to_float to float_to_complex_ CH" << n;
        }

    //connect null sinks to unused streams
    if (channels_in_udp_>RF_channels_)
    {
        for (int n = 0; n < (channels_in_udp_-RF_channels_); n++)
        {
            top_block->connect(float_to_complex_.at(RF_channels_+n),0,null_sinks_.at(n),0);
        }
    }

    if (dump_)
        {
            for (int n = 0; n < channels_in_udp_; n++)
            {
                top_block->connect(float_to_complex_.at(n), 0, file_sink_.at(n), 0);
                DLOG(INFO) << "connected source to file sink";
            }
        }
}


void UDPSignalSource::disconnect(gr::top_block_sptr top_block)
{

    for (int n = 0; n < (channels_in_udp_ * 2); n++)
        {
            top_block->disconnect(demux_, n, char_to_float_.at(n), 0);
            DLOG(INFO) << "disconnect demux to char_to_float CH" << n;
        }
    for (int n = 0; n < channels_in_udp_; n++)
        {
            if (!IQ_swap_)
            {
                top_block->disconnect(char_to_float_.at(n * 2), 0, float_to_complex_.at(n), 0);
                top_block->disconnect(char_to_float_.at(n * 2 + 1), 0, float_to_complex_.at(n), 1);
            }
            else
            {
                top_block->disconnect(char_to_float_.at(n * 2), 0, float_to_complex_.at(n), 1);
                top_block->disconnect(char_to_float_.at(n * 2 + 1), 0, float_to_complex_.at(n), 0);
            }
            DLOG(INFO) << "disconnect char_to_float to float_to_complex_ CH" << n;
        }

    //disconnect null sinks to unused streams
    if (channels_in_udp_>RF_channels_)
    {
        for (int n = 0; n < (channels_in_udp_-RF_channels_); n++)
        {
            top_block->disconnect(float_to_complex_.at(RF_channels_+n),0,null_sinks_.at(n),0);
        }
    }


    if (dump_)
        {
        for (int n = 0; n < channels_in_udp_; n++)
        {
            top_block->disconnect(float_to_complex_.at(n), 0, file_sink_.at(n), 0);
            DLOG(INFO) << "disconnected source to file sink";
        }
        }
    top_block->disconnect(udp_gnss_rx_source_,0, demux_,0);
    DLOG(INFO)<<"disconnected udp_source to demux"<<std::endl;
}


gr::basic_block_sptr UDPSignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::block_sptr();
}


gr::basic_block_sptr UDPSignalSource::get_right_block()
{
    return get_right_block(0);
}

gr::basic_block_sptr UDPSignalSource::get_right_block(int RF_channel)
{
    return float_to_complex_.at(RF_channel);
}

