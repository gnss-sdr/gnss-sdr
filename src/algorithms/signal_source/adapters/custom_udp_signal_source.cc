/*!
 * \file custom_udp_signal_source.cc
 * \brief Receives ip frames containing samples in UDP frame encapsulation
 * using a high performance packet capture library (libpcap)
 * \author Javier Arribas jarribas (at) cttc.es
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


#include "custom_udp_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_string_literals.h"
#include <glog/logging.h>
#include <iostream>


using namespace std::string_literals;

CustomUDPSignalSource::CustomUDPSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_stream,
    unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue __attribute__((unused)))
    : SignalSourceBase(configuration, role, "Custom_UDP_Signal_Source"s),
      item_size_(sizeof(gr_complex)),
      RF_channels_(configuration->property(role + ".RF_channels", 1)),
      channels_in_udp_(configuration->property(role + ".channels_in_udp", 1)),
      in_stream_(in_stream),
      out_stream_(out_stream),
      IQ_swap_(configuration->property(role + ".IQ_swap", false)),
      dump_(configuration->property(role + ".dump", false))
{
    // DUMP PARAMETERS
    const std::string default_dump_file("./data/signal_source.dat");
    const std::string default_item_type("gr_complex");
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    // network PARAMETERS
    const std::string default_capture_device("eth0");
    const std::string default_address("127.0.0.1");
    const int default_port = 1234;
    const std::string address = configuration->property(role + ".origin_address", default_address);
    std::string capture_device = configuration->property(role + ".capture_device", default_capture_device);
    int port = configuration->property(role + ".port", default_port);
    int payload_bytes = configuration->property(role + ".payload_bytes", 1024);

    const std::string default_sample_type("cbyte");
    const std::string sample_type = configuration->property(role + ".sample_type", default_sample_type);
    item_type_ = configuration->property(role + ".item_type", default_item_type);

    udp_gnss_rx_source_ = Gr_Complex_Ip_Packet_Source::make(capture_device,
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
                    null_sinks_.emplace_back(gr::blocks::null_sink::make(sizeof(gr_complex)));
                }
        }
    else
        {
            std::cout << "Configuration error: RF_channels<channels_in_use\n";
            exit(0);
        }

    if (dump_)
        {
            for (int n = 0; n < channels_in_udp_; n++)
                {
                    DLOG(INFO) << "Dumping output into file " << (dump_filename_ + "c_h" + std::to_string(n) + ".bin");
                    file_sink_.emplace_back(gr::blocks::file_sink::make(item_size_, (dump_filename_ + "_ch" + std::to_string(n) + ".bin").c_str()));
                }
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


void CustomUDPSignalSource::connect(gr::top_block_sptr top_block)
{
    // connect null sinks to unused streams
    for (int n = 0; n < channels_in_udp_; n++)
        {
            top_block->connect(udp_gnss_rx_source_, n, null_sinks_.at(n), 0);
        }
    DLOG(INFO) << "connected udp_source to null_sinks to enable the use of spare channels\n";

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
    // disconnect null sinks to unused streams
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
    DLOG(INFO) << "disconnected udp_source\n";
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
