/*!
 * \file ad936x_custom_signal_source.cc
 * \brief A direct IIO custom front-end gnss-sdr signal source for the AD936x AD front-end family with special FPGA custom functionalities.
 * \author Javier Arribas, jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "ad936x_custom_signal_source.h"
#include "configuration_interface.h"
#include "gnss_frequencies.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_valve.h"
#include <boost/exception/diagnostic_information.hpp>
#include <glog/logging.h>
#include <gnuradio/blocks/file_sink.h>
#include <iostream>

using namespace std::string_literals;

Ad936xCustomSignalSource::Ad936xCustomSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_stream,
    unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue __attribute__((unused)))
    : SignalSourceBase(configuration, role, "Ad936x_Custom_Signal_Source"s),
      in_stream_(in_stream),
      out_stream_(out_stream),
      item_type_(configuration->property(role + ".item_type", std::string("gr_complex"))),
      samples_(configuration->property(role + ".samples", int64_t(0))),
      dump_(configuration->property(role + ".dump", false)),
      dump_filename_(configuration->property(role + ".dump_filename", std::string("./data/signal_source.dat"))),
      pluto_uri_(configuration->property(role + ".pluto_uri", std::string("local"))),
      board_type_(configuration->property(role + ".board_type", std::string("single_ad9361"))),
      sample_rate_(configuration->property(role + ".sampling_frequency", 4.0e6)),
      bandwidth_(configuration->property(role + ".bandwidth", configuration->property(role + ".sampling_frequency", 4.0e6) / 1.1)),
      freq_(configuration->property(role + ".freq", FREQ1)),
      freq_2ch(configuration->property(role + ".freq_2ch", FREQ1)),
      rf_port_select_(configuration->property(role + ".rf_port_select", std::string("A_BALANCED"))),
      rf_filter(configuration->property(role + ".rf_filter", std::string("none"))),
      gain_mode_rx0_(configuration->property(role + ".gain_mode_rx0", std::string("slow_attack"))),
      gain_mode_rx1_(configuration->property(role + ".gain_mode_rx1", std::string("slow_attack"))),
      rf_gain_rx0_(configuration->property(role + ".gain_rx0", 40.0)),
      rf_gain_rx1_(configuration->property(role + ".gain_rx1", 40.0)),
      enable_ch0(configuration->property(role + ".enable_ch0", true)),
      enable_ch1(configuration->property(role + ".enable_ch1", false)),
      PPS_mode_(configuration->property(role + ".PPS_mode", false)),
      fe_ip_(configuration->property(role + ".fe_ip", std::string("192.168.2.1"))),
      fe_ctlport_(configuration->property(role + ".fe_ctlport", int32_t(10000))),
      ssize_(configuration->property(role + ".ssize", int32_t(16))),
      bshift_(configuration->property(role + ".bshift", int64_t(0))),
      spattern_(configuration->property(role + ".spattern", false)),
      inverted_spectrum_ch0_(configuration->property(role + ".inverted_spectrum_ch0", false)),
      inverted_spectrum_ch1_(configuration->property(role + ".inverted_spectrum_ch1", false))


{
    if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            // 1. Make the driver instance
            bool customsamplesize = false;
            if (ssize_ != 16 or spattern_ == true) customsamplesize = true;

            ad936x_iio_source = ad936x_iio_make_source_sptr(
                pluto_uri_,
                board_type_,
                bandwidth_,
                sample_rate_,
                freq_,
                rf_port_select_,
                rf_filter,
                gain_mode_rx0_,
                gain_mode_rx1_,
                rf_gain_rx0_,
                rf_gain_rx1_,
                enable_ch0,
                enable_ch1,
                freq_2ch,
                PPS_mode_,
                customsamplesize,
                fe_ip_,
                fe_ctlport_,
                ssize_,
                bshift_,
                spattern_);

            n_channels = 1;
            if (enable_ch0 == true and enable_ch1 == true)
                {
                    n_channels = 2;
                }

            for (int n = 0; n < n_channels; n++)
                {
                    if (ssize_ == 16)
                        {
                            gr_interleaved_short_to_complex_.push_back(gr::blocks::interleaved_short_to_complex::make());
                        }
                    else if (ssize_ == 8)
                        {
                            gr_interleaved_char_to_complex_.push_back(gr::blocks::interleaved_char_to_complex::make());
                        }
                    else if (ssize_ == 4)
                        {
                            gr_interleaved_short_to_complex_.push_back(gr::blocks::interleaved_short_to_complex::make(false, false));
                            unpack_byte_fourbits.push_back(make_unpack_byte_4bit_samples());
                        }
                    else if (ssize_ == 2)
                        {
                            gr_interleaved_short_to_complex_.push_back(gr::blocks::interleaved_short_to_complex::make(false, false));
                            unpack_byte_twobits.push_back(make_unpack_byte_2bit_cpx_samples());
                        }
                }
        }
    else
        {
            LOG(ERROR) << item_type_ << " unrecognized item type";
            exit(1);
        }


    if (dump_)
        {
            for (int n = 0; n < n_channels; n++)
                {
                    DLOG(INFO) << "Dumping output into file " << (dump_filename_ + "c_h" + std::to_string(n) + ".bin");
                    sink_.emplace_back(gr::blocks::file_sink::make(item_size_, (dump_filename_ + "_ch" + std::to_string(n) + ".bin").c_str()));
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


void Ad936xCustomSignalSource::connect(gr::top_block_sptr top_block)
{
    for (int n = 0; n < n_channels; n++)
        {
            if (ssize_ == 16)
                {
                    top_block->connect(ad936x_iio_source, n, gr_interleaved_short_to_complex_.at(n), 0);
                    DLOG(INFO) << "connected ad936x_iio_source source to gr_interleaved_short_to_complex for channel " << n;
                    if (dump_)
                        {
                            top_block->connect(gr_interleaved_short_to_complex_.at(n), 0, sink_.at(n), 0);
                            DLOG(INFO) << "connected source to file sink";
                        }
                }
            else if (ssize_ == 8)
                {
                    top_block->connect(ad936x_iio_source, n, gr_interleaved_char_to_complex_.at(n), 0);
                    DLOG(INFO) << "connected ad936x_iio_source source to gr_interleaved_char_to_complex_ for channel " << n;
                    if (dump_)
                        {
                            top_block->connect(gr_interleaved_char_to_complex_.at(n), 0, sink_.at(n), 0);
                            DLOG(INFO) << "connected source to file sink";
                        }
                }
            else if (ssize_ == 4)
                {
                    top_block->connect(ad936x_iio_source, n, unpack_byte_fourbits.at(n), 0);
                    top_block->connect(unpack_byte_fourbits.at(n), 0, gr_interleaved_short_to_complex_.at(n), 0);
                    DLOG(INFO) << "connected ad936x_iio_source source to unpack_byte_fourbits for channel " << n;
                    if (dump_)
                        {
                            top_block->connect(gr_interleaved_short_to_complex_.at(n), 0, sink_.at(n), 0);
                            DLOG(INFO) << "connected source to file sink";
                        }
                }
            else if (ssize_ == 2)
                {
                    top_block->connect(ad936x_iio_source, n, unpack_byte_twobits.at(n), 0);
                    top_block->connect(unpack_byte_twobits.at(n), 0, gr_interleaved_short_to_complex_.at(n), 0);
                    DLOG(INFO) << "connected ad936x_iio_source source to unpack_byte_fourbits for channel " << n;
                    if (dump_)
                        {
                            top_block->connect(gr_interleaved_short_to_complex_.at(n), 0, sink_.at(n), 0);
                            DLOG(INFO) << "connected source to file sink";
                        }
                }
            else
                {
                    top_block->connect(ad936x_iio_source, n, gr_interleaved_short_to_complex_.at(n), 0);
                    DLOG(INFO) << "connected ad936x_iio_source source to gr_interleaved_short_to_complex for channel " << n;
                    if (dump_)
                        {
                            top_block->connect(gr_interleaved_short_to_complex_.at(n), 0, sink_.at(n), 0);
                            DLOG(INFO) << "connected source to file sink";
                        }
                }
        }
}


void Ad936xCustomSignalSource::disconnect(gr::top_block_sptr top_block)
{
    for (int n = 0; n < n_channels; n++)
        {
            if (ssize_ == 16)
                {
                    top_block->disconnect(ad936x_iio_source, n, gr_interleaved_short_to_complex_.at(n), 0);
                    DLOG(INFO) << "connected ad936x_iio_source source to gr_interleaved_short_to_complex for channel " << n;
                    if (dump_)
                        {
                            top_block->disconnect(gr_interleaved_short_to_complex_.at(n), 0, sink_.at(n), 0);
                            DLOG(INFO) << "connected source to file sink";
                        }
                }
            else if (ssize_ == 8)
                {
                    top_block->disconnect(ad936x_iio_source, n, gr_interleaved_char_to_complex_.at(n), 0);
                    DLOG(INFO) << "connected ad936x_iio_source source to gr_interleaved_char_to_complex_ for channel " << n;
                    if (dump_)
                        {
                            top_block->disconnect(gr_interleaved_char_to_complex_.at(n), 0, sink_.at(n), 0);
                            DLOG(INFO) << "connected source to file sink";
                        }
                }
            else if (ssize_ == 4)
                {
                    top_block->disconnect(ad936x_iio_source, n, unpack_byte_fourbits.at(n), 0);
                    top_block->disconnect(unpack_byte_fourbits.at(n), 0, gr_interleaved_short_to_complex_.at(n), 0);
                    DLOG(INFO) << "connected ad936x_iio_source source to unpack_byte_fourbits for channel " << n;
                    if (dump_)
                        {
                            top_block->connect(gr_interleaved_short_to_complex_.at(n), 0, sink_.at(n), 0);
                            DLOG(INFO) << "connected source to file sink";
                        }
                }
            else if (ssize_ == 2)
                {
                    top_block->disconnect(ad936x_iio_source, n, unpack_byte_twobits.at(n), 0);
                    top_block->disconnect(unpack_byte_twobits.at(n), 0, gr_interleaved_short_to_complex_.at(n), 0);
                    DLOG(INFO) << "connected ad936x_iio_source source to unpack_byte_fourbits for channel " << n;
                    if (dump_)
                        {
                            top_block->disconnect(gr_interleaved_short_to_complex_.at(n), 0, sink_.at(n), 0);
                            DLOG(INFO) << "connected source to file sink";
                        }
                }
            else
                {
                    top_block->disconnect(ad936x_iio_source, n, gr_interleaved_short_to_complex_.at(n), 0);
                    DLOG(INFO) << "connected ad936x_iio_source source to gr_interleaved_short_to_complex for channel " << n;
                    if (dump_)
                        {
                            top_block->disconnect(gr_interleaved_short_to_complex_.at(n), 0, sink_.at(n), 0);
                            DLOG(INFO) << "connected source to file sink";
                        }
                }
        }
}


gr::basic_block_sptr Ad936xCustomSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return {};
}


gr::basic_block_sptr Ad936xCustomSignalSource::get_right_block()
{
    if (ssize_ == 16)
        {
            return gr_interleaved_short_to_complex_.at(0);
        }
    else if (ssize_ == 8)
        {
            return gr_interleaved_char_to_complex_.at(0);
        }
    else if (ssize_ == 4)
        {
            return gr_interleaved_short_to_complex_.at(0);
        }
    else if (ssize_ == 2)
        {
            return gr_interleaved_short_to_complex_.at(0);
        }
    else
        {
            return gr_interleaved_short_to_complex_.at(0);
        }
}

gr::basic_block_sptr Ad936xCustomSignalSource::get_right_block(int RF_channel)
{
    if (ssize_ == 16)
        {
            return gr_interleaved_short_to_complex_.at(RF_channel);
        }
    else if (ssize_ == 8)
        {
            return gr_interleaved_char_to_complex_.at(RF_channel);
        }
    else if (ssize_ == 4)
        {
            return gr_interleaved_short_to_complex_.at(RF_channel);
        }
    else if (ssize_ == 2)
        {
            return gr_interleaved_short_to_complex_.at(RF_channel);
        }
    else
        {
            return gr_interleaved_short_to_complex_.at(RF_channel);
        }
}
