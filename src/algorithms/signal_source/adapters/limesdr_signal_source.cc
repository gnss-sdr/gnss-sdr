/*!
 * \file limesdr_signal_source.cc
 * \brief Signal source for LimeSDR front-end
 * \author Javier Arribas, 2021. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "limesdr_signal_source.h"
#include "configuration_interface.h"
#include "gnss_sdr_valve.h"
#include <glog/logging.h>
#include <gnuradio/blocks/file_sink.h>
#include <iostream>
#include <utility>


LimesdrSignalSource::LimesdrSignalSource(const ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_stream, unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue) : role_(role), in_stream_(in_stream), out_stream_(out_stream)
{
    // DUMP PARAMETERS
    const std::string empty;
    const std::string default_dump_file("./data/signal_source.dat");
    const std::string default_item_type("gr_complex");
    samples_ = configuration->property(role + ".samples", static_cast<int64_t>(0));
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename",
        default_dump_file);

    // Driver parameters
    channel_ = configuration->property(role + ".channel", 0);
    //AGC_enabled_ = configuration->property(role + ".AGC_enabled", true);
    freq_ = configuration->property(role + ".freq", 1575420000);
    gain_ = configuration->property(role + ".gain", 40.0);
    sample_rate_ = configuration->property(role + ".sampling_frequency", 2.0e6);
    //todo: check aif bw is within limits
    analog_bw_hz_ = configuration->property(role + ".analog_bw", sample_rate_);
    digital_bw_hz_ = configuration->property(role + ".digital_bw", sample_rate_);
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    limesdr_serial_ = configuration->property(role + ".limesdr_serial", std::string());
    limesdr_file_ = configuration->property(role + ".limesdr_file", std::string());
    antenna_ = configuration->property(role + ".antenna", 255);

    PPS_mode_ = configuration->property(role + ".PPS_mode", false);
    //channel_mode Channel and mode selection A(1), B(2), (A+B)MIMO(3).
    limechannel_mode_ = configuration->property(role + ".limechannel_mode", 1);


    if (item_type_ == "short")
        {
            item_size_ = sizeof(int16_t);
        }
    else if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            //1. Make the driver instance

            try
                {
#ifdef LimeSDR_PPS
                    limesdr_source_ = gr::limesdr::source::make(limesdr_serial_, limechannel_mode_, limesdr_file_, PPS_mode_);
#else
                    limesdr_source_ = gr::limesdr::source::make(limesdr_serial_, limechannel_mode_, limesdr_file_);
#endif
                }
            catch (const boost::exception& e)
                {
                    LOG(WARNING) << "Boost exception: " << boost::diagnostic_information(e);
                    throw std::invalid_argument("Wrong LimeSDR arguments");
                }

            // For LimeSDR: Set RX antenna
            /**
             * Set which antenna is used
             *
             * @param   antenna Antenna to set: None(0), LNAH(1), LNAL(2), LNAW(3), AUTO(255)
             *
             * @param   channel  Channel selection: A(LMS_CH_0),B(LMS_CH_1).
             */

            limesdr_source_->set_antenna(antenna_, channel_);
            // 2 set sampling rate
            double actual_sample_rate = limesdr_source_->set_sample_rate(sample_rate_);
            std::cout << "Actual RX Rate: " << actual_sample_rate << " [SPS]...\n";
            LOG(INFO) << "Actual RX Rate: " << actual_sample_rate << " [SPS]...";

            // 3. set rx frequency
            double actual_center_freq = limesdr_source_->set_center_freq(freq_);

            std::cout << "Actual RX Freq: " << actual_center_freq << " [Hz]...\n";
            LOG(INFO) << "Actual RX Freq: " << actual_center_freq << " [Hz]...";

            // TODO: Assign the remnant IF from the PLL tune error
            std::cout << "PLL Frequency tune error: " << actual_center_freq - freq_ << " [Hz]...\n";
            LOG(INFO) << "PLL Frequency tune error: " << actual_center_freq - freq_ << " [Hz]...\n";


            // TODO: gr-limesdr does not report PLL tune frequency error...

            // 4. set rx gain
            //todo: gr-limesdr does not expose AGC controls..
            //            if (AGC_enabled_ == true)
            //                {
            //                    osmosdr_source_->set_gain_mode(true);
            //                    std::cout << "AGC enabled\n";
            //                    LOG(INFO) << "AGC enabled";
            //                }
            //            else
            //                {
            double actual_gain = limesdr_source_->set_gain(gain_, channel_);
            std::cout << "Actual RX Gain: " << actual_gain << " [dB]...\n";
            LOG(INFO) << "Actual RX Gain: " << actual_gain << " [dB]...";

            // Set analog bandwidth
            double actual_analog_bw = limesdr_source_->set_bandwidth(analog_bw_hz_, channel_);
            std::cout << "Actual Analog Bandwidth: " << actual_analog_bw << " [Hz]...\n";

            // Set digital bandwidth
            limesdr_source_->set_digital_filter(digital_bw_hz_, channel_);
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


void LimesdrSignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->connect(limesdr_source_, 0, valve_, 0);
            DLOG(INFO) << "connected limesdr source to valve";
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
                    top_block->connect(limesdr_source_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected limesdr source to file sink";
                }
        }
}


void LimesdrSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->disconnect(limesdr_source_, 0, valve_, 0);
            if (dump_)
                {
                    top_block->disconnect(valve_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (dump_)
                {
                    top_block->disconnect(limesdr_source_, 0, file_sink_, 0);
                }
        }
}


gr::basic_block_sptr LimesdrSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return gr::basic_block_sptr();
}


gr::basic_block_sptr LimesdrSignalSource::get_right_block()
{
    if (samples_ != 0)
        {
            return valve_;
        }
    else
        {
            return limesdr_source_;
        }
}
