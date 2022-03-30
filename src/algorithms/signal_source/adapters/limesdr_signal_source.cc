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
#include "gnss_frequencies.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_valve.h"
#include <boost/exception/diagnostic_information.hpp>
#include <glog/logging.h>
#include <gnuradio/blocks/file_sink.h>
#include <iostream>

using namespace std::string_literals;

LimesdrSignalSource::LimesdrSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_stream,
    unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "Limesdr_Signal_Source"s),
      item_type_(configuration->property(role + ".item_type", std::string("gr_complex"))),
      dump_filename_(configuration->property(role + ".dump_filename", std::string("./data/signal_source.dat"))),
      limesdr_serial_(configuration->property(role + ".limesdr_serial", std::string())),
      limesdr_file_(configuration->property(role + ".limesdr_file", std::string())),
      sample_rate_(configuration->property(role + ".sampling_frequency", 2.0e6)),
      freq_(configuration->property(role + ".freq", FREQ1)),
      gain_(configuration->property(role + ".gain", 40.0)),
      analog_bw_hz_(configuration->property(role + ".analog_bw", sample_rate_ / 2.0)),  // LPF analog filters in I,Q branches
      digital_bw_hz_(configuration->property(role + ".digital_bw", 0.0)),               // disable by default
      ext_clock_MHz_(configuration->property(role + ".ext_clock_MHz", 0.0)),            // external clock: 0.0 MHz will enable the internal clock
      samples_(configuration->property(role + ".samples", int64_t(0))),
      in_stream_(in_stream),
      out_stream_(out_stream),
      limechannel_mode_(configuration->property(role + ".limechannel_mode", 0)),
      antenna_(configuration->property(role + ".antenna", 255)),
      channel_(configuration->property(role + ".channel", 0)),
      PPS_mode_(configuration->property(role + ".PPS_mode", false)),
      dump_(configuration->property(role + ".dump", false))
{
    if ((limechannel_mode_ < 0) || (limechannel_mode_ > 2))
        {
            std::cerr << "ERROR: source_impl::source_impl(): ChannelMode must be A(0), B(1) or (A+B) MIMO(2)\n";
            exit(0);
        }

    if (item_type_ == "short")
        {
            item_size_ = sizeof(int16_t);
        }
    else if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            // 1. Make the driver instance
            try
                {
#ifdef LimeSDR_PPS
#ifdef GR_LIMESDR_IS_G38_BRANCH
                    limesdr_source_ = gr::limesdr::source::make(limesdr_serial_, limechannel_mode_, limesdr_file_, false, PPS_mode_);
#else
                    limesdr_source_ = gr::limesdr::source::make(limesdr_serial_, limechannel_mode_, limesdr_file_, PPS_mode_);
#endif
                    if (ext_clock_MHz_ != 0.0)
                        {
                            if (limesdr_source_->set_ext_clk(ext_clock_MHz_))
                                {
                                    std::cout << "External clock enabled with expected frequency input of " << ext_clock_MHz_ << " MHz\n";
                                }
                            else
                                {
                                    std::cout << "Error setting external reference clock\n";
                                }
                        }
                    else
                        {
                            limesdr_source_->disable_ext_clk();
                        }
#else  // LimeSDR_PPS
#ifdef GR_LIMESDR_IS_G38_BRANCH
                    limesdr_source_ = gr::limesdr::source::make(limesdr_serial_, limechannel_mode_, limesdr_file_, false);
#else
                    limesdr_source_ = gr::limesdr::source::make(limesdr_serial_, limechannel_mode_, limesdr_file_);
#endif
#endif  // LimeSDR_PPS
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
            std::cout << "LimeSDR RX antenna set to " << antenna_ << " for channel " << channel_ << '\n';
            LOG(INFO) << "LimeSDR RX antenna set to " << antenna_ << " for channel " << channel_;

            // 2. set sampling rate
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
            double actual_gain = limesdr_source_->set_gain(gain_, channel_);
            std::cout << "Actual RX Gain: " << actual_gain << " [dB]...\n";
            LOG(INFO) << "Actual RX Gain: " << actual_gain << " [dB]...";

            // Set analog bandwidth
            double current_analog_bw = limesdr_source_->set_bandwidth(analog_bw_hz_, channel_);
            std::cout << "Actual Analog Bandwidth: " << current_analog_bw << " [Hz]...\n";
            LOG(INFO) << "Actual Analog Bandwidth: : " << current_analog_bw << " [Hz]...";

            // Set digital bandwidth
            limesdr_source_->set_digital_filter(digital_bw_hz_, channel_);

            limesdr_source_->calibrate(sample_rate_ / 2, channel_);
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
    return {};
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
