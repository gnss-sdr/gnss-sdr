/*!
 * \file plutosdr_signal_source.cc
 * \brief Signal source for PlutoSDR
 * \author Rodrigo Muñoz, 2017, rmunozl(at)inacap.cl, rodrigo.munoz(at)proteinlab.cl
 *
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

#include "plutosdr_signal_source.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_valve.h"
#include <glog/logging.h>
#include <iostream>


using namespace std::string_literals;


PlutosdrSignalSource::PlutosdrSignalSource(const ConfigurationInterface* configuration,
    const std::string& role, unsigned int in_stream, unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "Plutosdr_Signal_Source"s),
      dump_filename_(configuration->property(role + ".dump_filename", std::string("./data/signal_source.dat"))),
      uri_(configuration->property(role + ".device_address", std::string("192.168.2.1"))),
      gain_mode_(configuration->property(role + ".gain_mode", default_gain_mode)),
      filter_file_(configuration->property(role + ".filter_file", std::string(""))),
      filter_filename_(configuration->property(role + ".filter_filename", filter_file_)),
      item_type_(configuration->property(role + ".item_type", std::string("gr_complex"))),
      rf_gain_(configuration->property(role + ".gain", 50.0)),
      samples_(configuration->property(role + ".samples", static_cast<int64_t>(0))),
      freq_(configuration->property(role + ".freq", static_cast<uint64_t>(GPS_L1_FREQ_HZ))),
      sample_rate_(configuration->property(role + ".sampling_frequency", static_cast<uint64_t>(3000000))),
      bandwidth_(configuration->property(role + ".bandwidth", static_cast<uint64_t>(2000000))),
      buffer_size_(configuration->property(role + ".buffer_size", 0xA0000)),
      item_size_(sizeof(gr_complex)),
      Fpass_(configuration->property(role + ".Fpass", 0.0)),
      Fstop_(configuration->property(role + ".Fstop", 0.0)),
      in_stream_(in_stream),
      out_stream_(out_stream),
      quadrature_(configuration->property(role + ".quadrature", true)),
      rf_dc_(configuration->property(role + ".rf_dc", true)),
      bb_dc_(configuration->property(role + ".bb_dc", true)),
      filter_auto_(configuration->property(role + ".filter_auto", false)),
      dump_(configuration->property(role + ".dump", false))
{
    if (filter_auto_)
        {
            filter_source_ = configuration->property(role + ".filter_source", std::string("Auto"));
        }
    else
        {
            filter_source_ = configuration->property(role + ".filter_source", std::string("Off"));
        }

    if (item_type_ != "gr_complex")
        {
            std::cout << "Configuration error: item_type must be gr_complex\n";
            LOG(FATAL) << "Configuration error: item_type must be gr_complex!";
        }

    // basic check
    if ((gain_mode_ != "manual") && (gain_mode_ != "slow_attack") && (gain_mode_ != "fast_attack") && (gain_mode_ != "hybrid"))
        {
            std::cout << "Configuration parameter gain_mode should take one of these values:\n";
            std::cout << " manual, slow_attack, fast_attack, hybrid\n";
            std::cout << "Error: provided value gain_mode=" << gain_mode_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value gain_mode=" << default_gain_mode << '\n';
            gain_mode_ = default_gain_mode;
            LOG(WARNING) << "Invalid configuration value for gain_mode parameter. Set to gain_mode=" << default_gain_mode;
        }

    if (gain_mode_ == "manual")
        {
            if (rf_gain_ > 73.0 || rf_gain_ < -1.0)
                {
                    std::cout << "Configuration parameter rf_gain should take values between -1.0 and 73 dB\n";
                    std::cout << "Error: provided value rf_gain=" << rf_gain_ << " is not among valid values\n";
                    std::cout << " This parameter has been set to its default value rf_gain=64.0\n";
                    rf_gain_ = 64.0;
                    LOG(WARNING) << "Invalid configuration value for rf_gain parameter. Set to rf_gain=64.0";
                }
        }

    if ((filter_source_ != "Off") && (filter_source_ != "Auto") && (filter_source_ != "File") && (filter_source_ != "Design"))
        {
            std::cout << "Configuration parameter filter_source should take one of these values:\n";
            std::cout << "  Off: Disable filter\n";
            std::cout << "  Auto: Use auto-generated filters\n";
            std::cout << "  File: User-provided filter in filter_filename parameter\n";
#if LIBAD9361_VERSION_GREATER_THAN_01
            std::cout << "  Design: Create filter from Fpass, Fstop, sampling_frequency and bandwidth parameters\n";
#endif
            std::cout << "Error: provided value filter_source=" << filter_source_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value filter_source=Off\n";
            filter_source_ = std::string("Off");
            LOG(WARNING) << "Invalid configuration value for filter_source parameter. Set to filter_source=Off";
        }

    if (bandwidth_ < 200000 || bandwidth_ > 56000000)
        {
            std::cout << "Configuration parameter bandwidth should take values between 200000 and 56000000 Hz\n";
            std::cout << "Error: provided value bandwidth=" << bandwidth_ << " is not among valid values\n";
            std::cout << " This parameter has been set to its default value bandwidth=2000000\n";
            bandwidth_ = 2000000;
            LOG(WARNING) << "Invalid configuration value for bandwidth parameter. Set to bandwidth=2000000";
        }

    std::cout << "device address: " << uri_ << '\n';
    std::cout << "frequency : " << freq_ << " Hz\n";
    std::cout << "sample rate: " << sample_rate_ << " Sps\n";
    std::cout << "gain mode: " << gain_mode_ << '\n';
    std::cout << "item type: " << item_type_ << '\n';

#if GNURADIO_API_IIO
#if GR_IIO_TEMPLATIZED_API
    plutosdr_source_ = gr::iio::fmcomms2_source<gr_complex>::make(uri_, {true}, buffer_size_);
    plutosdr_source_->set_gain_mode(0, gain_mode_);
    plutosdr_source_->set_gain(0, rf_gain_);
#else
    plutosdr_source_ = gr::iio::pluto_source::make(uri_, buffer_size_);
    plutosdr_source_->set_gain_mode(gain_mode_);
    plutosdr_source_->set_gain(rf_gain_);
#endif
    plutosdr_source_->set_frequency(freq_);
    plutosdr_source_->set_samplerate(sample_rate_);
    plutosdr_source_->set_quadrature(quadrature_);
    plutosdr_source_->set_rfdc(rf_dc_);
    plutosdr_source_->set_bbdc(bb_dc_);
    plutosdr_source_->set_filter_params(filter_source_, filter_filename_, Fpass_, Fstop_);
#else
    plutosdr_source_ = gr::iio::pluto_source::make(uri_, freq_, sample_rate_,
        bandwidth_, buffer_size_, quadrature_, rf_dc_, bb_dc_,
        gain_mode_.c_str(), rf_gain_, filter_file_.c_str(), filter_auto_);
#endif
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


void PlutosdrSignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->connect(plutosdr_source_, 0, valve_, 0);
            DLOG(INFO) << "connected plutosdr source to valve";
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
                    top_block->connect(plutosdr_source_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected plutosdr source to file sink";
                }
        }
}


void PlutosdrSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->disconnect(plutosdr_source_, 0, valve_, 0);
            if (dump_)
                {
                    top_block->disconnect(valve_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (dump_)
                {
                    top_block->disconnect(plutosdr_source_, 0, file_sink_, 0);
                }
        }
}


gr::basic_block_sptr PlutosdrSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return {};
}


gr::basic_block_sptr PlutosdrSignalSource::get_right_block()
{
    if (samples_ != 0)
        {
            return valve_;
        }
    else
        {
            return plutosdr_source_;
        }
}
