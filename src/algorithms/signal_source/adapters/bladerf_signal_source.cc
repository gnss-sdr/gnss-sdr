/*!
 * \file bladerf_signal_source.cc
 * \brief Signal source for the Nuand bladeRF family of front-ends (bladeRF
 * x40, x115, and bladeRF 2.0 Micro xA4/xA9), using libbladeRF directly.
 * \author Oleksandr Suvorov, 2026.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "bladerf_signal_source.h"
#include "configuration_interface.h"
#include "gnss_frequencies.h"
#include "gnss_sdr_string_literals.h"
#include "gnss_sdr_valve.h"
#include <gnuradio/blocks/file_sink.h>
#include <stdexcept>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

BladerfSignalSource::BladerfSignalSource(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_stream,
    unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "Bladerf_Signal_Source"s),
      item_type_(configuration->property(role + ".item_type", std::string("gr_complex"))),
      dump_filename_(configuration->property(role + ".dump_filename", std::string("./data/signal_source.dat"))),
      bladerf_args_(configuration->property(role + ".bladerf_args", std::string())),
      device_serial_(configuration->property(role + ".device_serial", std::string())),
      sample_rate_(configuration->property(role + ".sampling_frequency", 2.0e6)),
      freq_(configuration->property(role + ".freq", FREQ1)),
      bandwidth_(configuration->property(role + ".bandwidth", sample_rate_)),
      gain_(configuration->property(role + ".gain", 40.0)),
      samples_(configuration->property(role + ".samples", int64_t(0))),
      in_stream_(in_stream),
      out_stream_(out_stream),
      num_buffers_(configuration->property(role + ".num_buffers", 32u)),
      buffer_size_(configuration->property(role + ".buffer_size", 32768u)),
      num_transfers_(configuration->property(role + ".num_transfers", 16u)),
      stream_timeout_ms_(configuration->property(role + ".stream_timeout_ms", 3000u)),
      agc_enabled_(configuration->property(role + ".AGC_enabled", false)),
      rx_bias_tee_(configuration->property(role + ".rx_bias_tee", false)),
      dump_(configuration->property(role + ".dump", false))
{
    if (item_type_ != "gr_complex")
        {
            std::string error_message = "bladeRF: item_type '" + item_type_ + "' not supported, only 'gr_complex' is supported.";
            LOG(ERROR) << error_message;
            throw std::invalid_argument(error_message);
        }
    item_size_ = sizeof(gr_complex);

    try
        {
            bladerf_source_ = bladerf_make_source_sptr(bladerf_args_, device_serial_,
                sample_rate_, freq_, bandwidth_, gain_, agc_enabled_, rx_bias_tee_,
                num_buffers_, buffer_size_, num_transfers_, stream_timeout_ms_);
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << "bladeRF exception: " << e.what();
            throw std::invalid_argument("Wrong bladeRF arguments");
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


void BladerfSignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->connect(bladerf_source_, 0, valve_, 0);
            DLOG(INFO) << "connected bladeRF source to valve";
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
                    top_block->connect(bladerf_source_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected bladeRF source to file sink";
                }
        }
}


void BladerfSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->disconnect(bladerf_source_, 0, valve_, 0);
            if (dump_)
                {
                    top_block->disconnect(valve_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (dump_)
                {
                    top_block->disconnect(bladerf_source_, 0, file_sink_, 0);
                }
        }
}


gr::basic_block_sptr BladerfSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return {};
}


gr::basic_block_sptr BladerfSignalSource::get_right_block()
{
    if (samples_ != 0)
        {
            return valve_;
        }
    return bladerf_source_;
}
