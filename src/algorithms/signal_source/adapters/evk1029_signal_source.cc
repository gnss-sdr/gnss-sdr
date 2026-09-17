/*!
 * \file evk1029_signal_source.cc
 * \brief SAPHYRION EVK1029 signal source
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

#include "evk1029_signal_source.h"
#include "configuration_interface.h"
#include <cmath>
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

Evk1029SignalSource::Evk1029SignalSource(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_stream,
    unsigned int out_stream,
    Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "EVK1029_Signal_Source"s),
      dump_filename_(configuration->property(role + ".dump_filename"s, "./evk1029_signal_source.dat"s)),
      dump_(configuration->property(role + ".dump"s, false)),
      item_size_(sizeof(int8_t)),
      rf_channels_(static_cast<unsigned int>(getRfChannels())),
      enable_throttle_control_(configuration->property(role + ".enable_throttle_control"s, false))
{
    const std::string filename = configuration->property(role + ".filename"s, "data.bin"s);
    // NOTE: this must be the RAW (pre-decimation) ADC sample rate of the capture
    // file (e.g. the .sdrx metadata's <freqbase>), NOT GNSS-SDR.internal_fs_sps
    // (which is the decimated working rate downstream of InputFilter). It paces
    // the throttle (when enabled), sizes the block's scheduling quantum, and
    // converts .seconds_to_skip into a position in the file.
    const int64_t default_fs = configuration->property("GNSS-SDR.internal_fs_sps"s, int64_t(0));
    const int64_t fs = configuration->property(role + ".sampling_frequency"s, default_fs);

    const double seconds_to_skip = configuration->property(role + ".seconds_to_skip"s, 0.0);
    uint64_t bytes_to_skip = 0;
    if (seconds_to_skip > 0.0 && fs > 0)
        {
            bytes_to_skip = static_cast<uint64_t>(std::llround(seconds_to_skip * static_cast<double>(fs) / 2.0));
            if (!configuration->is_present(role + ".sampling_frequency"s))
                {
                    LOG(WARNING) << role << ".seconds_to_skip is set but " << role << ".sampling_frequency is not: "
                                 << "converting seconds into a file position with GNSS-SDR.internal_fs_sps=" << fs
                                 << " Sps, which is wrong if the Input Filter decimates. Set " << role
                                 << ".sampling_frequency to the raw ADC rate of the capture.";
                }
        }

    if (rf_channels_ == 0)
        {
            rf_channels_ = 1;
        }

    DLOG(INFO) << "EVK1029 Signal Source: filename=" << filename << ", fs=" << fs << ", item_size=" << item_size_
               << ", RF_channels=" << rf_channels_ << ", seconds_to_skip=" << seconds_to_skip;

    evk1029_source_ = evk1029_make_source(filename, queue, static_cast<double>(fs), bytes_to_skip);

    if (enable_throttle_control_)
        {
            throttle_ = gr::blocks::throttle::make(item_size_, static_cast<double>(fs));
        }
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
        }

    if (in_stream > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_stream > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void Evk1029SignalSource::connect(gr::top_block_sptr top_block)
{
    if (enable_throttle_control_)
        {
            top_block->connect(evk1029_source_, 0, throttle_, 0);
            DLOG(INFO) << "connected evk1029_source to throttle";
            if (dump_)
                {
                    top_block->connect(throttle_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected throttle to file sink";
                }
        }
    else if (dump_)
        {
            top_block->connect(evk1029_source_, 0, file_sink_, 0);
            DLOG(INFO) << "connected evk1029_source to file sink";
        }
}


void Evk1029SignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (enable_throttle_control_)
        {
            top_block->disconnect(evk1029_source_, 0, throttle_, 0);
            if (dump_)
                {
                    top_block->disconnect(throttle_, 0, file_sink_, 0);
                }
        }
    else if (dump_)
        {
            top_block->disconnect(evk1029_source_, 0, file_sink_, 0);
        }
}


gr::basic_block_sptr Evk1029SignalSource::get_right_block()
{
    if (enable_throttle_control_)
        {
            return throttle_;
        }
    return evk1029_source_;
}
