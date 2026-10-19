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
#include <gnuradio/filter/firdes.h>
#include <cmath>
#include <stdexcept>
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

using namespace std::string_literals;

namespace
{
// firdes takes gr::fft::window::win_type since GNU Radio 3.9, and its own
// firdes::win_type before that.
#if GNURADIO_USES_STD_POINTERS
using Fir_Window = gr::fft::window::win_type;
#else
using Fir_Window = gr::filter::firdes::win_type;
#endif

Fir_Window parse_window_type(const std::string& role, const std::string& name)
{
    if (name == "hamming")
        {
            return Fir_Window::WIN_HAMMING;
        }
    if (name == "hann")
        {
            return Fir_Window::WIN_HANN;
        }
    if (name == "blackman")
        {
            return Fir_Window::WIN_BLACKMAN;
        }
    if (name == "blackman_harris")
        {
            return Fir_Window::WIN_BLACKMAN_HARRIS;
        }
    if (name == "rectangular")
        {
            return Fir_Window::WIN_RECTANGULAR;
        }
    if (name == "kaiser")
        {
            return Fir_Window::WIN_KAISER;
        }
    LOG(WARNING) << role << ": unknown window type '" << name << "', falling back to hamming";
    return Fir_Window::WIN_HAMMING;
}

// Kaiser's own empirical formula (1974) relating a target stopband
// attenuation to the window beta that actually achieves it. GNU Radio's
// firdes::low_pass_2() does NOT derive beta from attenuation_dB itself --
// attenuation_dB there only sizes the tap count, while beta independently
// sets the real window shape -- so this block computes a consistent beta
// unless the user overrides it explicitly.
double kaiser_beta_from_attenuation(double attenuation_dB)
{
    if (attenuation_dB > 50.0)
        {
            return 0.1102 * (attenuation_dB - 8.7);
        }
    if (attenuation_dB >= 21.0)
        {
            return 0.5842 * std::pow(attenuation_dB - 21.0, 0.4) + 0.07886 * (attenuation_dB - 21.0);
        }
    return 0.0;
}

// "./evk1029_signal_source.dat" -> "./evk1029_signal_source_ch1.dat"
std::string band_dump_filename(const std::string& filename, unsigned int band)
{
    const std::string suffix = "_ch" + std::to_string(band);
    const auto dot = filename.find_last_of('.');
    const auto slash = filename.find_last_of("/\\");
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash) || dot == slash + 1)
        {
            return filename + suffix;
        }
    return filename.substr(0, dot) + suffix + filename.substr(dot);
}
}  // namespace

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
      enable_throttle_control_(configuration->property(role + ".enable_throttle_control"s, false)),
      enable_freq_xlating_(configuration->property(role + ".enable_freq_xlating"s, false))
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

    std::vector<Evk1029FreqXlatingBand> freq_xlating_bands;
    if (enable_freq_xlating_)
        {
            if (enable_throttle_control_)
                {
                    LOG(ERROR) << role << ": enable_throttle_control is not supported together with enable_freq_xlating; ignoring enable_throttle_control";
                    enable_throttle_control_ = false;
                }
            if (fs <= 0)
                {
                    throw std::invalid_argument(role + ".enable_freq_xlating requires a positive " + role + ".sampling_frequency");
                }
            item_size_ = sizeof(gr_complex);
            for (unsigned int i = 0; i < rf_channels_; i++)
                {
                    const std::string band = role + ".band"s + std::to_string(i) + "_"s;
                    const double intermediate_freq = configuration->property(band + "IF"s, 0.0);
                    const int decimation_factor = configuration->property(band + "decimation_factor"s, 1);
                    if (decimation_factor < 1)
                        {
                            throw std::invalid_argument(band + "decimation_factor must be at least 1");
                        }
                    if (!freq_xlating_bands.empty() && decimation_factor != freq_xlating_bands.front().decimation_factor)
                        {
                            throw std::invalid_argument(band + "decimation_factor (" + std::to_string(decimation_factor) + ") differs from " + role + ".band0_decimation_factor (" + std::to_string(freq_xlating_bands.front().decimation_factor) + "): all bands share one output rate");
                        }
                    const double default_bw = (static_cast<double>(fs) / decimation_factor) / 2;
                    const double bw = configuration->property(band + "bw"s, default_bw);
                    const double default_tw = bw / 10.0;
                    const double tw = configuration->property(band + "tw"s, default_tw);
                    const std::string window_name = configuration->property(band + "window"s, "hamming"s);
                    const Fir_Window window = parse_window_type(role, window_name);
                    Evk1029FreqXlatingBand cfg;
                    cfg.decimation_factor = decimation_factor;
                    cfg.center_freq = intermediate_freq;
                    cfg.use_cuda = configuration->property(band + "cuda"s, false);
                    if (configuration->is_present(band + "attenuation_dB"s))
                        {
                            const double attenuation_dB = configuration->property(band + "attenuation_dB"s, 53.0);
                            const double default_kaiser_beta = kaiser_beta_from_attenuation(attenuation_dB);
                            const double kaiser_beta = configuration->property(band + "kaiser_beta"s, default_kaiser_beta);
                            cfg.taps = gr::filter::firdes::low_pass_2(1.0, static_cast<double>(fs), bw, tw, attenuation_dB, window, kaiser_beta);
                        }
                    else
                        {
                            const double kaiser_beta = configuration->property(band + "kaiser_beta"s, 6.76);
                            cfg.taps = gr::filter::firdes::low_pass(1.0, static_cast<double>(fs), bw, tw, window, kaiser_beta);
                        }
                    LOG(INFO) << role << ": band " << i << " freq-xlating, IF=" << intermediate_freq
                              << " decimation=" << decimation_factor << " window=" << window_name << " taps=" << cfg.taps.size();
                    freq_xlating_bands.push_back(std::move(cfg));
                }
        }

    evk1029_source_ = evk1029_make_source(filename, queue, static_cast<double>(fs), bytes_to_skip, freq_xlating_bands);

    if (enable_throttle_control_)
        {
            throttle_ = gr::blocks::throttle::make(item_size_, static_cast<double>(fs));
        }
    if (dump_)
        {
            // One file per output port: the raw stream, or each translated band.
            const unsigned int ports = enable_freq_xlating_ ? rf_channels_ : 1;
            for (unsigned int i = 0; i < ports; i++)
                {
                    const std::string filename = ports > 1 ? band_dump_filename(dump_filename_, i) : dump_filename_;
                    DLOG(INFO) << "Dumping output into file " << filename;
                    file_sinks_.push_back(gr::blocks::file_sink::make(item_size_, filename.c_str()));
                }
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
                    top_block->connect(throttle_, 0, file_sinks_.front(), 0);
                    DLOG(INFO) << "connected throttle to file sink";
                }
        }
    else
        {
            for (size_t i = 0; i < file_sinks_.size(); i++)
                {
                    top_block->connect(evk1029_source_, static_cast<int>(i), file_sinks_[i], 0);
                    DLOG(INFO) << "connected evk1029_source output " << i << " to file sink";
                }
        }
}


void Evk1029SignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (enable_throttle_control_)
        {
            top_block->disconnect(evk1029_source_, 0, throttle_, 0);
            if (dump_)
                {
                    top_block->disconnect(throttle_, 0, file_sinks_.front(), 0);
                }
        }
    else
        {
            for (size_t i = 0; i < file_sinks_.size(); i++)
                {
                    top_block->disconnect(evk1029_source_, static_cast<int>(i), file_sinks_[i], 0);
                }
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
