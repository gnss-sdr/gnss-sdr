/*!
 * \file evk1029_source.cc
 * \brief Unpacks the SAPHYRION EVK1029 raw capture files (continuous stream
 * of OBA-encoded 4-bit samples, no periodic block header).
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

#include "evk1029_source.h"
#include "command_event.h"
#include <gnuradio/io_signature.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
// One file read at a time; large enough to amortize I/O overhead without
// requiring a huge allocation.
constexpr std::size_t kReadChunkBytes = 1 << 20;  // 1 MiB

// Empirically tuned (see set_output_multiple() call below): sweeping the
// output-multiple quantum at ~178 Msps found 131072 samples (~0.74 ms)
// measurably reduced CPU on both this block and the downstream
// freq_xlating_fir_filter versus 8192, with 524288 showing no further
// improvement (plateaued). Deriving the quantum from the actual sampling
// frequency instead of hardcoding 131072 keeps that same ~0.7 ms target
// (and so the same CPU behavior) if this is ever run at a rate other than
// the one it was tuned against.
constexpr double kOutputMultipleTargetSeconds = 0.0007;
constexpr int kOutputMultipleFallback = 131072;  // used if sampling_frequency is unavailable/invalid

int output_multiple_from_sampling_frequency(double sampling_frequency)
{
    if (!(sampling_frequency > 0.0))
        {
            return kOutputMultipleFallback;
        }
    const double target_samples = sampling_frequency * kOutputMultipleTargetSeconds;
    // Round to the *nearest* power of two, not just the next one up: compare
    // the target against both the power-of-two bracket it falls between.
    const int lower_exp = static_cast<int>(std::floor(std::log2(std::max(target_samples, 1.0))));
    const double lower = std::exp2(static_cast<double>(lower_exp));
    const double upper = std::exp2(static_cast<double>(lower_exp + 1));
    return static_cast<int>((target_samples - lower <= upper - target_samples) ? lower : upper);
}
}  // namespace


Evk1029Source_sptr evk1029_make_source(const std::string& filename, int n_streams, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency)
{
    return Evk1029Source_sptr(new Evk1029Source(filename, n_streams, queue, sampling_frequency));
}


Evk1029Source::Evk1029Source(const std::string& filename, int n_streams, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency)
    : gr::sync_block("evk1029_source",
          gr::io_signature::make(0, 0, 0),
          gr::io_signature::make(n_streams, n_streams, sizeof(int8_t))),
      queue_(queue),
      buffer_(kReadChunkBytes),
      buffer_valid_(0),
      buffer_pos_(0),
      n_streams_(n_streams)
{
    // Keep the scheduler from calling work() with a tiny noutput_items; each
    // unit of work produces a pair of samples (one input byte -> 2 samples).
    // This block runs at the raw, pre-decimation sample rate (up to ~180 Msps),
    // so call-frequency overhead here also drags down every downstream reader
    // (e.g. freq_xlating_fir_filter) that ends up woken at the same cadence.
    // See output_multiple_from_sampling_frequency()'s doc comment for how
    // this quantum is chosen.
    set_output_multiple(output_multiple_from_sampling_frequency(sampling_frequency));

    binary_input_file_.open(filename.c_str(), std::ios::in | std::ios::binary);
    if (binary_input_file_.is_open())
        {
            std::cout << "EVK1029_Source: file '" << filename << "' successfully opened.\n";
        }
    else
        {
            std::cerr << "EVK1029_Source: failed to open file: " << filename << '\n';
            exit(1);
        }
}


Evk1029Source::~Evk1029Source()
{
    try
        {
            if (binary_input_file_.is_open())
                {
                    binary_input_file_.close();
                }
        }
    catch (const std::ifstream::failure& e)
        {
            std::cerr << "EVK1029_Source: problem closing input file.\n";
        }
    catch (const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
}


int Evk1029Source::work(int noutput_items,
    gr_vector_const_void_star& input_items __attribute__((unused)),
    gr_vector_void_star& output_items)
{
    auto* out = reinterpret_cast<int8_t*>(output_items[0]);
    int produced = 0;
    bool eof = false;

    // Each input byte unpacks to two OBA 4-bit samples (low nibble first),
    // so only ever produce an even number of items. Note noutput_items can
    // legitimately be small (even 0 or 1) on some scheduler calls, which is
    // NOT end of file -- only an actual zero-byte file read means EOF.
    while (produced + 1 < noutput_items)
        {
            if (buffer_pos_ >= buffer_valid_)
                {
                    binary_input_file_.read(reinterpret_cast<char*>(buffer_.data()), static_cast<std::streamsize>(buffer_.size()));
                    buffer_valid_ = static_cast<std::size_t>(binary_input_file_.gcount());
                    buffer_pos_ = 0;
                    if (buffer_valid_ == 0)
                        {
                            eof = true;
                            break;
                        }
                }

            const uint8_t byte = buffer_[buffer_pos_++];
            const int low_nibble = byte & 0x0F;
            const int high_nibble = (byte >> 4) & 0x0F;
            // OBA 4-bit decode: raw in [0,15] -> {-15,-13,...,-1,1,...,15}
            out[produced++] = static_cast<int8_t>(low_nibble * 2 - 15);
            out[produced++] = static_cast<int8_t>(high_nibble * 2 - 15);
        }

    if (eof && produced == 0)
        {
            std::cout << "EVK1029_Source: EOF\n";
            queue_->push(pmt::make_any(command_event_make(200, 0)));
            return this->WORK_DONE;
        }

    // All output ports carry an identical copy of the same unpacked samples
    // (single read, single unpack, fanned out here) so that RF bands fed
    // from different ports of this one block can never drift apart.
    for (int p = 1; p < n_streams_; ++p)
        {
            std::memcpy(output_items[p], out, static_cast<std::size_t>(produced) * sizeof(int8_t));
        }

    return produced;
}
