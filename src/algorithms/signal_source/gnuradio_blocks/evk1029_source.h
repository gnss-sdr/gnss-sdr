/*!
 * \file evk1029_source.h
 * \brief Unpacks the SAPHYRION EVK1029 raw capture files: a continuous,
 * header-less stream of OBA-encoded 4-bit samples, 16 samples packed per
 * little-endian 64-bit word.
 *
 * With an empty freq_xlating_bands (the default), this block emits raw
 * decoded samples on a single output port, as before. A non-empty
 * freq_xlating_bands instead gives it one gr_complex output port per band,
 * each already frequency-translated and decimated to baseband inside this
 * block's own work() -- avoiding a separate filter block and thread per
 * band, at the cost of every band sharing one decimation factor.
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

#ifndef GNSS_SDR_EVK1029_SOURCE_H
#define GNSS_SDR_EVK1029_SOURCE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include <gnuradio/sync_decimator.h>
#include <pmt/pmt.h>
#include <volk/volk_alloc.hh>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#if CUDA_GPU_ACCEL
#include "cuda_ddc_engine.h"
#endif

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_gnuradio_blocks
 * \{ */


class Evk1029Source;

using Evk1029Source_sptr = gnss_shared_ptr<Evk1029Source>;

//! Per-band configuration for the optional internal frequency-translation mode.
struct Evk1029FreqXlatingBand
{
    int decimation_factor;
    double center_freq;
    std::vector<float> taps;  // real low-pass prototype, e.g. from firdes::low_pass
    bool use_cuda = false;    // only takes effect when built with -DENABLE_CUDA=ON; see evk1029_source.cc
};

// bytes_to_skip is rounded up to the next 64-bit word, matching the
// capture's native packing, so a jumped-to run stays comparable to a
// from-the-start one at the word level. freq_xlating_bands: see above.
Evk1029Source_sptr evk1029_make_source(const std::string& filename, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency, uint64_t bytes_to_skip = 0,
    const std::vector<Evk1029FreqXlatingBand>& freq_xlating_bands = {});

/*!
 * \brief Reads a continuous, header-less EVK1029 raw capture file and
 * unpacks its OBA-encoded 4-bit samples (two per byte, low nibble first).
 */
class Evk1029Source : public gr::sync_decimator
{
public:
    ~Evk1029Source() override;

private:
    friend Evk1029Source_sptr evk1029_make_source(const std::string& filename, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency, uint64_t bytes_to_skip,
        const std::vector<Evk1029FreqXlatingBand>& freq_xlating_bands);
    Evk1029Source(const std::string& filename, Concurrent_Queue<pmt::pmt_t>* queue, double sampling_frequency, uint64_t bytes_to_skip,
        const std::vector<Evk1029FreqXlatingBand>& freq_xlating_bands);

    int work(int noutput_items,
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items) override;

    // Decodes up to decoded_needed samples into target[start_offset...];
    // returns the total valid count (>= start_offset, < decoded_needed at EOF).
    std::size_t decode_into_buffer(std::size_t decoded_needed, std::vector<int8_t>& target, std::size_t start_offset = 0);

    // Ping-pong decode-ahead: a persistent thread decodes+converts the NEXT
    // chunk into whichever of decoded_/decoded_b_ isn't the active one,
    // concurrently with this call's FIR work -- so by the time the
    // following work() call runs, decode+convert for it is usually already
    // done, taking that serial cost off the critical path. See work()'s
    // doc comment for the full protocol (bootstrap, steady state, EOF).
    struct DecodeWorker
    {
        std::thread thread;
        std::mutex mutex;
        std::condition_variable cv;
        bool has_work = false;
        bool finished = true;
        bool stop = false;
        std::size_t decoded_needed = 0;
        std::size_t start_offset = 0;  // target already has this many valid samples (carried-forward leftover + history)
        std::vector<int8_t>* target_decoded = nullptr;
        volk::vector<float>* target_converted = nullptr;
        std::size_t produced = 0;
    };
    void decode_worker_loop();
    std::unique_ptr<DecodeWorker> decode_worker_;
    std::vector<int8_t> decoded_b_;    // decoded_'s ping-pong partner
    volk::vector<float> converted_b_;  // converted_'s ping-pong partner
    bool first_call_ = true;
    bool decode_worker_has_pending_result_ = false;  // false only once EOF means no further prefetch was started

    // Computes band bands_[b]'s output (FIR dot product + rotate) for
    // usable_noutput_items samples into out. Safe to run concurrently for
    // different b: only touches output_items[b] and bands_[b]'s own state.
    void process_band(std::size_t b, int usable_noutput_items, gr_complex* out);

    // One persistent worker per band beyond the first (that one runs on
    // work()'s own calling thread instead), parked on a condition variable
    // between calls rather than spawned/joined per call.
    struct BandWorker
    {
        std::thread thread;
        std::mutex mutex;
        std::condition_variable cv;
        bool has_work = false;  // set by work(), cleared by the worker once it starts
        bool finished = false;  // set by the worker when done, cleared by work()
        bool stop = false;      // set by the destructor to end the worker's loop
        int usable_noutput_items = 0;
        gr_complex* out = nullptr;
    };
    void worker_loop(std::size_t band_index, BandWorker* w);
    std::vector<std::unique_ptr<BandWorker>> workers_;

    std::ifstream binary_input_file_;
    Concurrent_Queue<pmt::pmt_t>* queue_;
    std::vector<uint8_t> buffer_;
    std::size_t buffer_valid_;  // number of valid bytes currently in buffer_
    std::size_t buffer_pos_;    // next unread byte offset within buffer_
    bool eof_;

    // Only used when freq_xlating_bands was non-empty at construction:
    struct Band
    {
        volk::vector<gr_complex> composite_fir;  // frequency-shifted, reversed to match fir_filter's convention
        std::size_t history_offset;              // history_len_ - (composite_fir.size() - 1), precomputed once
#if CUDA_GPU_ACCEL
        std::unique_ptr<CudaDdcEngine> cuda_engine;  // null => CPU path (construction or a prior compute() failed)
#endif
        gr_complex phase;
        gr_complex phase_incr;
        int samples_since_phase_renorm;
    };
    std::vector<Band> bands_;
    std::size_t history_len_;        // max(taps.size()) - 1 across bands_, 0 if bands_ is empty
    std::vector<int8_t> decoded_;    // history_len_ history samples, then newly decoded ones
    volk::vector<float> converted_;  // decoded_ converted to float (x256, matching char_to_short)
};


/** \} */
/** \} */
#endif  // GNSS_SDR_EVK1029_SOURCE_H
