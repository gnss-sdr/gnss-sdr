/*!
 * \file bit_synchronizer.h
 * \brief Histogram-based bit-edge synchronizer for GNSS prompt correlator outputs.
 * \author Carles Fernandez-Prades, 2026 cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_BIT_SYNCHRONIZER_H
#define GNSS_SDR_BIT_SYNCHRONIZER_H

#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

class HistogramBitSynchronizer
{
public:
    struct Config
    {
        int bit_period_ms;            // e.g., 20 for GPS L1 C/A
        int epoch_ms;                 // e.g., 1
        int min_events_for_lock;      // e.g., 30
        double dominance_ratio;       // e.g., 0.55
        int stable_best_required;     // e.g., 5
        float min_prompt_mag;         // gate: ignore |P| below this
        bool use_phase_dot_detector;  // true: dot(Pk,Pk-1) sign; false: sign(Re(P))

        Config()
            : bit_period_ms(20),
              epoch_ms(1),
              min_events_for_lock(30),
              dominance_ratio(0.55),
              stable_best_required(5),
              min_prompt_mag(0.0f),
              use_phase_dot_detector(true)
        {
        }
    };

    explicit HistogramBitSynchronizer(const Config& cfg)
        : cfg_(cfg),
          total_events_(0),
          epoch_count_(0),
          locked_(false),
          edge_phase_(-1),
          has_last_prompt_(false),
          last_prompt_(0.0f, 0.0f),
          has_last_sign_(false),
          last_sign_(+1),
          has_last_best_bin_(false),
          last_best_bin_(0),
          stable_best_count_(0)
    {
        hist_.assign(bins(), 0);
    }

    void reset();

    // Call once per epoch (e.g., per coherent integration output).
    // Returns true ONLY on the epoch when lock is first declared.
    bool update(const std::complex<float>& prompt, bool tracking_quality_ok);

    bool locked() const { return locked_; }

    // Returns -1 if not locked.
    int edge_phase() const { return edge_phase_; }

    // For a given epoch index k (0-based), tells you if it's the predicted edge epoch.
    bool is_edge_epoch(std::int64_t k) const;

    int bins() const;

    const std::vector<int>& histogram() const { return hist_; }
    std::int64_t total_events() const { return total_events_; }
    std::int64_t epoch_count() const { return epoch_count_; }

private:
    void best_bin_and_count(int& best_bin, int& best_count) const;

    Config cfg_;
    std::vector<int> hist_;
    std::int64_t total_events_;
    std::int64_t epoch_count_;

    bool locked_;
    int edge_phase_;

    // Prompt history (for dot detector)
    bool has_last_prompt_;
    std::complex<float> last_prompt_;

    // Sign history (for simple detector)
    bool has_last_sign_;
    int last_sign_;

    // Stability tracking for best bin
    bool has_last_best_bin_;
    int last_best_bin_;
    int stable_best_count_;
};

#endif // GNSS_SDR_BIT_SYNCHRONIZER_H