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

#include <algorithm>
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

    void reset()
    {
        std::fill(hist_.begin(), hist_.end(), 0);
        total_events_ = 0;
        epoch_count_ = 0;
        locked_ = false;
        edge_phase_ = -1;

        has_last_prompt_ = false;
        last_prompt_ = std::complex<float>(0.0f, 0.0f);

        has_last_sign_ = false;
        last_sign_ = +1;

        has_last_best_bin_ = false;
        last_best_bin_ = 0;
        stable_best_count_ = 0;
    }

    // Call once per epoch (e.g., per coherent integration output).
    // Returns true ONLY on the epoch when lock is first declared.
    bool update(const std::complex<float>& prompt, bool tracking_quality_ok)
    {
        const int N = bins();
        const int phase = (N > 0) ? static_cast<int>(epoch_count_ % N) : 0;

        // Always advance epoch counter; even if gated out we keep phase consistent.
        ++epoch_count_;

        // Gate on tracking status and magnitude
        if (!tracking_quality_ok || (std::abs(prompt) < cfg_.min_prompt_mag))
            {
                last_prompt_ = prompt;
                has_last_prompt_ = true;
                return false;
            }

        bool edge_event = false;

        if (cfg_.use_phase_dot_detector)
            {
                if (has_last_prompt_)
                    {
                        // dot = Re( Pk * conj(Pk-1) ); negative suggests polarity inversion
                        const double dot = static_cast<double>(std::real(prompt * std::conj(last_prompt_)));
                        edge_event = (dot < 0.0);
                    }
                // update last prompt after using it
                last_prompt_ = prompt;
                has_last_prompt_ = true;
            }
        else
            {
                const int s = (std::real(prompt) >= 0.0f) ? +1 : -1;
                if (has_last_sign_)
                    {
                        edge_event = (s != last_sign_);
                    }
                last_sign_ = s;
                has_last_sign_ = true;
            }

        if (edge_event && N > 0)
            {
                ++hist_[phase];
                ++total_events_;
            }

        // Evaluate lock condition
        if (!locked_ && (total_events_ >= cfg_.min_events_for_lock))
            {
                int best_bin = 0;
                int best_count = 0;
                best_bin_and_count(best_bin, best_count);

                const double ratio = (total_events_ > 0)
                                         ? (static_cast<double>(best_count) / static_cast<double>(total_events_))
                                         : 0.0;

                if (!has_last_best_bin_ || (best_bin != last_best_bin_))
                    {
                        last_best_bin_ = best_bin;
                        has_last_best_bin_ = true;
                        stable_best_count_ = 1;
                    }
                else
                    {
                        ++stable_best_count_;
                    }

                if ((ratio >= cfg_.dominance_ratio) &&
                    (stable_best_count_ >= cfg_.stable_best_required))
                    {
                        locked_ = true;
                        edge_phase_ = best_bin;
                        return true;  // lock event
                    }
            }

        return false;
    }

    bool locked() const { return locked_; }

    // Returns -1 if not locked.
    int edge_phase() const { return edge_phase_; }

    // For a given epoch index k (0-based), tells you if it's the predicted edge epoch.
    bool is_edge_epoch(std::int64_t k) const
    {
        if (!locked_ || edge_phase_ < 0) return false;
        const int N = bins();
        if (N <= 0) return false;
        return (static_cast<int>(k % N) == edge_phase_);
    }

    int bins() const
    {
        const int N = (cfg_.epoch_ms > 0) ? (cfg_.bit_period_ms / cfg_.epoch_ms) : 0;
        return (N > 0) ? N : 0;
    }

    const std::vector<int>& histogram() const { return hist_; }
    std::int64_t total_events() const { return total_events_; }
    std::int64_t epoch_count() const { return epoch_count_; }

private:
    void best_bin_and_count(int& best_bin, int& best_count) const
    {
        best_bin = 0;
        best_count = (hist_.empty() ? 0 : hist_[0]);
        for (int i = 1; i < static_cast<int>(hist_.size()); ++i)
            {
                if (hist_[i] > best_count)
                    {
                        best_count = hist_[i];
                        best_bin = i;
                    }
            }
    }

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