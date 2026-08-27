/*!
 * \file bit_synchronizer.cc
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

#include "bit_synchronizer.h"
#include <algorithm>

void HistogramBitSynchronizer::reset()
{
    std::fill(hist_.begin(), hist_.end(), 0);
    total_events_ = 0;
    epoch_count_ = 0;
    locked_ = false;
    edge_phase_ = -1;

    prompt_window_.clear();

    has_last_sign_ = false;
    last_sign_ = +1;

    has_last_best_bin_ = false;
    last_best_bin_ = 0;
    stable_best_count_ = 0;

    tentative_lock_ = false;
    tentative_phase_ = -1;
    tentative_event_count_ = 0;
}


bool HistogramBitSynchronizer::update(const std::complex<float>& prompt, bool tracking_quality_ok)
{
    const int N = bins();
    int phase = (N > 0) ? static_cast<int>(epoch_count_ % N) : 0;

    // Always advance epoch counter; even if gated out we keep phase consistent.
    ++epoch_count_;

    // Gate on tracking status and magnitude
    if (!tracking_quality_ok || (std::abs(prompt) < cfg_.min_prompt_mag))
        {
            prompt_window_.clear();
            has_last_sign_ = false;
            return false;
        }

    bool edge_event = false;

    if (cfg_.use_phase_dot_detector)
        {
            const int window_epochs = transition_window_epochs();
            prompt_window_.push_back(prompt);
            while (static_cast<int>(prompt_window_.size()) > 2 * window_epochs)
                {
                    prompt_window_.pop_front();
                }

            if (static_cast<int>(prompt_window_.size()) == 2 * window_epochs)
                {
                    std::complex<double> before(0.0, 0.0);
                    std::complex<double> after(0.0, 0.0);
                    double before_magnitude_sum = 0.0;
                    double after_magnitude_sum = 0.0;
                    for (int i = 0; i < window_epochs; ++i)
                        {
                            before += static_cast<std::complex<double>>(prompt_window_[i]);
                            after += static_cast<std::complex<double>>(prompt_window_[i + window_epochs]);
                            before_magnitude_sum += std::abs(prompt_window_[i]);
                            after_magnitude_sum += std::abs(prompt_window_[i + window_epochs]);
                        }

                    const double average_product = std::abs(before) * std::abs(after);
                    if (average_product > 0.0 && before_magnitude_sum > 0.0 && after_magnitude_sum > 0.0)
                        {
                            const double normalized_inversion =
                                -std::real(after * std::conj(before)) / average_product;
                            const double before_coherence = std::abs(before) / before_magnitude_sum;
                            const double after_coherence = std::abs(after) / after_magnitude_sum;
                            const double confidence = normalized_inversion * std::min(before_coherence, after_coherence);
                            edge_event = (confidence >= cfg_.transition_confidence);
                        }

                    const std::int64_t candidate_epoch = epoch_count_ - window_epochs;
                    phase = (N > 0) ? static_cast<int>(candidate_epoch % N) : 0;
                }
        }
    else
        {
            const int s = (std::real(prompt) >= 0.0F) ? +1 : -1;
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
    if (!locked_ && edge_event && (total_events_ >= cfg_.min_events_for_lock))
        {
            int best_bin = 0;
            int best_count = 0;
            int second_best_count = 0;
            best_bins_and_counts(best_bin, best_count, second_best_count);

            if (!histogram_confident(best_count, second_best_count))
                {
                    has_last_best_bin_ = false;
                    stable_best_count_ = 0;
                    tentative_lock_ = false;
                    tentative_phase_ = -1;
                    tentative_event_count_ = 0;
                }
            else
                {
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

                    if (cfg_.tentative_events_required > 0)
                        {
                            if (!tentative_lock_ || tentative_phase_ != best_bin)
                                {
                                    tentative_lock_ = true;
                                    tentative_phase_ = best_bin;
                                    tentative_event_count_ = 0;
                                }
                            else if (phase == tentative_phase_)
                                {
                                    ++tentative_event_count_;
                                }
                            else
                                {
                                    tentative_event_count_ = 0;
                                    stable_best_count_ = 1;
                                }
                        }

                    const bool tentative_validated =
                        cfg_.tentative_events_required <= 0 ||
                        tentative_event_count_ >= cfg_.tentative_events_required;
                    if (stable_best_count_ >= cfg_.stable_best_required && tentative_validated)
                        {
                            locked_ = true;
                            edge_phase_ = best_bin;
                            return true;
                        }
                }
        }

    return false;
}


bool HistogramBitSynchronizer::is_edge_epoch(std::int64_t k) const
{
    if (!locked_ || edge_phase_ < 0)
        {
            return false;
        }
    const int N = bins();
    if (N <= 0)
        {
            return false;
        }
    return (static_cast<int>(k % N) == edge_phase_);
}


int HistogramBitSynchronizer::bins() const
{
    const int N = (cfg_.epoch_ms > 0) ? (cfg_.bit_period_ms / cfg_.epoch_ms) : 0;
    return (N > 0) ? N : 0;
}


void HistogramBitSynchronizer::best_bins_and_counts(int& best_bin, int& best_count, int& second_best_count) const
{
    best_bin = 0;
    best_count = (hist_.empty() ? 0 : hist_[0]);
    second_best_count = 0;
    for (int i = 1; i < static_cast<int>(hist_.size()); ++i)
        {
            if (hist_[i] > best_count)
                {
                    second_best_count = best_count;
                    best_count = hist_[i];
                    best_bin = i;
                }
            else if (hist_[i] > second_best_count)
                {
                    second_best_count = hist_[i];
                }
        }
}


bool HistogramBitSynchronizer::histogram_confident(int best_count, int second_best_count) const
{
    if (total_events_ <= 0)
        {
            return false;
        }
    const auto total = static_cast<double>(total_events_);
    const double dominance = static_cast<double>(best_count) / total;
    const double runner_up_margin = static_cast<double>(best_count - second_best_count) / total;
    return dominance >= cfg_.dominance_ratio && runner_up_margin >= cfg_.runner_up_margin;
}


int HistogramBitSynchronizer::transition_window_epochs() const
{
    const int maximum_window = std::max(1, bins() / 2);
    return std::max(1, std::min(cfg_.transition_window_epochs, maximum_window));
}


int HistogramBitSynchronizer::epochs_until_next_edge() const
{
    if (!locked_ || edge_phase_ < 0)
        {
            return -1;
        }

    const int B = bins();
    if (B <= 0)
        {
            return -1;
        }

    // Current epoch index (last processed)
    const std::int64_t k_now = epoch_count_ - 1;

    // Phase of the current epoch
    const int cur_phase = static_cast<int>(k_now % B);

    // Forward distance (modulo B) to the next epoch whose phase equals edge_phase_
    // This returns:
    // 0 -> the current epoch IS the first epoch of a new bit
    // 1+ -> number of epochs to wait until the next bit boundary
    return (edge_phase_ - cur_phase + B) % B;
}
