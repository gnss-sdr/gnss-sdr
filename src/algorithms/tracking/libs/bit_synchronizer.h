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

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


/**
 * @brief Histogram-based navigation data bit synchronizer.
 */
class HistogramBitSynchronizer
{
public:
    /**
     * @brief Configuration parameters for HistogramBitSynchronizer.
     *
     * These parameters define the bit period, the update cadence, the lock criteria,
     * and the transition detection method.
     */
    struct Config
    {
        /**
         * @brief Navigation data bit period in milliseconds.
         *
         * This is the nominal duration of one navigation data bit.
         */
        int bit_period_ms;

        /**
         * @brief Time interval between successive calls to update(), in milliseconds.
         *
         * This should match the minimum integration interval (epoch) produced by the
         * tracking loop and used to generate the provided prompt correlator output.
         */
        int epoch_ms;

        /**
         * @brief Minimum number of detected transition events required before lock evaluation.
         *
         * The histogram is built from detected candidate transitions. Lock decisions are
         * not attempted until at least this many events have been accumulated.
         *
         * Trade-offs:
         *   - Larger values increase robustness against false locks but increase time-to-lock.
         *   - Smaller values reduce time-to-lock but increase sensitivity to noise/spurious transitions.
         */
        int min_events_for_lock;

        /**
         * @brief Required dominance ratio of the winning histogram bin.
         *
         * Lock requires the most frequent histogram bin to be sufficiently dominant:
         * @f[
         *   \text{dominance\_ratio} = \frac{\text{best\_bin\_count}}{\text{total\_detected\_events}}
         * @f]
         *
         * Guidance:
         *   - Values near 0.5 may lock faster but increase false-lock probability.
         *   - Values closer to 1.0 are conservative and require a clearly dominant phase.
         */
        double dominance_ratio;

        /**
         * @brief Required stability of the dominant histogram bin (consecutive evaluations).
         *
         * Even if the dominance ratio is met, the algorithm requires that the same histogram
         * bin remains dominant for this many consecutive lock evaluations before declaring lock.
         *
         * This helps prevent locking on transient peaks caused by noise or short-lived disturbances.
         */
        int stable_best_required;

        /**
         * @brief Minimum magnitude of the prompt correlator output.
         *
         * Candidate transition detection is suppressed when @f$|P| < \text{min\_prompt\_mag}@f$,
         * where @f$P@f$ is the prompt correlator output.
         *
         * Use this to avoid counting unreliable transitions when tracking quality is poor or
         * the prompt output is dominated by noise.
         */
        float min_prompt_mag;

        /**
         * @brief Select the transition detection method.
         *
         * If true (recommended), uses a “phase-dot” detector:
         *   - A candidate transition is detected when:
         *     @f[
         *       \Re\{ P_k \cdot P^*_{k-1} \} < 0
         *     @f]
         *     where @f$P_k@f$ is the current prompt and @f$P^*_{k-1}@f$ the conjugate of the previous.
         *
         * This method is largely insensitive to constant carrier phase rotations and is often
         * more robust during early tracking / imperfect carrier phase alignment.
         *
         * If false, uses a simpler sign-change detector on the real part:
         *   - A candidate transition is detected when sign(Re(P_k)) != sign(Re(P_{k-1})).
         *
         * This assumes the prompt output is already aligned with the data bit polarity
         * (i.e., stable PLL lock and correct navigation bit polarity mapping).
         */
        bool use_phase_dot_detector;

        Config()
            : bit_period_ms(20),
              epoch_ms(1),
              min_events_for_lock(10),
              dominance_ratio(0.6),
              stable_best_required(5),
              min_prompt_mag(0.0f),
              use_phase_dot_detector(true)
        {
        }
    };

    /**
     * @brief Construct a histogram bit synchronizer with the provided configuration.
     *
     * Initializes internal counters and allocates the histogram with bins() entries,
     * all set to zero.
     *
     * @param cfg Configuration parameters.
     */
    explicit HistogramBitSynchronizer(const Config& cfg)
        : cfg_(cfg),
          hist_(),
          total_events_(0),
          epoch_count_(0),
          last_prompt_(0.0f, 0.0f),
          edge_phase_(-1),
          last_sign_(+1),
          last_best_bin_(0),
          stable_best_count_(0),
          locked_(false),
          has_last_prompt_(false),
          has_last_sign_(false),
          has_last_best_bin_(false)
    {
        hist_.assign(bins(), 0);
    }

    /**
     * @brief Reset the synchronizer state.
     *
     * Clears the histogram and all internal counters/flags, returning the instance to the
     * pre-lock state:
     *   - locked() becomes false
     *   - edge_phase() becomes -1
     *   - total_events and epoch_count are reset to zero
     */
    void reset();

    /**
     * @brief Update the synchronizer once per epoch.
     *
     * This method should be called at a fixed cadence defined by Config::epoch_ms
     *
     * The method:
     *   - Advances the internal epoch counter,
     *   - Optionally performs candidate transition detection if tracking quality is acceptable,
     *   - Updates the phase histogram on detected transitions,
     *   - Evaluates lock once enough events have been gathered.
     *
     * @param prompt Prompt correlator output for the current epoch.
     * @param tracking_quality_ok Indicates whether tracking quality is sufficient to trust
     *        the prompt sample for transition detection (e.g., code/carrier lock metrics).
     *
     * @return True only on the epoch when lock is first declared; false otherwise
     *         (including subsequent epochs after lock has been achieved).
     */
    bool update(const std::complex<float>& prompt, bool tracking_quality_ok);

    /**
     * @brief Query whether the synchronizer has achieved lock.
     *
     * @return True if lock has been declared, false otherwise.
     */
    bool locked() const { return locked_; }

    /**
     * @brief Get the estimated bit edge phase bin.
     *
     * The edge phase is expressed as an integer histogram bin index in the range
     * [0, bins()-1] when locked. The interpretation is “which epoch phase within the
     * bit period is most likely to contain a navigation bit transition.”
     *
     * @return Estimated edge phase bin index, or -1 if not locked.
     */
    int edge_phase() const { return edge_phase_; }

    /**
     * @brief Predict whether a given epoch index corresponds to a bit edge.
     *
     * For a given epoch index @p k (0-based), this function returns true when @p k is
     * aligned with the currently estimated edge phase (i.e., the predicted transition epoch),
     * and false otherwise.
     *
     * If not locked, this always returns false.
     *
     * @param k Epoch index (0-based, consistent with the caller's epoch counting).
     * @return True if @p k is the predicted edge epoch; false otherwise.
     */
    bool is_edge_epoch(std::int64_t k) const;

    /**
     * @brief Return the number of histogram bins.
     *
     * Derived from the bit period and epoch duration, e.g.:
     *   bins = bit_period_ms / epoch_ms
     *
     * @return Number of histogram bins.
     */
    int bins() const;

    /**
     * @brief Access the internal histogram (read-only).
     *
     * Each entry counts how many detected candidate transitions occurred at the
     * corresponding phase bin within the bit period.
     *
     * @return Reference to the histogram vector.
     */
    const std::vector<int>& get_histogram() const { return hist_; }

    /**
     * @brief Total number of detected transition events accumulated into the histogram.
     *
     * @return Total detected events.
     */
    std::int64_t get_total_events() const { return total_events_; }

    /**
     * @brief Total number of epochs processed by update().
     *
     * This counter increments once per call to update(), regardless of whether a transition
     * is detected or whether tracking_quality_ok is true.
     *
     * @return Total processed epochs.
     */
    std::int64_t get_epoch_count() const { return epoch_count_; }

    /**
     * @brief Return the number of epochs until the next predicted navigation bit edge.
     *
     * When the synchronizer is locked, this function computes the forward distance
     * (in epochs) from the most recently processed epoch to the next epoch that is
     * aligned with the estimated bit-edge phase.
     *
     * The result is expressed modulo the bit period and has the following meaning:
     *   - 0  : the current epoch corresponds to the predicted start of a new navigation bit
     *   - >0 : number of epochs remaining until the next bit boundary
     *
     * The computation is based on the internal epoch counter advanced by update(),
     * assuming that update() is called once per epoch with a constant cadence
     * equal to Config::epoch_ms.
     *
     * If the synchronizer is not locked, or if the configuration yields an invalid
     * number of bins, this function returns -1.
     *
     * @return Number of epochs until the next predicted bit edge, or -1 if not locked
     *         or if the bit period configuration is invalid.
     */
    int epochs_until_next_edge() const;

private:
    void best_bin_and_count(int& best_bin, int& best_count) const;

    Config cfg_;
    std::vector<int> hist_;

    std::int64_t total_events_;
    std::int64_t epoch_count_;

    std::complex<float> last_prompt_;  // Sign history (for simple detector)

    int edge_phase_;
    int last_sign_;          // Sign history (for simple detector)
    int last_best_bin_;      // Stability tracking for best bin
    int stable_best_count_;  // Stability tracking for best bin

    bool locked_;
    bool has_last_prompt_;    // Prompt history (for dot detector)
    bool has_last_sign_;      // Sign history (for simple detector)
    bool has_last_best_bin_;  // Stability tracking for best bin
};

/** \} */
/** \} */
#endif  // GNSS_SDR_BIT_SYNCHRONIZER_H