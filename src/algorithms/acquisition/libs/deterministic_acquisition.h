/*!
 * \file deterministic_acquisition.h
 * \brief Bookkeeping of an acquisition block in deterministic processing
 * \author Carles Fernandez Prades, 2026 cfernandez(at)cttc.es
 *
 * With GNSS-SDR.deterministic=true (see DeterministicClock), an acquisition
 * block starts each search at a stamped sample, publishes its progress, and
 * raises its results in its own thread, so that they reach the control queue
 * before its progress passes their stamp.
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

#ifndef GNSS_SDR_DETERMINISTIC_ACQUISITION_H
#define GNSS_SDR_DETERMINISTIC_ACQUISITION_H

#include "deterministic_clock.h"
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup acquisition_libs acquisition_libs
 * \{ */


/*!
 * \brief Deterministic-processing state of an acquisition block. It does
 * nothing unless GNSS-SDR.deterministic=true.
 */
class DeterministicAcquisition
{
public:
    //! Registers the block as a producer of channel events, if enabled
    DeterministicAcquisition();

    bool enabled() const { return static_cast<bool>(d_producer); }

    /*!
     * \brief Declares that the block reads the stream decimated by \p factor
     * (GNSS-SDR.use_acquisition_resampler=true). The samples passed to this
     * class are then input samples of the block, and they are converted to
     * samples of the clock, which counts the stream before decimation. The
     * decimator reads \p factor stream samples per output and does not look
     * ahead (its history is zero-filled), so input sample n is stream sample
     * n * factor. Call before require_span().
     */
    void set_decimation(uint64_t factor) { d_decimation = factor > 0 ? factor : 1; }

    //! Stream sample of the clock at input sample \p sample of the block
    uint64_t clock_sample(uint64_t sample) const { return sample * d_decimation; }

    /*!
     * \brief Declares the input that a search needs beyond the first sample
     * of its window, so that the lookahead covers it.
     */
    void require_span(uint64_t samples) const;

    /*!
     * \brief Call when the search is activated. It starts one lookahead after
     * the event that the calling thread is handling, or at \p consumed
     * outside an event (e.g., at connection time).
     */
    void activate(uint64_t consumed, uint32_t channel);

    /*!
     * \brief Whether the first window of a new search can start now, at the
     * first item boundary at or after its stamped sample. If not, \p items is
     * the number of input items to consume first (possibly 0, if none is
     * available yet).
     * \param consumed Samples consumed so far.
     * \param window_offset Samples between the start of the window and the
     * consumed count (history - 1 for stream inputs).
     * \param samples_per_item Samples per input item (vector length).
     * \param available_items New input items available in this call.
     * \param[out] items Input items to consume before the window can start.
     * \param channel For logging only.
     */
    bool ready_to_start(uint64_t consumed, uint64_t window_offset, uint64_t samples_per_item, int available_items, int& items, uint32_t channel);

    //! The current window starts at \p sample: no later result has a lower stamp
    void begin_window(uint64_t sample);

    /*!
     * \brief Defers a result (DeterministicClock::ACQ_POSITIVE or
     * ACQ_NEGATIVE), which may be raised with the block lock held, to
     * dispatch().
     */
    void raise(int event, uint64_t stamp);

    /*!
     * \brief Records the consumed count when a call to general_work() begins,
     * for blocks that decide a result in a call after the one that consumed
     * its last window: it is then the end of that window.
     */
    void begin_call(uint64_t consumed);

    //! Defers a result stamped with the count recorded by begin_call()
    void raise(int event);

    /*!
     * \brief Applies the deferred result, if any, in the calling thread.
     * Call it without the block lock: a repeated search restarts the block.
     * Calls \p fallback (which should publish the event) if the channel has
     * no event sink.
     */
    void dispatch(uint32_t channel, const std::function<void(int)>& fallback);

    /*!
     * \brief Publishes the progress: \p consumed, or the start of the current
     * window while a search, or a result, is pending.
     */
    void publish(uint64_t consumed, bool searching, uint32_t channel);

private:
    std::shared_ptr<DeterministicProducer> d_producer;
    std::mutex d_mutex;
    uint64_t d_decimation{1};           // stream samples per input sample
    uint64_t d_start{NO_SAMPLE_STAMP};  // first sample of the next search
    uint64_t d_floor{0};                // no result of the current search can have a lower stamp
    uint64_t d_stamp{NO_SAMPLE_STAMP};  // deferred result
    uint64_t d_published{0};
    uint64_t d_call_start{0};
    int d_event{0};
    bool d_claimed{false};  // a thread is dispatching the deferred result
};


/** \} */
/** \} */
#endif  // GNSS_SDR_DETERMINISTIC_ACQUISITION_H
