/*!
 * \file deterministic_tracking.h
 * \brief Bookkeeping of a tracking block in deterministic processing
 * \author Carles Fernandez Prades, 2026 cfernandez(at)cttc.es
 *
 * With GNSS-SDR.deterministic=true (see DeterministicClock), a tracking block
 * pulls in at a stamped sample, raises loss of lock in its own thread, applies
 * telemetry faults at a stamped sample, and publishes a progress that its
 * telemetry decoder cannot fall too far behind.
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

#ifndef GNSS_SDR_DETERMINISTIC_TRACKING_H
#define GNSS_SDR_DETERMINISTIC_TRACKING_H

#include "deterministic_clock.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


/*!
 * \brief Deterministic-processing state of a tracking block. It does nothing
 * unless GNSS-SDR.deterministic=true.
 */
class DeterministicTracking
{
public:
    /*!
     * \brief Registers \p block as a producer of channel events, if enabled.
     * Its progress is bounded by the first of its outputs that the blocks
     * downstream have not finished with yet (see DeterministicClock::consumed()).
     */
    explicit DeterministicTracking(gr::block* block);
    ~DeterministicTracking();
    DeterministicTracking(const DeterministicTracking&) = delete;
    DeterministicTracking& operator=(const DeterministicTracking&) = delete;

    bool enabled() const { return static_cast<bool>(d_producer); }

    //! Declares the input needed beyond the progress (e.g., the forecast)
    void require_span(uint64_t samples) const;

    //! start_tracking(): the pull-in takes place one lookahead after \p acq_stamp
    void start(uint64_t acq_stamp);

    //! Input sample of the pending pull-in, NO_SAMPLE_STAMP if none
    uint64_t pull_in_sample();

    /*!
     * \brief Samples to consume before pulling in at the stamped sample, from
     * input position \p position (0 to pull in now). Clears the pending start
     * once it is reached.
     */
    uint64_t samples_to_pull_in(uint64_t position, uint32_t channel);

    //! Remembers the sample stamp of output item \p item
    void record_output(uint64_t item, uint64_t stamp);

    //! Defers a loss of lock, which may be raised with the block lock held, to dispatch()
    void raise_loss(uint64_t stamp);

    /*!
     * \brief Applies the deferred loss of lock, if any, in the calling thread.
     * Calls \p fallback (which should publish the event) if the channel has
     * no event sink.
     */
    void dispatch(uint32_t channel, const std::function<void(int)>& fallback);

    //! Publishes the input position reached
    void publish(uint64_t position);

    /*!
     * \brief Takes a telemetry fault stamped by the telemetry decoder.
     * \return false if \p msg is not one.
     */
    bool take_fault(const pmt::pmt_t& msg, uint64_t position, bool tracking, uint32_t channel);

    //! Reads the synchronous fault handoff; true once when a fault is due at \p position.
    bool fault_due(uint64_t position, uint32_t channel);

private:
    uint64_t downstream_bound();

    gr::block* d_block;
    std::shared_ptr<DeterministicProducer> d_producer;
    std::mutex d_mutex;
    std::deque<std::pair<uint64_t, uint64_t>> d_output_stamps;  // (output item, sample stamp) not read downstream yet
    uint64_t d_start{NO_SAMPLE_STAMP};                          // pull-in sample
    uint64_t d_session_start{0};                                // pull-in sample of the current satellite
    uint64_t d_loss{NO_SAMPLE_STAMP};                           // deferred loss of lock
    uint64_t d_fault{NO_SAMPLE_STAMP};                          // telemetry fault waiting for its effective sample
};


/** \} */
/** \} */
#endif  // GNSS_SDR_DETERMINISTIC_TRACKING_H
