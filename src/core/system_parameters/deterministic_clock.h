/*!
 * \file deterministic_clock.h
 * \brief Sample-time control plane for deterministic processing
 * \author Carles Fernandez Prades, 2026 cfernandez(at)cttc.es
 *
 * With GNSS-SDR.deterministic=true, every action that crosses threads takes
 * effect at a sample stamp that depends only on the input stream:
 *
 * - Event producers (acquisition and tracking blocks) publish their progress:
 *   the sample count below which they have raised all their channel events.
 * - The control thread handles the events in stamp order, up to the lowest
 *   progress c, and then lets the stream advance up to the horizon c + L,
 *   where L is the lookahead.
 * - An action caused by an event stamped s takes effect at sample s + L,
 *   which no block has consumed yet.
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

#ifndef GNSS_SDR_DETERMINISTIC_CLOCK_H
#define GNSS_SDR_DETERMINISTIC_CLOCK_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

//! Marks the absence of a sample stamp
constexpr uint64_t NO_SAMPLE_STAMP = std::numeric_limits<uint64_t>::max();

/*!
 * \brief Channel data beyond an epoch that the observables block takes into
 * account before computing it, in ms of signal: one tracking output after the
 * epoch, for the interpolation.
 */
constexpr uint32_t DETERMINISTIC_OBSERVABLES_MARGIN_MS = 40;

/*!
 * \brief Signal time that the telemetry decoders may need to deliver a
 * symbol after tracking has produced it, in ms.
 */
constexpr uint32_t DETERMINISTIC_SYMBOL_MARGIN_MS = 20;

/*!
 * \brief Progress of a block that can raise channel events, or that receives
 * messages to apply at a sample stamp.
 */
class DeterministicProducer
{
public:
    enum class Kind
    {
        Acquisition,
        Tracking,
        Observables
    };

    explicit DeterministicProducer(Kind kind);

    Kind kind() const { return kind_; }

    /*!
     * \brief Sets the sample count below which all the events of this block
     * have been raised. It never decreases.
     */
    void set_progress(uint64_t samples);

    /*!
     * \brief Sets a bound on the progress that is computed every time the
     * control thread reads it, e.g. from the state of downstream blocks.
     */
    void set_bound(std::function<uint64_t()> bound);

    //! Progress, limited by the bound if there is one
    uint64_t progress() const;

private:
    std::atomic<uint64_t> progress_;
    mutable std::mutex bound_mutex_;
    std::function<uint64_t()> bound_;
    const Kind kind_;
};


/*!
 * \brief Process-wide state of the deterministic processing mode.
 */
class DeterministicClock
{
public:
    //! Channel events, numbered as the "events" messages of the channel blocks
    enum ChannelEvent : int
    {
        ACQ_POSITIVE = 1,
        ACQ_NEGATIVE = 2,
        TRK_LOSS = 3
    };

    using ChannelEventSink = std::function<void(int event)>;

    static DeterministicClock& instance();

    DeterministicClock(const DeterministicClock&) = delete;
    DeterministicClock& operator=(const DeterministicClock&) = delete;

    /*!
     * \brief Enables the mode. Blocks created afterwards register as producers.
     * \param sampling_rate Rate of the stamps, in samples per second.
     * \param lookahead_samples Delay between an event and its effect.
     * \param tick_samples Period of the control-plane maintenance tasks.
     */
    void enable(double sampling_rate, uint64_t lookahead_samples, uint64_t tick_samples);

    //! Disables the mode and forgets producers, sinks and horizon.
    void disable();

    bool enabled() const { return enabled_.load(std::memory_order_acquire); }
    double sampling_rate() const;
    uint64_t lookahead() const { return lookahead_.load(std::memory_order_acquire); }
    uint64_t tick() const;

    /*!
     * \brief Raises the lookahead so that a producer that needs \p samples of
     * input beyond its progress can always get them. Call before the
     * flowgraph starts.
     */
    void require_span(uint64_t samples);

    /*!
     * \brief Sample at which an action caused by an event stamped \p stamp
     * takes effect. Stream start if \p stamp is NO_SAMPLE_STAMP.
     */
    uint64_t effective_sample(uint64_t stamp) const;

    std::shared_ptr<DeterministicProducer> register_producer(DeterministicProducer::Kind kind);
    size_t count_producers(DeterministicProducer::Kind kind) const;

    //! Lowest progress among live producers, NO_SAMPLE_STAMP if there are none
    uint64_t min_progress() const;

    /*!
     * \brief Lets a channel handle its events in the thread that raises
     * them, so that they reach the control queue before the producer
     * progresses past their stamp.
     */
    void set_channel_event_sink(int32_t channel, ChannelEventSink sink);

    /*!
     * \brief Hands an event stamped \p stamp to the sink of \p channel.
     * \return false if the channel has no sink.
     */
    bool emit_channel_event(int32_t channel, int event, uint64_t stamp) const;

    //! Records a telemetry fault before the decoder releases its progress.
    void report_telemetry_fault(int32_t channel, uint64_t stamp);

    /*!
     * \brief Drains queued faults, returning the earliest stamp in this
     * tracking session, or NO_SAMPLE_STAMP. Ignores stamps before \p session_start.
     */
    uint64_t take_telemetry_fault(int32_t channel, uint64_t session_start);

    /*!
     * \brief A block that consumes its input before it publishes the messages
     * that the input causes reports here that it has finished with its first
     * \p items input items, messages included.
     */
    void set_consumed(const void* consumer, uint64_t items);

    /*!
     * \brief Input items that \p consumer has finished with: those reported by
     * set_consumed(), or \p read (its read count) if it reports none.
     */
    uint64_t consumed(const void* consumer, uint64_t read) const;

    //! Number of samples that the stream may release
    uint64_t horizon() const { return horizon_.load(std::memory_order_acquire); }

    //! Moves the horizon forward to \p samples (never backwards)
    void advance_horizon(uint64_t samples);

    //! Releases the whole stream (end of the run)
    void open_horizon();

    /*!
     * \brief Waits up to \p timeout for the horizon to exceed \p released.
     * \return true if it does.
     */
    bool wait_beyond(uint64_t released, std::chrono::milliseconds timeout);

    /*!
     * \brief Stamp of the event that the calling thread is handling. Actions
     * taken inside the scope are caused by that event.
     */
    class Scope
    {
    public:
        explicit Scope(uint64_t stamp);
        ~Scope();
        Scope(const Scope&) = delete;
        Scope& operator=(const Scope&) = delete;

    private:
        uint64_t previous_;
    };

    //! Stamp of the innermost Scope of the calling thread, or NO_SAMPLE_STAMP
    static uint64_t scope_stamp();

private:
    DeterministicClock();

    mutable std::mutex mutex_;
    std::condition_variable horizon_cv_;
    std::vector<std::weak_ptr<DeterministicProducer>> producers_;
    std::map<int32_t, ChannelEventSink> sinks_;
    std::map<int32_t, std::vector<uint64_t>> telemetry_faults_;
    mutable std::mutex consumed_mutex_;
    std::map<const void*, uint64_t> consumed_;
    std::atomic<uint64_t> horizon_;
    std::atomic<uint64_t> lookahead_;
    std::atomic<bool> enabled_;
    double sampling_rate_;
    uint64_t tick_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_DETERMINISTIC_CLOCK_H
