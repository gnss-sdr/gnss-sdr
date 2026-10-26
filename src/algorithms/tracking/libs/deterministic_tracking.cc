/*!
 * \file deterministic_tracking.cc
 * \brief Bookkeeping of a tracking block in deterministic processing
 * \author Carles Fernandez Prades, 2026 cfernandez(at)cttc.es
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

#include "deterministic_tracking.h"
#include <gnuradio/block_detail.h>
#include <gnuradio/buffer.h>
#include <algorithm>
#include <exception>

#if GNURADIO_HAS_BUFFER_READER_H
#include <gnuradio/buffer_reader.h>
#endif

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


DeterministicTracking::DeterministicTracking(gr::block* block)
    : d_block(block)
{
    auto& clock = DeterministicClock::instance();
    if (clock.enabled())
        {
            d_producer = clock.register_producer(DeterministicProducer::Kind::Tracking);
            d_producer->set_bound([this]() { return downstream_bound(); });
        }
}


DeterministicTracking::~DeterministicTracking()
{
    if (d_producer)
        {
            d_producer->set_bound(nullptr);
        }
}


void DeterministicTracking::require_span(uint64_t samples) const
{
    if (d_producer)
        {
            DeterministicClock::instance().require_span(samples);
        }
}


void DeterministicTracking::start(uint64_t acq_stamp)
{
    if (!d_producer)
        {
            return;
        }
    // At a sample that the block has not consumed yet
    std::lock_guard<std::mutex> lock(d_mutex);
    d_start = acq_stamp + DeterministicClock::instance().lookahead();
    d_session_start = d_start;
    d_fault = NO_SAMPLE_STAMP;
}


uint64_t DeterministicTracking::pull_in_sample()
{
    std::lock_guard<std::mutex> lock(d_mutex);
    return d_start;
}


uint64_t DeterministicTracking::samples_to_pull_in(uint64_t position, uint32_t channel)
{
    if (!d_producer)
        {
            return 0;
        }
    std::lock_guard<std::mutex> lock(d_mutex);
    if (d_start == NO_SAMPLE_STAMP)
        {
            return 0;
        }
    if (position < d_start)
        {
            return d_start - position;
        }
    if (position > d_start)
        {
            LOG(ERROR) << "Deterministic pull-in in channel " << channel << " started "
                       << position - d_start << " samples after its stamped start " << d_start;
        }
    d_start = NO_SAMPLE_STAMP;
    return 0;
}


void DeterministicTracking::record_output(uint64_t item, uint64_t stamp)
{
    if (!d_producer)
        {
            return;
        }
    std::lock_guard<std::mutex> lock(d_mutex);
    d_output_stamps.emplace_back(item, stamp);
}


void DeterministicTracking::raise_loss(uint64_t stamp)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_loss = stamp;
}


void DeterministicTracking::dispatch(uint32_t channel, const std::function<void(int)>& fallback)
{
    if (!d_producer)
        {
            return;
        }
    uint64_t stamp = NO_SAMPLE_STAMP;
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        std::swap(stamp, d_loss);
    }
    if (stamp != NO_SAMPLE_STAMP &&
        !DeterministicClock::instance().emit_channel_event(static_cast<int32_t>(channel), DeterministicClock::TRK_LOSS, stamp))
        {
            fallback(DeterministicClock::TRK_LOSS);
        }
}


void DeterministicTracking::publish(uint64_t position)
{
    if (d_producer)
        {
            d_producer->set_progress(position);
        }
}


bool DeterministicTracking::take_fault(const pmt::pmt_t& msg, uint64_t position, bool tracking, uint32_t channel)
{
    // (1 . sample stamp of the offending symbol), see stamp_telemetry_message()
    if (!pmt::is_pair(msg) || !pmt::is_integer(pmt::car(msg)) || pmt::to_long(pmt::car(msg)) != 1 || !pmt::is_uint64(pmt::cdr(msg)))
        {
            return false;
        }
    if (!d_producer)
        {
            return true;
        }
    const uint64_t stamp = pmt::to_uint64(pmt::cdr(msg));
    std::lock_guard<std::mutex> lock(d_mutex);
    // Ignore faults of a previous satellite
    if (tracking && stamp >= d_session_start)
        {
            const uint64_t effective = stamp + DeterministicClock::instance().lookahead();
            if (position > effective)
                {
                    LOG(ERROR) << "Deterministic telemetry fault in channel " << channel << " arrived "
                               << position - effective << " samples late";
                }
            d_fault = std::min(d_fault, stamp);
        }
    return true;
}


bool DeterministicTracking::fault_due(uint64_t position, uint32_t channel)
{
    if (!d_producer)
        {
            return false;
        }
    std::lock_guard<std::mutex> lock(d_mutex);
    // Read the decoder's synchronous handoff on every work call. A GNU Radio
    // message may arrive after the scheduler has already checked its queue.
    auto& clock = DeterministicClock::instance();
    const uint64_t reported = clock.take_telemetry_fault(static_cast<int32_t>(channel), d_session_start);
    if (reported != NO_SAMPLE_STAMP && position > reported + clock.lookahead())
        {
            LOG(ERROR) << "Deterministic telemetry fault in channel " << channel << " arrived "
                       << position - (reported + clock.lookahead()) << " samples late";
        }
    d_fault = std::min(d_fault, reported);
    if (d_fault == NO_SAMPLE_STAMP || position < d_fault + DeterministicClock::instance().lookahead())
        {
            return false;
        }
    d_fault = NO_SAMPLE_STAMP;
    return true;
}


uint64_t DeterministicTracking::downstream_bound()
{
    const auto state = d_block->detail();
    if (!state || state->noutputs() < 1)
        {
            return NO_SAMPLE_STAMP;
        }
    const auto output = state->output(0);
    if (!output)
        {
            return NO_SAMPLE_STAMP;
        }
    // A telemetry decoder consumes a symbol before publishing its faults, so
    // count only the symbols that it has finished with (see TelemetryWorkScope)
    const auto& clock = DeterministicClock::instance();
    uint64_t consumed = NO_SAMPLE_STAMP;
    {
        gr::thread::scoped_lock lock(*output->mutex());
        for (size_t r = 0; r < output->nreaders(); r++)
            {
                const auto& reader = output->reader(r);
                const gr::block* reader_block = nullptr;
                try
                    {
                        reader_block = reader->link().get();
                    }
                catch (const std::exception&)
                    {
                        // The reading block is gone
                    }
                consumed = std::min(consumed, clock.consumed(reader_block, reader->nitems_read()));
            }
    }
    std::lock_guard<std::mutex> lock(d_mutex);
    while (!d_output_stamps.empty() && d_output_stamps.front().first < consumed)
        {
            d_output_stamps.pop_front();
        }
    return d_output_stamps.empty() ? NO_SAMPLE_STAMP : d_output_stamps.front().second;
}
