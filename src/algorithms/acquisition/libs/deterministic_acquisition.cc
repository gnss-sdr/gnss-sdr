/*!
 * \file deterministic_acquisition.cc
 * \brief Bookkeeping of an acquisition block in deterministic processing
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

#include "deterministic_acquisition.h"
#include <algorithm>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


DeterministicAcquisition::DeterministicAcquisition()
{
    auto& clock = DeterministicClock::instance();
    if (clock.enabled())
        {
            d_producer = clock.register_producer(DeterministicProducer::Kind::Acquisition);
        }
}


void DeterministicAcquisition::require_span(uint64_t samples) const
{
    if (d_producer)
        {
            DeterministicClock::instance().require_span(clock_sample(samples));
        }
}


void DeterministicAcquisition::activate(uint64_t consumed, uint32_t channel)
{
    if (!d_producer)
        {
            return;
        }
    const uint64_t cause = DeterministicClock::scope_stamp();
    if (cause == NO_SAMPLE_STAMP && consumed > 0)
        {
            LOG(WARNING) << "Deterministic acquisition started without a stamped cause in channel " << channel;
        }
    std::lock_guard<std::mutex> lock(d_mutex);
    if (cause == NO_SAMPLE_STAMP)
        {
            d_start = consumed;
        }
    else
        {
            // First input sample at or after the effective stream sample
            d_start = (DeterministicClock::instance().effective_sample(cause) + d_decimation - 1) / d_decimation;
        }
    d_floor = std::max(d_floor, d_start);
}


bool DeterministicAcquisition::ready_to_start(uint64_t consumed, uint64_t window_offset, uint64_t samples_per_item, int available_items, int& items, uint32_t channel)
{
    items = 0;
    if (!d_producer)
        {
            return true;
        }
    std::lock_guard<std::mutex> lock(d_mutex);
    if (d_start != NO_SAMPLE_STAMP && d_start > 0)
        {
            // A search started at the stream start keeps the window it has
            const uint64_t target = d_start + window_offset;
            if (consumed < target)
                {
                    const uint64_t needed = (target - consumed + samples_per_item - 1) / samples_per_item;
                    items = static_cast<int>(std::min<uint64_t>(needed, static_cast<uint64_t>(std::max(available_items, 0))));
                    return false;
                }
            if (consumed >= target + samples_per_item)
                {
                    LOG(ERROR) << "Deterministic acquisition in channel " << channel << " started " << consumed - target
                               << " samples after its stamped start " << d_start;
                }
        }
    d_start = NO_SAMPLE_STAMP;
    d_floor = std::max(d_floor, consumed >= window_offset ? consumed - window_offset : 0);
    return true;
}


void DeterministicAcquisition::begin_window(uint64_t sample)
{
    if (!d_producer)
        {
            return;
        }
    std::lock_guard<std::mutex> lock(d_mutex);
    d_floor = std::max(d_floor, sample);
}


void DeterministicAcquisition::raise(int event, uint64_t stamp)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_event = event;
    d_stamp = stamp;
}


void DeterministicAcquisition::begin_call(uint64_t consumed)
{
    if (!d_producer)
        {
            return;
        }
    std::lock_guard<std::mutex> lock(d_mutex);
    d_call_start = consumed;
}


void DeterministicAcquisition::raise(int event)
{
    std::lock_guard<std::mutex> lock(d_mutex);
    d_event = event;
    d_stamp = d_call_start;
}


void DeterministicAcquisition::dispatch(uint32_t channel, const std::function<void(int)>& fallback)
{
    if (!d_producer)
        {
            return;
        }
    uint64_t stamp = NO_SAMPLE_STAMP;
    int event = 0;
    {
        std::lock_guard<std::mutex> lock(d_mutex);
        if (d_claimed || d_stamp == NO_SAMPLE_STAMP)
            {
                return;
            }
        d_claimed = true;
        stamp = d_stamp;
        event = d_event;
    }
    if (!DeterministicClock::instance().emit_channel_event(static_cast<int32_t>(channel), event, clock_sample(stamp)))
        {
            fallback(event);
        }
    std::lock_guard<std::mutex> lock(d_mutex);
    if (d_stamp == stamp)
        {
            d_stamp = NO_SAMPLE_STAMP;
        }
    d_claimed = false;
}


void DeterministicAcquisition::publish(uint64_t consumed, bool searching, uint32_t channel)
{
    if (!d_producer)
        {
            return;
        }
    std::lock_guard<std::mutex> lock(d_mutex);
    uint64_t progress = consumed;
    if (searching || d_stamp != NO_SAMPLE_STAMP || d_start != NO_SAMPLE_STAMP)
        {
            progress = std::min(progress, d_floor);
        }
    if (progress < d_published)
        {
            LOG(WARNING) << "Deterministic progress of the acquisition in channel " << channel
                         << " moved back from " << d_published << " to " << progress;
            return;
        }
    d_published = progress;
    d_producer->set_progress(clock_sample(progress));
}
