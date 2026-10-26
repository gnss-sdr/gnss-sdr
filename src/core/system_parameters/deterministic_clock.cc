/*!
 * \file deterministic_clock.cc
 * \brief Sample-time control plane for deterministic processing
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

#include "deterministic_clock.h"
#include <algorithm>
#include <utility>

namespace
{
thread_local uint64_t current_scope_stamp = NO_SAMPLE_STAMP;
}  // namespace


DeterministicProducer::DeterministicProducer(Kind kind)
    : progress_(0),
      kind_(kind)
{
}


void DeterministicProducer::set_progress(uint64_t samples)
{
    uint64_t current = progress_.load(std::memory_order_relaxed);
    while (samples > current && !progress_.compare_exchange_weak(current, samples, std::memory_order_release, std::memory_order_relaxed))
        {
        }
}


void DeterministicProducer::set_bound(std::function<uint64_t()> bound)
{
    std::lock_guard<std::mutex> lock(bound_mutex_);
    bound_ = std::move(bound);
}


uint64_t DeterministicProducer::progress() const
{
    const uint64_t progress = progress_.load(std::memory_order_acquire);
    std::lock_guard<std::mutex> lock(bound_mutex_);
    return bound_ ? std::min(progress, bound_()) : progress;
}


DeterministicClock::DeterministicClock()
    : horizon_(0),
      lookahead_(0),
      enabled_(false),
      sampling_rate_(0.0),
      tick_(0)
{
}


DeterministicClock& DeterministicClock::instance()
{
    static DeterministicClock clock;
    return clock;
}


void DeterministicClock::enable(double sampling_rate, uint64_t lookahead_samples, uint64_t tick_samples)
{
    std::lock_guard<std::mutex> lock(mutex_);
    producers_.clear();
    sinks_.clear();
    telemetry_faults_.clear();
    sampling_rate_ = sampling_rate;
    tick_ = std::max<uint64_t>(tick_samples, 1);
    lookahead_.store(std::max<uint64_t>(lookahead_samples, 1), std::memory_order_release);
    horizon_.store(0, std::memory_order_release);
    enabled_.store(true, std::memory_order_release);
    std::lock_guard<std::mutex> consumed_lock(consumed_mutex_);
    consumed_.clear();
}


void DeterministicClock::disable()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_.store(false, std::memory_order_release);
        producers_.clear();
        sinks_.clear();
        telemetry_faults_.clear();
        horizon_.store(NO_SAMPLE_STAMP, std::memory_order_release);
    }
    {
        std::lock_guard<std::mutex> lock(consumed_mutex_);
        consumed_.clear();
    }
    horizon_cv_.notify_all();
}


double DeterministicClock::sampling_rate() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return sampling_rate_;
}


uint64_t DeterministicClock::tick() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return tick_;
}


void DeterministicClock::require_span(uint64_t samples)
{
    // Leave room for the integration that follows the span
    const uint64_t needed = samples + samples / 4 + 1;
    uint64_t current = lookahead_.load(std::memory_order_relaxed);
    while (needed > current && !lookahead_.compare_exchange_weak(current, needed, std::memory_order_release, std::memory_order_relaxed))
        {
        }
}


uint64_t DeterministicClock::effective_sample(uint64_t stamp) const
{
    return stamp == NO_SAMPLE_STAMP ? 0 : stamp + lookahead();
}


std::shared_ptr<DeterministicProducer> DeterministicClock::register_producer(DeterministicProducer::Kind kind)
{
    auto producer = std::make_shared<DeterministicProducer>(kind);
    std::lock_guard<std::mutex> lock(mutex_);
    producers_.push_back(producer);
    return producer;
}


size_t DeterministicClock::count_producers(DeterministicProducer::Kind kind) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<size_t>(std::count_if(producers_.begin(), producers_.end(),
        [kind](const std::weak_ptr<DeterministicProducer>& weak) {
            const auto producer = weak.lock();
            return producer && producer->kind() == kind;
        }));
}


uint64_t DeterministicClock::min_progress() const
{
    std::vector<std::shared_ptr<DeterministicProducer>> live;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        live.reserve(producers_.size());
        for (const auto& weak : producers_)
            {
                auto producer = weak.lock();
                if (producer)
                    {
                        live.push_back(std::move(producer));
                    }
            }
    }
    uint64_t result = NO_SAMPLE_STAMP;
    for (const auto& producer : live)
        {
            result = std::min(result, producer->progress());
        }
    return result;
}


void DeterministicClock::set_channel_event_sink(int32_t channel, ChannelEventSink sink)
{
    std::lock_guard<std::mutex> lock(mutex_);
    sinks_[channel] = std::move(sink);
}


bool DeterministicClock::emit_channel_event(int32_t channel, int event, uint64_t stamp) const
{
    ChannelEventSink sink;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = sinks_.find(channel);
        if (it == sinks_.end() || !it->second)
            {
                return false;
            }
        sink = it->second;
    }
    const Scope scope(stamp);
    sink(event);
    return true;
}


void DeterministicClock::report_telemetry_fault(int32_t channel, uint64_t stamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    telemetry_faults_[channel].push_back(stamp);
}


uint64_t DeterministicClock::take_telemetry_fault(int32_t channel, uint64_t session_start)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = telemetry_faults_.find(channel);
    if (it == telemetry_faults_.end())
        {
            return NO_SAMPLE_STAMP;
        }
    uint64_t stamp = NO_SAMPLE_STAMP;
    for (const auto candidate : it->second)
        {
            if (candidate >= session_start)
                {
                    stamp = std::min(stamp, candidate);
                }
        }
    telemetry_faults_.erase(it);
    return stamp;
}


void DeterministicClock::set_consumed(const void* consumer, uint64_t items)
{
    std::lock_guard<std::mutex> lock(consumed_mutex_);
    consumed_[consumer] = items;
}


uint64_t DeterministicClock::consumed(const void* consumer, uint64_t read) const
{
    std::lock_guard<std::mutex> lock(consumed_mutex_);
    const auto it = consumed_.find(consumer);
    return it == consumed_.end() ? read : std::min(it->second, read);
}


void DeterministicClock::advance_horizon(uint64_t samples)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (samples <= horizon_.load(std::memory_order_relaxed))
            {
                return;
            }
        horizon_.store(samples, std::memory_order_release);
    }
    horizon_cv_.notify_all();
}


void DeterministicClock::open_horizon()
{
    advance_horizon(NO_SAMPLE_STAMP);
}


bool DeterministicClock::wait_beyond(uint64_t released, std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    return horizon_cv_.wait_for(lock, timeout, [this, released]() { return horizon_.load(std::memory_order_acquire) > released; });
}


DeterministicClock::Scope::Scope(uint64_t stamp)
    : previous_(current_scope_stamp)
{
    current_scope_stamp = stamp;
}


DeterministicClock::Scope::~Scope()
{
    current_scope_stamp = previous_;
}


uint64_t DeterministicClock::scope_stamp()
{
    return current_scope_stamp;
}
