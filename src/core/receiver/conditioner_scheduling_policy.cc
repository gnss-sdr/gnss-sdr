/*!
 * \file conditioner_scheduling_policy.cc
 * \brief Portable, configuration-aware conditioner publication planning
 * \author Carles Fernandez Prades, 2026 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2026 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "conditioner_scheduling_policy.h"
#include "configuration_interface.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

std::string ConditionerSchedulingPolicy::trim(const std::string& text)
{
    const auto begin = text.find_first_not_of(" \t\r\n");
    return begin == std::string::npos ? std::string() : text.substr(begin, text.find_last_not_of(" \t\r\n") - begin + 1);
}


double ConditionerSchedulingPolicy::number(const std::string& text, const std::string& key)
{
    try
        {
            size_t end = 0;
            const auto value = std::stod(text, &end);
            if (end == text.size() && std::isfinite(value) && value >= 0.0)
                {
                    return value;
                }
        }
    catch (const std::exception&)
        {
        }
    throw std::invalid_argument(key + " must be a finite, nonnegative number");
}


ConditionerSchedulingPolicy::Plan ConditionerSchedulingPolicy::select(
    const ConfigurationInterface& configuration, const std::string& role, const Request& request)
{
    Plan plan;
    plan.readers = request.readers;
    plan.reader_window_items = request.minimum_input_items;
    const auto text = trim(configuration.property(role + ".batch_size_ms", std::string("auto")));
    plan.automatic = text == "auto";
    const double duration = plan.automatic ? 0.0 : number(text, role + ".batch_size_ms");
    if (duration > STREAM_BATCHER_MAX_LATENCY_MS)
        {
            throw std::invalid_argument(role + ".batch_size_ms must be auto or in [0, 20] ms");
        }
    const double observable_ms = number(trim(configuration.property("GNSS-SDR.observable_interval_ms", std::string("20"))), "GNSS-SDR.observable_interval_ms");
    if (observable_ms <= 0.0 || observable_ms > std::numeric_limits<int>::max() || std::floor(observable_ms) != observable_ms)
        {
            throw std::invalid_argument("GNSS-SDR.observable_interval_ms must be a positive integer no larger than INT_MAX");
        }
    plan.max_latency_ms = std::max(duration, std::min(STREAM_BATCHER_DEFAULT_LATENCY_MS, observable_ms));
    if (configuration.is_present(role + ".max_batch_latency_ms"))
        {
            plan.max_latency_ms = number(trim(configuration.property(role + ".max_batch_latency_ms", std::string())), role + ".max_batch_latency_ms");
        }
    if (plan.max_latency_ms <= 0.0 || plan.max_latency_ms > STREAM_BATCHER_MAX_LATENCY_MS || duration > plan.max_latency_ms)
        {
            throw std::invalid_argument(role + ".max_batch_latency_ms must be in (0, 20] and not smaller than the fixed batch duration");
        }
    const bool forced_removal = configuration.is_present(role + ".remove_pass_through");
    plan.remove_identity = configuration.property(role + ".remove_pass_through", duration > 0.0);
    const double buffer_items = number(trim(configuration.property("GNSS-SDR.max_source_buffer_samples", std::string("0"))), "GNSS-SDR.max_source_buffer_samples");
    if (buffer_items > std::numeric_limits<int>::max() || std::floor(buffer_items) != buffer_items)
        {
            throw std::invalid_argument("GNSS-SDR.max_source_buffer_samples must be an integer no larger than INT_MAX");
        }
    plan.max_buffer_items = static_cast<uint64_t>(buffer_items);
    if (plan.max_buffer_items == 1 || plan.max_buffer_items > static_cast<uint64_t>(std::numeric_limits<int>::max()))
        {
            throw std::invalid_argument("GNSS-SDR.max_source_buffer_samples must be 0 or between 2 and INT_MAX");
        }

    // The receiver clock counts samples as they are published, while a channel
    // that is short of CPU, or held back by a reader that stopped consuming,
    // processes them up to one buffer later. The observables block computes an
    // epoch when it is a fixed number of clock ticks old, so the buffer must
    // not hold more signal than that delay, less the time a channel takes to
    // report (its input window and one telemetry symbol) and a guard.
    const auto depth_text = trim(configuration.property(role + ".output_buffer_ms", std::string("auto")));
    const bool automatic_depth = depth_text == "auto";
    const double explicit_depth_ms = automatic_depth ? 0.0 : number(depth_text, role + ".output_buffer_ms");
    const double delay_ms = observables_clock_delay_ms(static_cast<uint32_t>(std::min(observable_ms, 1.0e6)));
    if (explicit_depth_ms > delay_ms)
        {
            throw std::invalid_argument(role + ".output_buffer_ms must be auto or not above " + std::to_string(static_cast<int>(delay_ms)) +
                                        " ms, the delay of the observables for this GNSS-SDR.observable_interval_ms");
        }
    const bool known_rate = std::isfinite(request.sample_rate) && request.sample_rate > 0.0 && request.item_size > 0;
    // GNU Radio up to 3.9 addresses a stream buffer with a signed int.
    const uint64_t addressable_items = known_rate ? static_cast<uint64_t>(std::numeric_limits<int>::max() / 2) / request.item_size : 0;
    const auto set_buffer = [&](Plan& result) {
        if (!known_rate)
            {
                return;
            }
        const uint64_t batch = result.batch_items;
        double budget_ms = delay_ms - std::ceil(1000.0 * static_cast<double>(request.channel_input_items) / request.sample_rate) -
                           CONDITIONED_STREAM_SYMBOL_MS - CONDITIONED_STREAM_GUARD_MS;
        if (!request.feeds_clock)
            {
                // This stream can also trail the clock stream by what is queued
                // in front of it: two batches, and the default buffers (64 KiB
                // each) of its source and conditioning blocks.
                const double queued = 2.0 * static_cast<double>(batch) + 3.0 * 65536.0 / static_cast<double>(request.item_size);
                budget_ms -= std::ceil(1000.0 * queued / request.sample_rate);
            }
        result.alignment_budget_ms = std::max(budget_ms, 0.0);
        result.buffer_over_budget = explicit_depth_ms > result.alignment_budget_ms;
        // An unthrottled replay has no pace to keep: there the receiver waits
        // for its slowest reader, as it always did, unless told otherwise.
        const double depth_ms = automatic_depth ? (request.real_time ? result.alignment_budget_ms : 0.0) : explicit_depth_ms;
        auto window = static_cast<uint64_t>(std::min(std::floor(request.sample_rate * depth_ms / 1000.0), static_cast<double>(addressable_items)));
        if (automatic_depth)
            {
                window = std::min(window, CONDITIONED_STREAM_AUTO_BUFFER_BYTES / request.item_size);
            }
        if (result.max_buffer_items > 0)
            {
                window = std::min(window, result.max_buffer_items);
            }
        if (batch == 0)
            {
                result.min_buffer_items = window;
                return;
            }
        // GNU Radio rounds the buffer of a block down to its output multiple.
        result.min_buffer_items = std::max(result.min_buffer_items, window - window % batch);
    };
    // A reader waiting for W items and a writer waiting for room for a whole
    // batch cannot block each other if the buffer holds both; GNU Radio also
    // offers at most half a buffer per call. In whole batches:
    const auto capacity = [&request](uint64_t batch) {
        return batch * std::max<uint64_t>(2, 1 + (request.minimum_input_items + batch - 1) / batch);
    };

    if (!plan.automatic && duration == 0.0)
        {
            plan.reason = "batching explicitly disabled";
            set_buffer(plan);
            return plan;
        }
    if (!known_rate)
        {
            throw std::invalid_argument(role + ": batching requires a positive finite output sample rate and item size");
        }
    if (request.minimum_input_items >= static_cast<uint64_t>(std::numeric_limits<int>::max()))
        {
            if (!plan.automatic)
                {
                    throw std::invalid_argument(role + ": consumer input requirement exceeds supported buffering");
                }
            plan.reason = "consumer input window exceeds supported buffering";
            set_buffer(plan);
            return plan;
        }
    if (!plan.automatic)
        {
            if (!request.windows_known)
                {
                    throw std::invalid_argument(role + ": batching requires readers whose input window is known");
                }
            const auto samples = std::ceil(request.sample_rate * duration / 1000.0);
            if (!std::isfinite(samples) || samples < 1.0 || samples > std::numeric_limits<int>::max() / 4.0)
                {
                    throw std::invalid_argument(role + ".batch_size_ms exceeds the supported sample count");
                }
            plan.batch_items = static_cast<uint64_t>(samples);
            if (plan.max_buffer_items > 0)
                {
                    plan.batch_items = std::min(plan.batch_items, plan.max_buffer_items / 2);
                }
            plan.min_buffer_items = capacity(plan.batch_items);
            if (plan.min_buffer_items > addressable_items)
                {
                    throw std::invalid_argument(role + ": consumer buffer exceeds addressable memory");
                }
            if (plan.max_buffer_items > 0 && plan.min_buffer_items > plan.max_buffer_items)
                {
                    throw std::invalid_argument(role + ": max_source_buffer_samples is too small for a batch plus consumer forecast/history requirements");
                }
            set_buffer(plan);
            plan.reason = "explicit batch target";
            return plan;
        }
    if (forced_removal)
        {
            plan.reason = "honoring explicit identity-removal policy";
            set_buffer(plan);
            return plan;
        }
    if (!request.eligible || request.identity_stages == 0)
        {
            plan.reason = "no removable output copy; preserving configured topology";
            set_buffer(plan);
            return plan;
        }
    if (request.readers < 2)
        {
            plan.reason = "no reader fan-out to coalesce";
            set_buffer(plan);
            return plan;
        }

    // Publication-related reader notifications scale approximately as
    // readers * sample_rate / batch_items. Use the largest permitted target
    // without assuming a CPU-specific notification or copy cost. This is a
    // scheduling heuristic, not a prediction of optimal processing throughput.
    // The batch waits in the buffer in front of the batching block, which
    // GNU Radio sizes for two batches: bound it in bytes, independently of
    // sample representation or CPU.
    uint64_t ceiling = STREAM_BATCHER_AUTO_BATCH_BYTES / request.item_size;
    const double samples = std::floor(request.sample_rate * plan.max_latency_ms / 1000.0);
    ceiling = static_cast<uint64_t>(std::min(static_cast<double>(ceiling), samples));
    if (plan.max_buffer_items > 0)
        {
            ceiling = std::min(ceiling, plan.max_buffer_items / 2);
        }
    if (ceiling < 2)
        {
            plan.reason = "latency or memory budget cannot coalesce multiple samples";
            set_buffer(plan);
            return plan;
        }
    // Consumer progress does not depend on the batch size. In particular a
    // long integration or sample-counter window can exceed two batches.
    // Reserve that window plus a writable batch, rather than rejecting high
    // sample rates.
    const uint64_t buffer_items_required = capacity(ceiling);
    if (buffer_items_required > addressable_items ||
        (plan.max_buffer_items > 0 && buffer_items_required > plan.max_buffer_items))
        {
            plan.reason = "consumer input window exceeds the available buffer budget";
            set_buffer(plan);
            return plan;
        }
    plan.batch_items = ceiling;
    plan.min_buffer_items = buffer_items_required;
    plan.remove_identity = true;
    set_buffer(plan);
    plan.reason = "portable fan-out coalescing within latency and memory limits";
    return plan;
}
