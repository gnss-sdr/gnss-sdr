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
    if (!plan.automatic && duration == 0.0)
        {
            plan.reason = "batching explicitly disabled";
            return plan;
        }
    if (!std::isfinite(request.sample_rate) || request.sample_rate <= 0.0 || request.item_size == 0)
        {
            throw std::invalid_argument(role + ": batching requires a positive finite output sample rate and item size");
        }
    if (!plan.automatic)
        {
            const auto samples = std::ceil(request.sample_rate * duration / 1000.0);
            if (!std::isfinite(samples) || samples < 1.0 || samples > std::numeric_limits<int>::max() / 2.0)
                {
                    throw std::invalid_argument(role + ".batch_size_ms exceeds the supported sample count");
                }
            plan.batch_items = static_cast<uint64_t>(samples);
            if (plan.max_buffer_items > 0)
                {
                    plan.batch_items = std::min(plan.batch_items, plan.max_buffer_items / 2);
                }
            if (request.minimum_input_items >= static_cast<uint64_t>(std::numeric_limits<int>::max()))
                {
                    throw std::invalid_argument(role + ": consumer input requirement exceeds supported buffering");
                }
            plan.min_buffer_items = std::max(2 * plan.batch_items, request.minimum_input_items + 1);
            if (plan.min_buffer_items > std::numeric_limits<size_t>::max() / request.item_size)
                {
                    throw std::invalid_argument(role + ": consumer buffer exceeds addressable memory");
                }
            if (plan.max_buffer_items > 0 && plan.min_buffer_items > plan.max_buffer_items)
                {
                    throw std::invalid_argument(role + ": max_source_buffer_samples is too small for consumer forecast/history requirements");
                }
            plan.reason = "explicit batch target";
            return plan;
        }
    if (forced_removal)
        {
            plan.reason = "honoring explicit identity-removal policy";
            return plan;
        }
    if (!request.eligible || request.identity_stages == 0)
        {
            plan.reason = "no removable output copy; preserving configured topology";
            return plan;
        }
    if (request.readers < 2)
        {
            plan.reason = "no reader fan-out to coalesce";
            return plan;
        }

    // Publication-related reader notifications scale approximately as
    // readers * sample_rate / batch_items. Use the largest permitted target
    // without assuming a CPU-specific notification or copy cost. This is a
    // scheduling heuristic, not a prediction of optimal processing throughput.
    // Bound private sample staging to 512 KiB per conditioned stream. This is
    // a portable memory policy, independent of sample representation or CPU.
    constexpr uint64_t staging_bytes = 512U * 1024U;
    uint64_t ceiling = staging_bytes / request.item_size;
    const double samples = std::floor(request.sample_rate * plan.max_latency_ms / 1000.0);
    ceiling = static_cast<uint64_t>(std::min(static_cast<double>(ceiling), samples));
    if (plan.max_buffer_items > 0)
        {
            ceiling = std::min(ceiling, plan.max_buffer_items / 2);
        }
    if (ceiling < 2)
        {
            plan.reason = "latency or memory budget cannot coalesce multiple samples";
            return plan;
        }
    // Consumer progress is independent of staging size. In particular a long
    // integration or sample-counter window can exceed two batches. Reserve
    // that window plus writable space, rather than rejecting high sample rates.
    if (request.minimum_input_items >= static_cast<uint64_t>(std::numeric_limits<int>::max()))
        {
            plan.reason = "consumer input window exceeds supported buffering";
            return plan;
        }
    const uint64_t buffer_items_required = std::max(2 * ceiling, request.minimum_input_items + 1);
    if (buffer_items_required > std::numeric_limits<size_t>::max() / request.item_size ||
        (plan.max_buffer_items > 0 && buffer_items_required > plan.max_buffer_items))
        {
            plan.reason = "consumer input window exceeds the available buffer budget";
            return plan;
        }
    plan.batch_items = ceiling;
    plan.min_buffer_items = buffer_items_required;
    plan.remove_identity = true;
    plan.reason = "portable fan-out coalescing within latency and memory limits";
    return plan;
}
