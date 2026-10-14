/*!
 * \file conditioner_scheduling_policy.h
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

#ifndef GNSS_SDR_CONDITIONER_SCHEDULING_POLICY_H
#define GNSS_SDR_CONDITIONER_SCHEDULING_POLICY_H

#include "stream_batcher_limits.h"
#include <cstddef>
#include <cstdint>
#include <string>

/** \addtogroup Core Core GNSS Receiver
 * Core GNSS Receiver.
 * \{ */
/** \addtogroup Core_Receiver
 * Classes for the core GNSS receiver.
 * \{ */

class ConfigurationInterface;

class ConditionerSchedulingPolicy
{
public:
    struct Request
    {
        double sample_rate = 0.0;
        size_t item_size = 0;
        size_t readers = 0;
        size_t identity_stages = 0;
        //! Largest forecast or history among all readers, in items.
        uint64_t minimum_input_items = 0;
        //! The same for channel readers only, without the receiver clock.
        uint64_t channel_input_items = 0;
        bool eligible = true;
        //! False if the input window of some reader could not be queried.
        bool windows_known = true;
        //! True if the receiver clock counts the samples of this stream.
        bool feeds_clock = true;
        //! False if the receiver sets the pace, as in an unthrottled replay.
        bool real_time = true;
    };
    struct Plan
    {
        uint64_t batch_items = 0;
        uint64_t max_buffer_items = 0;
        //! Output buffer to request for the conditioned stream, 0 for none.
        uint64_t min_buffer_items = 0;
        //! Largest reader window that a batch must leave room for.
        uint64_t reader_window_items = 0;
        size_t readers = 0;
        double max_latency_ms = STREAM_BATCHER_DEFAULT_LATENCY_MS;
        //! Signal time that readers may drift apart before channels miss epochs.
        double alignment_budget_ms = 0.0;
        bool remove_identity = false;
        bool automatic = true;
        //! True if the configuration asks for more depth than that budget.
        bool buffer_over_budget = false;
        std::string reason;
    };

    static Plan select(const ConfigurationInterface& configuration, const std::string& role, const Request& request);

private:
    static std::string trim(const std::string& text);
    static double number(const std::string& text, const std::string& key);
};

/** \} */
/** \} */
#endif  // GNSS_SDR_CONDITIONER_SCHEDULING_POLICY_H
