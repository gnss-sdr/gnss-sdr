/*!
 * \file galileo_tow_utils.h
 * \brief Small helpers for Galileo TOW arithmetic in telemetry decoder blocks
 * \author Carles Fernandez, 2026. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_GALILEO_TOW_UTILS_H
#define GNSS_SDR_GALILEO_TOW_UTILS_H

#include "Galileo_E5a.h"   // for GALILEO_E5A_CODE_PERIOD_MS, GALILEO_FNAV_CODES_PER_SYMBOL
#include "Galileo_INAV.h"  // for GALILEO_INAV_PAGE_PART_MS
#include <cstdint>
#include <limits>

namespace galileo_tow
{
constexpr uint32_t WEEK_MS = 604800000U;


inline uint32_t wrap_ms(int64_t tow_ms)
{
    tow_ms %= static_cast<int64_t>(WEEK_MS);
    if (tow_ms < 0)
        {
            tow_ms += static_cast<int64_t>(WEEK_MS);
        }
    return static_cast<uint32_t>(tow_ms);
}


inline uint32_t seconds_to_ms(int32_t tow_s)
{
    return wrap_ms(static_cast<int64_t>(tow_s) * 1000LL);
}


inline uint32_t add_ms(uint32_t tow_ms, int64_t delta_ms)
{
    return wrap_ms(static_cast<int64_t>(tow_ms) + delta_ms);
}


inline uint32_t floor_to_second_ms(uint32_t tow_ms)
{
    return (tow_ms / 1000U) * 1000U;
}


inline int64_t floor_div(int64_t numerator, int64_t denominator)
{
    const int64_t quotient = numerator / denominator;
    const int64_t remainder = numerator % denominator;
    if (remainder != 0 && ((remainder < 0) != (denominator < 0)))
        {
            return quotient - 1;
        }
    return quotient;
}


inline bool week_after_delta(uint32_t week, uint32_t tow_ms, int64_t delta_ms, uint32_t& result_week)
{
    const int64_t week_delta = floor_div(static_cast<int64_t>(tow_ms) + delta_ms, static_cast<int64_t>(WEEK_MS));
    if (week_delta < 0)
        {
            const uint64_t negative_week_delta = static_cast<uint64_t>(-(week_delta + 1LL)) + 1ULL;
            if (negative_week_delta > static_cast<uint64_t>(week))
                {
                    return false;
                }
        }
    else if (week_delta > static_cast<int64_t>(std::numeric_limits<uint32_t>::max() - week))
        {
            return false;
        }

    result_week = static_cast<uint32_t>(static_cast<int64_t>(week) + week_delta);
    return true;
}


inline int64_t sample_counter_delta(uint64_t current, uint64_t reference)
{
    const auto max_delta = static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
    if (current >= reference)
        {
            const uint64_t delta = current - reference;
            return delta > max_delta ? std::numeric_limits<int64_t>::max() : static_cast<int64_t>(delta);
        }

    const uint64_t delta = reference - current;
    return delta > max_delta ? std::numeric_limits<int64_t>::min() + 1 : -static_cast<int64_t>(delta);
}


inline int64_t inav_current_symbol_delay_ms(uint32_t required_symbols, uint32_t code_period_ms)
{
    return static_cast<int64_t>(GALILEO_INAV_PAGE_PART_MS) + (static_cast<int64_t>(required_symbols) + 1LL) * static_cast<int64_t>(code_period_ms);
}


inline int64_t fnav_current_symbol_delay_ms(uint32_t required_symbols)
{
    return (static_cast<int64_t>(required_symbols) + 1LL) * static_cast<int64_t>(GALILEO_FNAV_CODES_PER_SYMBOL) * static_cast<int64_t>(GALILEO_E5A_CODE_PERIOD_MS);
}

}  // namespace galileo_tow

#endif  // GNSS_SDR_GALILEO_TOW_UTILS_H
