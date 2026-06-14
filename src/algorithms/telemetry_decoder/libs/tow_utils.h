/*!
 * \file tow_utils.h
 * \brief Small helpers for TOW arithmetic shared by telemetry decoder blocks
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

#ifndef GNSS_SDR_TOW_UTILS_H
#define GNSS_SDR_TOW_UTILS_H

#include <cmath>
#include <cstdint>
#include <limits>

/*!
 * Helpers for arithmetic on the GPS time of week reported by the telemetry
 * decoders, which spans [0, 604800) seconds and rolls over at the week
 * boundary.
 */
namespace gnss_tow
{
constexpr uint32_t WEEK_MS = 604800000U;
constexpr double WEEK_S = 604800.0;


inline uint32_t wrap_ms(int64_t tow_ms)
{
    tow_ms %= static_cast<int64_t>(WEEK_MS);
    if (tow_ms < 0)
        {
            tow_ms += static_cast<int64_t>(WEEK_MS);
        }
    return static_cast<uint32_t>(tow_ms);
}


inline double wrap_s(double tow_s)
{
    double wrapped_tow_s = std::fmod(tow_s, WEEK_S);
    if (wrapped_tow_s < 0.0)
        {
            wrapped_tow_s += WEEK_S;
        }
    return wrapped_tow_s;
}


inline uint32_t wrap_s_to_ms(double tow_s)
{
    return static_cast<uint32_t>(std::round(wrap_s(tow_s) * 1000.0)) % WEEK_MS;
}


inline int64_t to_ms(double tow_s)
{
    return static_cast<int64_t>(tow_s * 1000.0);
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


/*!
 * Distance in ms between two TOW values along the shortest direction of the
 * week circle.
 */
inline uint32_t circular_error_ms(uint32_t lhs_ms, uint32_t rhs_ms)
{
    const uint32_t forward_error_ms = lhs_ms >= rhs_ms ? lhs_ms - rhs_ms : lhs_ms + WEEK_MS - rhs_ms;
    const uint32_t reverse_error_ms = rhs_ms >= lhs_ms ? rhs_ms - lhs_ms : rhs_ms + WEEK_MS - lhs_ms;
    return forward_error_ms < reverse_error_ms ? forward_error_ms : reverse_error_ms;
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

}  // namespace gnss_tow

#endif  // GNSS_SDR_TOW_UTILS_H
