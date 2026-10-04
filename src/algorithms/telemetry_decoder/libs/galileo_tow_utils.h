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
#include "tow_utils.h"     // for the system-agnostic TOW arithmetic helpers
#include <cstdint>

namespace galileo_tow
{
// Generic TOW arithmetic, shared with the other telemetry decoders
using gnss_tow::add_ms;
using gnss_tow::floor_div;
using gnss_tow::floor_to_second_ms;
using gnss_tow::sample_counter_delta;
using gnss_tow::seconds_to_ms;
using gnss_tow::week_after_delta;
using gnss_tow::WEEK_MS;
using gnss_tow::wrap_ms;


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
