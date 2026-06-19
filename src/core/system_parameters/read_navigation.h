/*!
 * \file read_navigation.h
 * \brief  Helper functions to read fields from a navigation message from std::bitset
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 * \author Carles Fernandez Prades, 2015-2026. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_READ_NAVIGATION_H
#define GNSS_SDR_READ_NAVIGATION_H

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

template <size_t SIZE>
uint64_t read_navigation_unsigned(const std::bitset<SIZE>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    uint64_t value = 0ULL;
    for (const auto& p : parameter)
        {
            for (int32_t j = 0; j < p.second; j++)
                {
                    value = (value << 1) | static_cast<uint64_t>(bits[SIZE - p.first - j]);
                }
        }
    return value;
}

template <size_t SIZE>
int64_t read_navigation_signed(const std::bitset<SIZE>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    int64_t value = (bits[SIZE - parameter[0].first] == 1) ? -1LL : 0LL;
    for (const auto& p : parameter)
        {
            for (int32_t j = 0; j < p.second; j++)
                {
                    value = (value << 1) | static_cast<int64_t>(bits[SIZE - p.first - j]);
                }
        }
    return value;
}

template <size_t SIZE>
bool read_navigation_bool(const std::bitset<SIZE>& bits, const std::vector<std::pair<int32_t, int32_t>>& parameter)
{
    bool value = bits[SIZE - parameter[0].first];
    return value;
}


/** \} */
/** \} */
#endif  // GNSS_SDR_READ_NAVIGATION_H
