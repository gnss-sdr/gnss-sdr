/*!
 * \file receiver_type.h
 * \brief Helper function to get the receiver type
 * \author Mathieu Favreau, 2025. favreau.mathieu(at)hotmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RECEIVER_TYPE_H
#define GNSS_SDR_RECEIVER_TYPE_H

#include <cstdint>

class ConfigurationInterface;

enum signal_flag : uint32_t
{
    GPS_1C = 0x1 << 0,
    GPS_2S = 0x1 << 1,
    GPS_L5 = 0x1 << 2,
    GAL_1B = 0x1 << 3,
    GAL_E5a = 0x1 << 4,
    GAL_E5b = 0x1 << 5,
    GAL_E6 = 0x1 << 6,
    GLO_1G = 0x1 << 7,
    GLO_2G = 0x1 << 8,
    BDS_B1 = 0x1 << 9,
    BDS_B3 = 0x1 << 10
};

class Signal_Enabled_Flags
{
public:
    explicit Signal_Enabled_Flags(const ConfigurationInterface* configuration);
    explicit Signal_Enabled_Flags(uint32_t flags_);

    template <typename T>
    uint32_t or_all(const T& value) const
    {
        return value;
    }

    template <typename T, typename... Args>
    uint32_t or_all(const T& first, const Args&... rest) const
    {
        return first | or_all(rest...);
    }

    template <typename... Args>
    bool check_only_enabled(const Args&... args) const
    {
        return (flags ^ or_all(args...)) == 0;
    }

    template <typename... Args>
    bool check_any_enabled(const Args&... args) const
    {
        return (flags & or_all(args...)) > 0;
    }

    const uint32_t flags;
};

// Infer the type of receiver
/*
 *   TYPE  |  RECEIVER
 *     0   |  Unknown
 *     1   |  GPS L1 C/A
 *     2   |  GPS L2C
 *     3   |  GPS L5
 *     4   |  Galileo E1B
 *     5   |  Galileo E5a
 *     6   |  Galileo E5b
 *     7   |  GPS L1 C/A + GPS L2C
 *     8   |  GPS L1 C/A + GPS L5
 *     9   |  GPS L1 C/A + Galileo E1B
 *    10   |  GPS L1 C/A + Galileo E5a
 *    11   |  GPS L1 C/A + Galileo E5b
 *    12   |  Galileo E1B + GPS L2C
 *    13   |  Galileo E5a + GPS L5
 *    14   |  Galileo E1B + Galileo E5a
 *    15   |  Galileo E1B + Galileo E5b
 *    16   |  GPS L2C + GPS L5
 *    17   |  GPS L2C + Galileo E5a
 *    18   |  GPS L2C + Galileo E5b
 *    19   |  Galileo E5a + Galileo E5b
 *    20   |  GPS L5 + Galileo E5b
 *    21   |  GPS L1 C/A + Galileo E1B + GPS L2C
 *    22   |  GPS L1 C/A + Galileo E1B + GPS L5
 *    23   |  GLONASS L1 C/A
 *    24   |  GLONASS L2 C/A
 *    25   |  GLONASS L1 C/A + GLONASS L2 C/A
 *    26   |  GPS L1 C/A + GLONASS L1 C/A
 *    27   |  Galileo E1B + GLONASS L1 C/A
 *    28   |  GPS L2C + GLONASS L1 C/A
 *    29   |  GPS L1 C/A + GLONASS L2 C/A
 *    30   |  Galileo E1B + GLONASS L2 C/A
 *    31   |  GPS L2C + GLONASS L2 C/A
 *    32   |  GPS L1 C/A + Galileo E1B + GPS L5 + Galileo E5a
 *    33   |  GPS L1 C/A + Galileo E1B + Galileo E5a
 *
 *    Skipped previous values to avoid overlapping
 *    100   |  Galileo E6B
 *    101   |  Galileo E1B + Galileo E6B
 *    102   |  Galileo E5a + Galileo E6B
 *    103   |  Galileo E5b + Galileo E6B
 *    104   |  Galileo E1B + Galileo E5a + Galileo E6B
 *    105   |  Galileo E1B + Galileo E5b + Galileo E6B
 *    106   |  GPS L1 C/A + Galileo E1B + Galileo E6B
 *    107   |  GPS L1 C/A + Galileo E6B
 *    108   |  GPS L1 C/A + Galileo E1B + GPS L5 + Galileo E5a + Galileo E6B
 *    Skipped previous values to avoid overlapping
 *    500   |  BeiDou B1I
 *    501   |  BeiDou B1I + GPS L1 C/A
 *    502   |  BeiDou B1I + Galileo E1B
 *    503   |  BeiDou B1I + GLONASS L1 C/A
 *    504   |  BeiDou B1I + GPS L1 C/A + Galileo E1B
 *    505   |  BeiDou B1I + GPS L1 C/A + GLONASS L1 C/A + Galileo E1B
 *    506   |  BeiDou B1I + Beidou B3I
 *    Skipped previous values to avoid overlapping
 *    600   |  BeiDou B3I
 *    601   |  BeiDou B3I + GPS L2C
 *    602   |  BeiDou B3I + GLONASS L2 C/A
 *    603   |  BeiDou B3I + GPS L2C + GLONASS L2 C/A
 *
 *    1000  |  GPS L1 C/A + GPS L2C + GPS L5
 *    1001  |  GPS L1 C/A + Galileo E1B + GPS L2C + GPS L5 + Galileo E5a
 */

uint32_t get_type_of_receiver(const Signal_Enabled_Flags& signal_enabled_flags);

#endif  // GNSS_SDR_RECEIVER_TYPE_H
