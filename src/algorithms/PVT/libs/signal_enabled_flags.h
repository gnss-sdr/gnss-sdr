/*!
 * \file signal_enabled_flags.h
 * \brief Class to check the enabled signals
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

#ifndef GNSS_SDR_SIGNAL_ENABLED_FLAGS_H
#define GNSS_SDR_SIGNAL_ENABLED_FLAGS_H

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

#endif  // GNSS_SDR_SIGNAL_ENABLED_FLAGS_H
