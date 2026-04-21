/*!
 * \file signal_flag.h
 * \brief Enum for signals
 * \author Mathieu Favreau, 2026. favreau.mathieu(at)hotmail.com
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

#ifndef GNSS_SDR_SIGNAL_FLAG_H
#define GNSS_SDR_SIGNAL_FLAG_H

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
    BDS_B3 = 0x1 << 10,
    QZS_J1 = 0x1 << 11,
    QZS_J5 = 0x1 << 12
};

#endif  // GNSS_SDR_SIGNAL_FLAG_H
