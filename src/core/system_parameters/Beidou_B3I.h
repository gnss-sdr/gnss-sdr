/*!
 * \file Beidou_B3I.h
 * \brief  Defines system parameters for BeiDou B3I signal and DNAV data
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
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


#ifndef GNSS_SDR_BEIDOU_B3I_H
#define GNSS_SDR_BEIDOU_B3I_H

#include "gnss_frequencies.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


// carrier and code frequencies
constexpr double BEIDOU_B3I_FREQ_HZ = FREQ3_BDS;          //!< BeiDou B3I [Hz]
constexpr double BEIDOU_B3I_CODE_RATE_CPS = 10.23e6;      //!< BeiDou B3I code rate [chips/s]
constexpr double BEIDOU_B3I_CODE_LENGTH_CHIPS = 10230.0;  //!< BeiDou B3I code length [chips]
constexpr double BEIDOU_B3I_CODE_PERIOD_S = 0.001;        //!< BeiDou B3I code period [seconds]
constexpr double BEIDOU_B3I_PREAMBLE_DURATION_S = 0.220;
constexpr uint32_t BEIDOU_B3I_CODE_PERIOD_MS = 1;  //!< BeiDou B3I code period [ms]
constexpr uint32_t BEIDOU_B3I_PREAMBLE_LENGTH_BITS = 11;
constexpr uint32_t BEIDOU_B3I_PREAMBLE_LENGTH_SYMBOLS = 220;  // **************
constexpr int32_t BEIDOU_B3I_SECONDARY_CODE_LENGTH = 20;
constexpr int32_t BEIDOU_B3I_GEO_PREAMBLE_LENGTH_SYMBOLS = 22;
constexpr int32_t BEIDOU_B3I_PREAMBLE_DURATION_MS = 220;
constexpr int32_t BEIDOU_B3I_TELEMETRY_RATE_BITS_SECOND = 50;  //!< D1 NAV message bit rate [bits/s]
constexpr int32_t BEIDOU_B3I_TELEMETRY_SYMBOLS_PER_BIT = 20;
constexpr int32_t BEIDOU_B3I_GEO_TELEMETRY_SYMBOLS_PER_BIT = 2;  // *************
constexpr int32_t BEIDOU_B3I_TELEMETRY_SYMBOL_PERIOD_MS = static_cast<int32_t>(static_cast<uint32_t>(BEIDOU_B3I_TELEMETRY_SYMBOLS_PER_BIT) * BEIDOU_B3I_CODE_PERIOD_MS);
constexpr int32_t BEIDOU_B3I_TELEMETRY_RATE_SYMBOLS_SECOND = BEIDOU_B3I_TELEMETRY_RATE_BITS_SECOND * BEIDOU_B3I_TELEMETRY_SYMBOLS_PER_BIT;
constexpr char BEIDOU_B3I_SECONDARY_CODE_STR[21] = "00000100110101001110";
constexpr char BEIDOU_B3I_GEO_PREAMBLE_SYMBOLS_STR[23] = "1111110000001100001100";
constexpr char BEIDOU_B3I_D2_SECONDARY_CODE_STR[3] = "00";


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B3I_H
