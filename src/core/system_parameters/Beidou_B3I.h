/*!
 * \file Beidou_B3I.h
 * \brief  Defines system parameters for BeiDou B3I signal and DNAV data
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_BEIDOU_B3I_H
#define GNSS_SDR_BEIDOU_B3I_H

#include "MATH_CONSTANTS.h"
#include <cstdint>
#include <string>

// carrier and code frequencies
const double BEIDOU_B3I_FREQ_HZ = 1.268520e9;         //!< BeiDou B3I [Hz]
const double BEIDOU_B3I_CODE_RATE_CPS = 10.23e6;      //!< BeiDou B3I code rate [chips/s]
const double BEIDOU_B3I_CODE_LENGTH_CHIPS = 10230.0;  //!< BeiDou B3I code length [chips]
const double BEIDOU_B3I_CODE_PERIOD_S = 0.001;        //!< BeiDou B3I code period [seconds]
constexpr uint32_t BEIDOU_B3I_CODE_PERIOD_MS = 1;     //!< BeiDou B3I code period [ms]
const int32_t BEIDOU_B3I_SECONDARY_CODE_LENGTH = 20;
const std::string BEIDOU_B3I_SECONDARY_CODE_STR = "00000100110101001110";
const std::string BEIDOU_B3I_GEO_PREAMBLE_SYMBOLS_STR = {"1111110000001100001100"};
const int32_t BEIDOU_B3I_GEO_PREAMBLE_LENGTH_SYMBOLS = 22;
const std::string BEIDOU_B3I_D2_SECONDARY_CODE_STR = "00";
const uint32_t BEIDOU_B3I_PREAMBLE_LENGTH_BITS = 11;
const uint32_t BEIDOU_B3I_PREAMBLE_LENGTH_SYMBOLS = 220;  // **************
const double BEIDOU_B3I_PREAMBLE_DURATION_S = 0.220;
const int32_t BEIDOU_B3I_PREAMBLE_DURATION_MS = 220;
constexpr int32_t BEIDOU_B3I_TELEMETRY_RATE_BITS_SECOND = 50;  //!< D1 NAV message bit rate [bits/s]
constexpr int32_t BEIDOU_B3I_TELEMETRY_SYMBOLS_PER_BIT = 20;
const int32_t BEIDOU_B3I_GEO_TELEMETRY_SYMBOLS_PER_BIT = 2;  // *************
constexpr int32_t BEIDOU_B3I_TELEMETRY_SYMBOL_PERIOD_MS = BEIDOU_B3I_TELEMETRY_SYMBOLS_PER_BIT * BEIDOU_B3I_CODE_PERIOD_MS;
constexpr int32_t BEIDOU_B3I_TELEMETRY_RATE_SYMBOLS_SECOND = BEIDOU_B3I_TELEMETRY_RATE_BITS_SECOND * BEIDOU_B3I_TELEMETRY_SYMBOLS_PER_BIT;


#endif  // GNSS_SDR_BEIDOU_B3I_H
