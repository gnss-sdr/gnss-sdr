/*!
 * \file Beidou_B1I.h
 * \brief  Defines system parameters for BeiDou B1I signal and DNAV data
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 * \author Damian Miralles, 2018. dmiralles2009@gmail.com
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_BEIDOU_B1I_H_
#define GNSS_SDR_BEIDOU_B1I_H_

#include "MATH_CONSTANTS.h"
#include <cstdint>
#include <string>

// carrier and code frequencies
const double BEIDOU_B1I_FREQ_HZ = 1.561098e9;        //!< B1I [Hz]
const double BEIDOU_B1I_CODE_RATE_HZ = 2.046e6;      //!< Beidou B1I code rate [chips/s]
const double BEIDOU_B1I_CODE_LENGTH_CHIPS = 2046.0;  //!< Beidou B1I code length [chips]
const double BEIDOU_B1I_CODE_PERIOD = 0.001;         //!< Beidou B1I code period [seconds]
const uint32_t BEIDOU_B1I_CODE_PERIOD_MS = 1;        //!< Beidou B1I code period [ms]
const int32_t BEIDOU_B1I_SECONDARY_CODE_LENGTH = 20;
const std::string BEIDOU_B1I_SECONDARY_CODE_STR = "00000100110101001110";
const std::string BEIDOU_B1I_GEO_PREAMBLE_SYMBOLS_STR = {"1111110000001100001100"};
const int32_t BEIDOU_B1I_GEO_PREAMBLE_LENGTH_SYMBOLS = 22;
const std::string BEIDOU_B1I_D2_SECONDARY_CODE_STR = "00";
const uint32_t BEIDOU_B1I_PREAMBLE_LENGTH_BITS = 11;
const uint32_t BEIDOU_B1I_PREAMBLE_LENGTH_SYMBOLS = 220;
const double BEIDOU_B1I_PREAMBLE_DURATION_S = 0.220;
const int32_t BEIDOU_B1I_PREAMBLE_DURATION_MS = 220;
const int32_t BEIDOU_B1I_TELEMETRY_RATE_BITS_SECOND = 50;
const int32_t BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT = 20;
const int32_t BEIDOU_B1I_GEO_TELEMETRY_SYMBOLS_PER_BIT = 2;
const int32_t BEIDOU_B1I_TELEMETRY_SYMBOL_PERIOD_MS = BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT * BEIDOU_B1I_CODE_PERIOD_MS;
const int32_t BEIDOU_B1I_TELEMETRY_RATE_SYMBOLS_SECOND = BEIDOU_B1I_TELEMETRY_RATE_BITS_SECOND * BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT;


#endif /* GNSS_SDR_BEIDOU_B1I_H_ */
