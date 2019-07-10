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

// Physical constants
const double BEIDOU_C_M_S = 299792458.0;             //!< The speed of light, [m/s]
const double BEIDOU_C_M_MS = 299792.4580;            //!< The speed of light, [m/ms]
const double BEIDOU_PI = 3.1415926535898;            //!< Pi
const double BEIDOU_TWO_PI = 6.283185307179586;      //!< 2Pi
const double BEIDOU_OMEGA_EARTH_DOT = 7.2921150e-5;  //!< Earth rotation rate, [rad/s] as defined in CGCS2000
const double BEIDOU_GM = 3.986004418e14;             //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2] as defined in CGCS2000
const double BEIDOU_F = -4.442807309e-10;            //!< Constant, [s/(m)^(1/2)] F=-2(GM)^.5/C^2


// carrier and code frequencies
const double BEIDOU_B1I_FREQ_HZ = 1.561098e9;        //!< B1I [Hz]
const double BEIDOU_B1I_CODE_RATE_HZ = 2.046e6;      //!< Beidou B1I code rate [chips/s]
const double BEIDOU_B1I_CODE_LENGTH_CHIPS = 2046.0;  //!< Beidou B1I code length [chips]
const double BEIDOU_B1I_CODE_PERIOD = 0.001;         //!< Beidou B1I code period [seconds]
const uint32_t BEIDOU_B1I_CODE_PERIOD_MS = 1;        //!< Beidou B1I code period [ms]
const double BEIDOU_B1I_CHIP_PERIOD = 4.8875e-07;    //!< Beidou B1I chip period [seconds]
const int32_t BEIDOU_B1I_SECONDARY_CODE_LENGTH = 20;
const std::string BEIDOU_B1I_SECONDARY_CODE = "00000100110101001110";
const std::string BEIDOU_B1I_SECONDARY_CODE_STR = "00000100110101001110";
const std::string BEIDOU_B1I_GEO_PREAMBLE_SYMBOLS_STR = {"1111110000001100001100"};
const int32_t BEIDOU_B1I_GEO_PREAMBLE_LENGTH_SYMBOLS = 22;

const std::string BEIDOU_B1I_D2_SECONDARY_CODE_STR = "00";
const int BEIDOU_B1I_PREAMBLE_LENGTH_BITS = 11;
const int BEIDOU_B1I_PREAMBLE_LENGTH_SYMBOLS = 220;  // **************
const double BEIDOU_B1I_PREAMBLE_DURATION_S = 0.220;
const int BEIDOU_B1I_PREAMBLE_DURATION_MS = 220;
const int BEIDOU_B1I_TELEMETRY_RATE_BITS_SECOND = 50;  //!< D1 NAV message bit rate [bits/s]
const int BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT = 20;
const int BEIDOU_B1I_GEO_TELEMETRY_SYMBOLS_PER_BIT = 2;
const int BEIDOU_B1I_TELEMETRY_SYMBOL_PERIOD_MS = BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT * BEIDOU_B1I_CODE_PERIOD_MS;
const int BEIDOU_B1I_TELEMETRY_RATE_SYMBOLS_SECOND = BEIDOU_B1I_TELEMETRY_RATE_BITS_SECOND * BEIDOU_B1I_TELEMETRY_SYMBOLS_PER_BIT;  //************!< NAV message bit rate [symbols/s]
const int BEIDOU_WORD_LENGTH = 4;                                                                                                   //**************!< CRC + BEIDOU WORD (-2 -1 0 ... 29) Bits = 4 bytes
const int BEIDOU_SUBFRAME_LENGTH = 40;                                                                                              //**************!< BEIDOU_WORD_LENGTH x 10 = 40 bytes
const int BEIDOU_DNAV_SUBFRAME_DATA_BITS = 300;                                                                                     //!< Number of bits per subframe in the NAV message [bits]
const int BEIDOU_SUBFRAME_SECONDS = 6;                                                                                              //!< Subframe duration [seconds]
const int BEIDOU_SUBFRAME_MS = 6000;                                                                                                //!< Subframe duration [miliseconds]
const int BEIDOU_WORD_BITS = 30;                                                                                                    //!< Number of bits per word in the NAV message [bits]


#endif /* GNSS_SDR_BEIDOU_B1I_H_ */
