/*!
 * \file GPS_L1_CA.h
 * \brief  Defines system parameters for GPS L1 C/A signal and NAV data
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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


#ifndef GNSS_SDR_GPS_L1_CA_H_
#define GNSS_SDR_GPS_L1_CA_H_

#include "gnss_frequencies.h"
#include "MATH_CONSTANTS.h"
#include <cstdint>
#include <vector>
#include <utility>  // std::pair


// Physical constants
const double GPS_C_m_s = SPEED_OF_LIGHT;                 //!< The speed of light, [m/s]
const double GPS_C_m_ms = 299792.4580;                   //!< The speed of light, [m/ms]
const double GPS_PI = 3.1415926535898;                   //!< Pi as defined in IS-GPS-200E
const double GPS_TWO_PI = 6.283185307179586;             //!< 2Pi as defined in IS-GPS-200E
const double OMEGA_EARTH_DOT = DEFAULT_OMEGA_EARTH_DOT;  //!< Earth rotation rate, [rad/s]
const double GM = 3.986005e14;                           //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2]
const double F = -4.442807633e-10;                       //!< Constant, [s/(m)^(1/2)]


// carrier and code frequencies
const double GPS_L1_FREQ_HZ = FREQ1;                //!< L1 [Hz]
const double GPS_L1_CA_CODE_RATE_HZ = 1.023e6;      //!< GPS L1 C/A code rate [chips/s]
const double GPS_L1_CA_CODE_LENGTH_CHIPS = 1023.0;  //!< GPS L1 C/A code length [chips]
const double GPS_L1_CA_CODE_PERIOD = 0.001;         //!< GPS L1 C/A code period [seconds]
const uint32_t GPS_L1_CA_CODE_PERIOD_MS = 1U;       //!< GPS L1 C/A code period [ms]
const double GPS_L1_CA_CHIP_PERIOD = 9.7752e-07;    //!< GPS L1 C/A chip period [seconds]

/*!
 * \brief Maximum Time-Of-Arrival (TOA) difference between satellites for a receiver operated on Earth surface is 20 ms
 *
 * According to the GPS orbit model described in [1] Pag. 32.
 * It should be taken into account to set the buffer size for the PRN start timestamp in the pseudoranges block.
 * [1] J. Bao-Yen Tsui, Fundamentals of Global Positioning System Receivers. A Software Approach, John Wiley & Sons,
 * Inc., Hoboken, NJ, 2nd edition, 2005.
 */
const double MAX_TOA_DELAY_MS = 20;

//#define NAVIGATION_SOLUTION_RATE_MS 1000 // this cannot go here
//const double GPS_STARTOFFSET_ms = 68.802;  //[ms] Initial sign. travel time (this cannot go here)
const double GPS_STARTOFFSET_ms = 60.0;

// OBSERVABLE HISTORY DEEP FOR INTERPOLATION
const int32_t GPS_L1_CA_HISTORY_DEEP = 100;

// NAVIGATION MESSAGE DEMODULATION AND DECODING

#define GPS_PREAMBLE           \
    {                          \
        1, 0, 0, 0, 1, 0, 1, 1 \
    }
const int32_t GPS_CA_PREAMBLE_LENGTH_BITS = 8;
const int32_t GPS_CA_PREAMBLE_LENGTH_SYMBOLS = 160;
const double GPS_CA_PREAMBLE_DURATION_S = 0.160;
const int32_t GPS_CA_PREAMBLE_DURATION_MS = 160;
const int32_t GPS_CA_TELEMETRY_RATE_BITS_SECOND = 50;  //!< NAV message bit rate [bits/s]
const int32_t GPS_CA_TELEMETRY_SYMBOLS_PER_BIT = 20;
const int32_t GPS_CA_TELEMETRY_RATE_SYMBOLS_SECOND = GPS_CA_TELEMETRY_RATE_BITS_SECOND * GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;  //!< NAV message bit rate [symbols/s]
const int32_t GPS_WORD_LENGTH = 4;                                                                                          //!< CRC + GPS WORD (-2 -1 0 ... 29) Bits = 4 bytes
const int32_t GPS_SUBFRAME_LENGTH = 40;                                                                                     //!< GPS_WORD_LENGTH x 10 = 40 bytes
const int32_t GPS_SUBFRAME_BITS = 300;                                                                                      //!< Number of bits per subframe in the NAV message [bits]
const int32_t GPS_SUBFRAME_SECONDS = 6;                                                                                     //!< Subframe duration [seconds]
const int32_t GPS_SUBFRAME_MS = 6000;                                                                                       //!< Subframe duration [seconds]
const int32_t GPS_WORD_BITS = 30;                                                                                           //!< Number of bits per word in the NAV message [bits]

// GPS NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS (from IS-GPS-200E Appendix II)

// SUBFRAME 1-5 (TLM and HOW)

const std::vector<std::pair<int32_t, int32_t>> TOW({{31, 17}});
const std::vector<std::pair<int32_t, int32_t>> INTEGRITY_STATUS_FLAG({{23, 1}});
const std::vector<std::pair<int32_t, int32_t>> ALERT_FLAG({{48, 1}});
const std::vector<std::pair<int32_t, int32_t>> ANTI_SPOOFING_FLAG({{49, 1}});
const std::vector<std::pair<int32_t, int32_t>> SUBFRAME_ID({{50, 3}});

// SUBFRAME 1
const std::vector<std::pair<int32_t, int32_t>> GPS_WEEK({{61, 10}});
const std::vector<std::pair<int32_t, int32_t>> CA_OR_P_ON_L2({{71, 2}});  //*
const std::vector<std::pair<int32_t, int32_t>> SV_ACCURACY({{73, 4}});
const std::vector<std::pair<int32_t, int32_t>> SV_HEALTH({{77, 6}});
const std::vector<std::pair<int32_t, int32_t>> L2_P_DATA_FLAG({{91, 1}});
const std::vector<std::pair<int32_t, int32_t>> T_GD({{197, 8}});
const double T_GD_LSB = TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> IODC({{83, 2}, {211, 8}});
const std::vector<std::pair<int32_t, int32_t>> T_OC({{219, 16}});
const double T_OC_LSB = TWO_P4;
const std::vector<std::pair<int32_t, int32_t>> A_F2({{241, 8}});
const double A_F2_LSB = TWO_N55;
const std::vector<std::pair<int32_t, int32_t>> A_F1({{249, 16}});
const double A_F1_LSB = TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> A_F0({{271, 22}});
const double A_F0_LSB = TWO_N31;

// SUBFRAME 2
const std::vector<std::pair<int32_t, int32_t>> IODE_SF2({{61, 8}});
const std::vector<std::pair<int32_t, int32_t>> C_RS({{69, 16}});
const double C_RS_LSB = TWO_N5;
const std::vector<std::pair<int32_t, int32_t>> DELTA_N({{91, 16}});
const double DELTA_N_LSB = PI_TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> M_0({{107, 8}, {121, 24}});
const double M_0_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> C_UC({{151, 16}});
const double C_UC_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> E({{167, 8}, {181, 24}});
const double E_LSB = TWO_N33;
const std::vector<std::pair<int32_t, int32_t>> C_US({{211, 16}});
const double C_US_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> SQRT_A({{227, 8}, {241, 24}});
const double SQRT_A_LSB = TWO_N19;
const std::vector<std::pair<int32_t, int32_t>> T_OE({{271, 16}});
const double T_OE_LSB = TWO_P4;
const std::vector<std::pair<int32_t, int32_t>> FIT_INTERVAL_FLAG({{271, 1}});
const std::vector<std::pair<int32_t, int32_t>> AODO({{272, 5}});
const int32_t AODO_LSB = 900;

// SUBFRAME 3
const std::vector<std::pair<int32_t, int32_t>> C_IC({{61, 16}});
const double C_IC_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> OMEGA_0({{77, 8}, {91, 24}});
const double OMEGA_0_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> C_IS({{121, 16}});
const double C_IS_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> I_0({{137, 8}, {151, 24}});
const double I_0_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> C_RC({{181, 16}});
const double C_RC_LSB = TWO_N5;
const std::vector<std::pair<int32_t, int32_t>> OMEGA({{197, 8}, {211, 24}});
const double OMEGA_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> OMEGA_DOT({{241, 24}});
const double OMEGA_DOT_LSB = PI_TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> IODE_SF3({{271, 8}});
const std::vector<std::pair<int32_t, int32_t>> I_DOT({{279, 14}});
const double I_DOT_LSB = PI_TWO_N43;


// SUBFRAME 4-5
const std::vector<std::pair<int32_t, int32_t>> SV_DATA_ID({{61, 2}});
const std::vector<std::pair<int32_t, int32_t>> SV_PAGE({{63, 6}});

// SUBFRAME 4
//! \todo read all pages of subframe 4
// Page 18 - Ionospheric and UTC data
const std::vector<std::pair<int32_t, int32_t>> ALPHA_0({{69, 8}});
const double ALPHA_0_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t>> ALPHA_1({{77, 8}});
const double ALPHA_1_LSB = TWO_N27;
const std::vector<std::pair<int32_t, int32_t>> ALPHA_2({{91, 8}});
const double ALPHA_2_LSB = TWO_N24;
const std::vector<std::pair<int32_t, int32_t>> ALPHA_3({{99, 8}});
const double ALPHA_3_LSB = TWO_N24;
const std::vector<std::pair<int32_t, int32_t>> BETA_0({{107, 8}});
const double BETA_0_LSB = TWO_P11;
const std::vector<std::pair<int32_t, int32_t>> BETA_1({{121, 8}});
const double BETA_1_LSB = TWO_P14;
const std::vector<std::pair<int32_t, int32_t>> BETA_2({{129, 8}});
const double BETA_2_LSB = TWO_P16;
const std::vector<std::pair<int32_t, int32_t>> BETA_3({{137, 8}});
const double BETA_3_LSB = TWO_P16;
const std::vector<std::pair<int32_t, int32_t>> A_1({{151, 24}});
const double A_1_LSB = TWO_N50;
const std::vector<std::pair<int32_t, int32_t>> A_0({{181, 24}, {211, 8}});
const double A_0_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t>> T_OT({{219, 8}});
const double T_OT_LSB = TWO_P12;
const std::vector<std::pair<int32_t, int32_t>> WN_T({{227, 8}});
const double WN_T_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> DELTAT_LS({{241, 8}});
const double DELTAT_LS_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> WN_LSF({{249, 8}});
const double WN_LSF_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> DN({{257, 8}});
const double DN_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> DELTAT_LSF({{271, 8}});
const double DELTAT_LSF_LSB = 1;

// Page 25 - Antispoofing, SV config and SV health (PRN 25 -32)
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV25({{229, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV26({{241, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV27({{247, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV28({{253, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV29({{259, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV30({{271, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV31({{277, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV32({{283, 6}});


// SUBFRAME 5
//! \todo read all pages of subframe 5

// page 25 - Health (PRN 1 - 24)
const std::vector<std::pair<int32_t, int32_t>> T_OA({{69, 8}});
const double T_OA_LSB = TWO_P12;
const std::vector<std::pair<int32_t, int32_t>> WN_A({{77, 8}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV1({{91, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV2({{97, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV3({{103, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV4({{109, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV5({{121, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV6({{127, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV7({{133, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV8({{139, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV9({{151, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV10({{157, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV11({{163, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV12({{169, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV13({{181, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV14({{187, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV15({{193, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV16({{199, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV17({{211, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV18({{217, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV19({{223, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV20({{229, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV21({{241, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV22({{247, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV23({{253, 6}});
const std::vector<std::pair<int32_t, int32_t>> HEALTH_SV24({{259, 6}});

#endif /* GNSS_SDR_GPS_L1_CA_H_ */
