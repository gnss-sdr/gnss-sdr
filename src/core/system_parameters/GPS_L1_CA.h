/*!
 * \file GPS_L1_CA.h
 * \brief  Defines system parameters for GPS L1 C/A signal and NAV data
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_L1_CA_H
#define GNSS_SDR_GPS_L1_CA_H

#include "MATH_CONSTANTS.h"
#include "gnss_frequencies.h"
#include <cstdint>
#include <utility>  // std::pair
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


// carrier and code frequencies
constexpr double GPS_L1_FREQ_HZ = FREQ1;                //!< L1 [Hz]
constexpr double GPS_L1_CA_CODE_RATE_CPS = 1.023e6;     //!< GPS L1 C/A code rate [chips/s]
constexpr double GPS_L1_CA_CODE_LENGTH_CHIPS = 1023.0;  //!< GPS L1 C/A code length [chips]
constexpr double GPS_L1_CA_CODE_PERIOD_S = 0.001;       //!< GPS L1 C/A code period [seconds]
constexpr double GPS_L1_CA_CHIP_PERIOD_S = 9.7752e-07;  //!< GPS L1 C/A chip period [seconds]
constexpr uint32_t GPS_L1_CA_CODE_PERIOD_MS = 1U;       //!< GPS L1 C/A code period [ms]
constexpr uint32_t GPS_L1_CA_BIT_PERIOD_MS = 20U;       //!< GPS L1 C/A bit period [ms]

/*!
 * \brief Maximum Time-Of-Arrival (TOA) difference between satellites for a receiver operated on Earth surface is 20 ms
 *
 * According to the GPS orbit model described in [1] Pag. 32.
 * It should be taken into account to set the buffer size for the PRN start timestamp in the pseudoranges block.
 * [1] J. Bao-Yen Tsui, Fundamentals of Global Positioning System Receivers. A Software Approach, John Wiley & Sons,
 * Inc., Hoboken, NJ, 2nd edition, 2005.
 */
constexpr double MAX_TOA_DELAY_MS = 20.0;

// optimum parameters
constexpr uint32_t GPS_L1_CA_OPT_ACQ_FS_SPS = 2000000;  //!< Sampling frequency that maximizes the acquisition SNR while using a non-multiple of chip rate

// OBSERVABLE HISTORY DEEP FOR INTERPOLATION
constexpr int32_t GPS_L1_CA_HISTORY_DEEP = 100;

// NAVIGATION MESSAGE DEMODULATION AND DECODING
constexpr double GPS_CA_PREAMBLE_DURATION_S = 0.160;
constexpr int32_t GPS_CA_PREAMBLE_LENGTH_BITS = 8;
constexpr int32_t GPS_CA_PREAMBLE_LENGTH_SYMBOLS = 160;
constexpr int32_t GPS_CA_PREAMBLE_DURATION_MS = 160;
constexpr int32_t GPS_CA_TELEMETRY_RATE_BITS_SECOND = 50;  //!< NAV message bit rate [bits/s]
constexpr int32_t GPS_CA_TELEMETRY_SYMBOLS_PER_BIT = 20;
constexpr int32_t GPS_CA_TELEMETRY_RATE_SYMBOLS_SECOND = GPS_CA_TELEMETRY_RATE_BITS_SECOND * GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;  //!< NAV message bit rate [symbols/s]
constexpr int32_t GPS_WORD_LENGTH = 4;                                                                                          //!< CRC + GPS WORD (-2 -1 0 ... 29) Bits = 4 bytes
constexpr int32_t GPS_SUBFRAME_LENGTH = 40;                                                                                     //!< GPS_WORD_LENGTH x 10 = 40 bytes
constexpr int32_t GPS_SUBFRAME_BITS = 300;                                                                                      //!< Number of bits per subframe in the NAV message [bits]
constexpr int32_t GPS_SUBFRAME_SECONDS = 6;                                                                                     //!< Subframe duration [seconds]
constexpr int32_t GPS_SUBFRAME_MS = 6000;                                                                                       //!< Subframe duration [seconds]
constexpr int32_t GPS_WORD_BITS = 30;                                                                                           //!< Number of bits per word in the NAV message [bits]
constexpr char GPS_CA_PREAMBLE[9] = "10001011";
constexpr char GPS_CA_PREAMBLE_SYMBOLS_STR[161] = "1111111111111111111100000000000000000000000000000000000000000000000000000000000011111111111111111111000000000000000000001111111111111111111111111111111111111111";

// GPS NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS (from IS-GPS-200M Appendix II)

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
constexpr double T_GD_LSB = TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> IODC({{83, 2}, {211, 8}});
const std::vector<std::pair<int32_t, int32_t>> T_OC({{219, 16}});
constexpr int32_t T_OC_LSB = static_cast<int32_t>(TWO_P4);
const std::vector<std::pair<int32_t, int32_t>> A_F2({{241, 8}});
constexpr double A_F2_LSB = TWO_N55;
const std::vector<std::pair<int32_t, int32_t>> A_F1({{249, 16}});
constexpr double A_F1_LSB = TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> A_F0({{271, 22}});
constexpr double A_F0_LSB = TWO_N31;

// SUBFRAME 2
const std::vector<std::pair<int32_t, int32_t>> IODE_SF2({{61, 8}});
const std::vector<std::pair<int32_t, int32_t>> C_RS({{69, 16}});
constexpr double C_RS_LSB = TWO_N5;
const std::vector<std::pair<int32_t, int32_t>> DELTA_N({{91, 16}});
constexpr double DELTA_N_LSB = PI_TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> M_0({{107, 8}, {121, 24}});
constexpr double M_0_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> C_UC({{151, 16}});
constexpr double C_UC_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> ECCENTRICITY({{167, 8}, {181, 24}});
constexpr double ECCENTRICITY_LSB = TWO_N33;
const std::vector<std::pair<int32_t, int32_t>> C_US({{211, 16}});
constexpr double C_US_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> SQRT_A({{227, 8}, {241, 24}});
constexpr double SQRT_A_LSB = TWO_N19;
const std::vector<std::pair<int32_t, int32_t>> T_OE({{271, 16}});
constexpr int32_t T_OE_LSB = static_cast<int32_t>(TWO_P4);
const std::vector<std::pair<int32_t, int32_t>> FIT_INTERVAL_FLAG({{271, 1}});
const std::vector<std::pair<int32_t, int32_t>> AODO({{272, 5}});
constexpr int32_t AODO_LSB = 900;

// SUBFRAME 3
const std::vector<std::pair<int32_t, int32_t>> C_IC({{61, 16}});
constexpr double C_IC_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> OMEGA_0({{77, 8}, {91, 24}});
constexpr double OMEGA_0_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> C_IS({{121, 16}});
constexpr double C_IS_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> I_0({{137, 8}, {151, 24}});
constexpr double I_0_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> C_RC({{181, 16}});
constexpr double C_RC_LSB = TWO_N5;
const std::vector<std::pair<int32_t, int32_t>> OMEGA({{197, 8}, {211, 24}});
constexpr double OMEGA_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> OMEGA_DOT({{241, 24}});
constexpr double OMEGA_DOT_LSB = PI_TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> IODE_SF3({{271, 8}});
const std::vector<std::pair<int32_t, int32_t>> I_DOT({{279, 14}});
constexpr double I_DOT_LSB = PI_TWO_N43;

// SUBFRAME 4-5
const std::vector<std::pair<int32_t, int32_t>> SV_DATA_ID({{61, 2}});
const std::vector<std::pair<int32_t, int32_t>> SV_PAGE({{63, 6}});

// SUBFRAME 4
//! \todo read all pages of subframe 4
// Page 18 - Ionospheric and UTC data
const std::vector<std::pair<int32_t, int32_t>> ALPHA_0({{69, 8}});
constexpr double ALPHA_0_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t>> ALPHA_1({{77, 8}});
constexpr double ALPHA_1_LSB = TWO_N27;
const std::vector<std::pair<int32_t, int32_t>> ALPHA_2({{91, 8}});
constexpr double ALPHA_2_LSB = TWO_N24;
const std::vector<std::pair<int32_t, int32_t>> ALPHA_3({{99, 8}});
constexpr double ALPHA_3_LSB = TWO_N24;
const std::vector<std::pair<int32_t, int32_t>> BETA_0({{107, 8}});
constexpr double BETA_0_LSB = TWO_P11;
const std::vector<std::pair<int32_t, int32_t>> BETA_1({{121, 8}});
constexpr double BETA_1_LSB = TWO_P14;
const std::vector<std::pair<int32_t, int32_t>> BETA_2({{129, 8}});
constexpr double BETA_2_LSB = TWO_P16;
const std::vector<std::pair<int32_t, int32_t>> BETA_3({{137, 8}});
constexpr double BETA_3_LSB = TWO_P16;
const std::vector<std::pair<int32_t, int32_t>> A_1({{151, 24}});
constexpr double A_1_LSB = TWO_N50;
const std::vector<std::pair<int32_t, int32_t>> A_0({{181, 24}, {211, 8}});
constexpr double A_0_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t>> T_OT({{219, 8}});
constexpr double T_OT_LSB = TWO_P12;
const std::vector<std::pair<int32_t, int32_t>> WN_T({{227, 8}});
constexpr double WN_T_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> DELTAT_LS({{241, 8}});
constexpr double DELTAT_LS_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> WN_LSF({{249, 8}});
constexpr double WN_LSF_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> DN({{257, 8}});
constexpr double DN_LSB = 1;
const std::vector<std::pair<int32_t, int32_t>> DELTAT_LSF({{271, 8}});
constexpr double DELTAT_LSF_LSB = 1;

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
constexpr int32_t T_OA_LSB = TWO_P12;
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


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1_CA_H
