/*!
 * \file Galileo_INAV.h
 * \brief Galileo INAV mesage constants
 * \author Carles Fernandez, 2020. cfernandez(at)cttc.es
 *
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

#ifndef GNSS_SDR_GALILEO_INAV_H
#define GNSS_SDR_GALILEO_INAV_H

#include "MATH_CONSTANTS.h"
#include <cstdint>
#include <utility>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


// Galileo INAV message structure
constexpr double GALILEO_INAV_PAGE_PART_WITH_PREABLE_SECONDS = 2.04;  //!< Page Duration + (Galileo I/NAV Preamble bits)*(Galileo E5b-I tiered Code Period(seconds))
constexpr uint32_t GALILEO_INAV_PAGE_SYMBOLS = 500;                   //!< The complete Galileo INAV page length
constexpr int32_t GALILEO_INAV_PREAMBLE_LENGTH_BITS = 10;
constexpr int32_t GALILEO_INAV_PREAMBLE_PERIOD_SYMBOLS = 250;
constexpr int32_t GALILEO_INAV_PAGE_PART_SYMBOLS = 250;  //!< Each Galileo INAV pages are composed of two parts (even and odd) each of 250 symbols, including preamble. See Galileo ICD 4.3.2
constexpr int32_t GALILEO_INAV_PAGE_PART_SECONDS = 1;    // a page part last 1 sec
constexpr int32_t GALILEO_INAV_PAGE_PART_MS = 1000;      // a page part last 1 sec
constexpr int32_t GALILEO_INAV_PAGE_SECONDS = 2;         // a full page last 2 sec
constexpr int32_t GALILEO_INAV_INTERLEAVER_ROWS = 8;
constexpr int32_t GALILEO_INAV_INTERLEAVER_COLS = 30;
constexpr int32_t GALILEO_TELEMETRY_RATE_BITS_SECOND = 250;  // bps
constexpr int32_t GALILEO_PAGE_TYPE_BITS = 6;
constexpr int32_t GALILEO_DATA_JK_BITS = 128;
constexpr int32_t GALILEO_DATA_FRAME_BITS = 196;
constexpr int32_t GALILEO_DATA_FRAME_BYTES = 25;
constexpr char GALILEO_INAV_PREAMBLE[11] = "0101100000";

const std::vector<std::pair<int32_t, int32_t>> TYPE({{1, 6}});
const std::vector<std::pair<int32_t, int32_t>> PAGE_TYPE_BIT({{1, 6}});

/* Page 1 - Word type 1: Ephemeris (1/4) */
const std::vector<std::pair<int32_t, int32_t>> IOD_NAV_1_BIT({{7, 10}});
const std::vector<std::pair<int32_t, int32_t>> T0_E_1_BIT({{17, 14}});
constexpr int32_t T0E_1_LSB = 60;
const std::vector<std::pair<int32_t, int32_t>> M0_1_BIT({{31, 32}});
constexpr double M0_1_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> E_1_BIT({{63, 32}});
constexpr double E_1_LSB = TWO_N33;
const std::vector<std::pair<int32_t, int32_t>> A_1_BIT({{95, 32}});
constexpr double A_1_LSB_GAL = TWO_N19;
// last two bits are reserved


/* Page 2 - Word type 2: Ephemeris (2/4) */
const std::vector<std::pair<int32_t, int32_t>> IOD_NAV_2_BIT({{7, 10}});
const std::vector<std::pair<int32_t, int32_t>> OMEGA_0_2_BIT({{17, 32}});
constexpr double OMEGA_0_2_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> I_0_2_BIT({{49, 32}});
constexpr double I_0_2_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> OMEGA_2_BIT({{81, 32}});
constexpr double OMEGA_2_LSB = PI_TWO_N31;
const std::vector<std::pair<int32_t, int32_t>> I_DOT_2_BIT({{113, 14}});
constexpr double I_DOT_2_LSB = PI_TWO_N43;
// last two bits are reserved

/* Word type 3: Ephemeris (3/4) and SISA */
const std::vector<std::pair<int32_t, int32_t>> IOD_NAV_3_BIT({{7, 10}});
const std::vector<std::pair<int32_t, int32_t>> OMEGA_DOT_3_BIT({{17, 24}});
constexpr double OMEGA_DOT_3_LSB = PI_TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> DELTA_N_3_BIT({{41, 16}});
constexpr double DELTA_N_3_LSB = PI_TWO_N43;
const std::vector<std::pair<int32_t, int32_t>> C_UC_3_BIT({{57, 16}});
constexpr double C_UC_3_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> C_US_3_BIT({{73, 16}});
constexpr double C_US_3_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> C_RC_3_BIT({{89, 16}});
constexpr double C_RC_3_LSB = TWO_N5;
const std::vector<std::pair<int32_t, int32_t>> C_RS_3_BIT({{105, 16}});
constexpr double C_RS_3_LSB = TWO_N5;
const std::vector<std::pair<int32_t, int32_t>> SISA_3_BIT({{121, 8}});


/* Word type 4: Ephemeris (4/4) and Clock correction parameters */
const std::vector<std::pair<int32_t, int32_t>> IOD_NAV_4_BIT({{7, 10}});
const std::vector<std::pair<int32_t, int32_t>> SV_ID_PRN_4_BIT({{17, 6}});
const std::vector<std::pair<int32_t, int32_t>> C_IC_4_BIT({{23, 16}});
constexpr double C_IC_4_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> C_IS_4_BIT({{39, 16}});
constexpr double C_IS_4_LSB = TWO_N29;
const std::vector<std::pair<int32_t, int32_t>> T0C_4_BIT({{55, 14}});  //
constexpr int32_t T0C_4_LSB = 60;
const std::vector<std::pair<int32_t, int32_t>> AF0_4_BIT({{69, 31}});  //
constexpr double AF0_4_LSB = TWO_N34;
const std::vector<std::pair<int32_t, int32_t>> AF1_4_BIT({{100, 21}});  //
constexpr double AF1_4_LSB = TWO_N46;
const std::vector<std::pair<int32_t, int32_t>> AF2_4_BIT({{121, 6}});
constexpr double AF2_4_LSB = TWO_N59;
const std::vector<std::pair<int32_t, int32_t>> SPARE_4_BIT({{127, 2}});
// last two bits are reserved

/* Word type 5: Ionospheric correction, BGD, signal health and data validity status and GST */
/* Ionospheric correction */
/* Az */
const std::vector<std::pair<int32_t, int32_t>> AI0_5_BIT({{7, 11}});  //
constexpr double AI0_5_LSB = TWO_N2;
const std::vector<std::pair<int32_t, int32_t>> AI1_5_BIT({{18, 11}});  //
constexpr double AI1_5_LSB = TWO_N8;
const std::vector<std::pair<int32_t, int32_t>> AI2_5_BIT({{29, 14}});  //
constexpr double AI2_5_LSB = TWO_N15;
/* Ionospheric disturbance flag */
const std::vector<std::pair<int32_t, int32_t>> REGION1_5_BIT({{43, 1}});      //
const std::vector<std::pair<int32_t, int32_t>> REGION2_5_BIT({{44, 1}});      //
const std::vector<std::pair<int32_t, int32_t>> REGION3_5_BIT({{45, 1}});      //
const std::vector<std::pair<int32_t, int32_t>> REGION4_5_BIT({{46, 1}});      //
const std::vector<std::pair<int32_t, int32_t>> REGION5_5_BIT({{47, 1}});      //
const std::vector<std::pair<int32_t, int32_t>> BGD_E1_E5A_5_BIT({{48, 10}});  //
constexpr double BGD_E1_E5A_5_LSB = TWO_N32;
const std::vector<std::pair<int32_t, int32_t>> BGD_E1_E5B_5_BIT({{58, 10}});  //
constexpr double BGD_E1_E5B_5_LSB = TWO_N32;
const std::vector<std::pair<int32_t, int32_t>> E5B_HS_5_BIT({{68, 2}});    //
const std::vector<std::pair<int32_t, int32_t>> E1_B_HS_5_BIT({{70, 2}});   //
const std::vector<std::pair<int32_t, int32_t>> E5B_DVS_5_BIT({{72, 1}});   //
const std::vector<std::pair<int32_t, int32_t>> E1_B_DVS_5_BIT({{73, 1}});  //
/* GST */
const std::vector<std::pair<int32_t, int32_t>> WN_5_BIT({{74, 12}});
const std::vector<std::pair<int32_t, int32_t>> TOW_5_BIT({{86, 20}});
const std::vector<std::pair<int32_t, int32_t>> SPARE_5_BIT({{106, 23}});


/* Page 6 */
const std::vector<std::pair<int32_t, int32_t>> A0_6_BIT({{7, 32}});
constexpr double A0_6_LSB = TWO_N30;
const std::vector<std::pair<int32_t, int32_t>> A1_6_BIT({{39, 24}});
constexpr double A1_6_LSB = TWO_N50;
const std::vector<std::pair<int32_t, int32_t>> DELTA_T_LS_6_BIT({{63, 8}});
const std::vector<std::pair<int32_t, int32_t>> T0T_6_BIT({{71, 8}});
constexpr int32_t T0T_6_LSB = 3600;
const std::vector<std::pair<int32_t, int32_t>> W_NOT_6_BIT({{79, 8}});
const std::vector<std::pair<int32_t, int32_t>> WN_LSF_6_BIT({{87, 8}});
const std::vector<std::pair<int32_t, int32_t>> DN_6_BIT({{95, 3}});
const std::vector<std::pair<int32_t, int32_t>> DELTA_T_LSF_6_BIT({{98, 8}});
const std::vector<std::pair<int32_t, int32_t>> TOW_6_BIT({{106, 20}});


/* Page 7 */
const std::vector<std::pair<int32_t, int32_t>> IOD_A_7_BIT({{7, 4}});
const std::vector<std::pair<int32_t, int32_t>> WN_A_7_BIT({{11, 2}});
const std::vector<std::pair<int32_t, int32_t>> T0A_7_BIT({{13, 10}});
constexpr int32_t T0A_7_LSB = 600;
const std::vector<std::pair<int32_t, int32_t>> SVI_D1_7_BIT({{23, 6}});
const std::vector<std::pair<int32_t, int32_t>> DELTA_A_7_BIT({{29, 13}});
constexpr double DELTA_A_7_LSB = TWO_N9;
const std::vector<std::pair<int32_t, int32_t>> E_7_BIT({{42, 11}});
constexpr double E_7_LSB = TWO_N16;
const std::vector<std::pair<int32_t, int32_t>> OMEGA_7_BIT({{53, 16}});
constexpr double OMEGA_7_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> DELTA_I_7_BIT({{69, 11}});
constexpr double DELTA_I_7_LSB = TWO_N14;
const std::vector<std::pair<int32_t, int32_t>> OMEGA0_7_BIT({{80, 16}});
constexpr double OMEGA0_7_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> OMEGA_DOT_7_BIT({{96, 11}});
constexpr double OMEGA_DOT_7_LSB = TWO_N33;
const std::vector<std::pair<int32_t, int32_t>> M0_7_BIT({{107, 16}});
constexpr double M0_7_LSB = TWO_N15;


/* Page 8 */
const std::vector<std::pair<int32_t, int32_t>> IOD_A_8_BIT({{7, 4}});
const std::vector<std::pair<int32_t, int32_t>> AF0_8_BIT({{11, 16}});
constexpr double AF0_8_LSB = TWO_N19;
const std::vector<std::pair<int32_t, int32_t>> AF1_8_BIT({{27, 13}});
constexpr double AF1_8_LSB = TWO_N38;
const std::vector<std::pair<int32_t, int32_t>> E5B_HS_8_BIT({{40, 2}});
const std::vector<std::pair<int32_t, int32_t>> E1_B_HS_8_BIT({{42, 2}});
const std::vector<std::pair<int32_t, int32_t>> SVI_D2_8_BIT({{44, 6}});
const std::vector<std::pair<int32_t, int32_t>> DELTA_A_8_BIT({{50, 13}});
constexpr double DELTA_A_8_LSB = TWO_N9;
const std::vector<std::pair<int32_t, int32_t>> E_8_BIT({{63, 11}});
constexpr double E_8_LSB = TWO_N16;
const std::vector<std::pair<int32_t, int32_t>> OMEGA_8_BIT({{74, 16}});
constexpr double OMEGA_8_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> DELTA_I_8_BIT({{90, 11}});
constexpr double DELTA_I_8_LSB = TWO_N14;
const std::vector<std::pair<int32_t, int32_t>> OMEGA0_8_BIT({{101, 16}});
constexpr double OMEGA0_8_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> OMEGA_DOT_8_BIT({{117, 11}});
constexpr double OMEGA_DOT_8_LSB = TWO_N33;


/* Page 9 */
const std::vector<std::pair<int32_t, int32_t>> IOD_A_9_BIT({{7, 4}});
const std::vector<std::pair<int32_t, int32_t>> WN_A_9_BIT({{11, 2}});
const std::vector<std::pair<int32_t, int32_t>> T0A_9_BIT({{13, 10}});
constexpr int32_t T0A_9_LSB = 600;
const std::vector<std::pair<int32_t, int32_t>> M0_9_BIT({{23, 16}});
constexpr double M0_9_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> AF0_9_BIT({{39, 16}});
constexpr double AF0_9_LSB = TWO_N19;
const std::vector<std::pair<int32_t, int32_t>> AF1_9_BIT({{55, 13}});
constexpr double AF1_9_LSB = TWO_N38;
const std::vector<std::pair<int32_t, int32_t>> E5B_HS_9_BIT({{68, 2}});
const std::vector<std::pair<int32_t, int32_t>> E1_B_HS_9_BIT({{70, 2}});
const std::vector<std::pair<int32_t, int32_t>> SVI_D3_9_BIT({{72, 6}});
const std::vector<std::pair<int32_t, int32_t>> DELTA_A_9_BIT({{78, 13}});
constexpr double DELTA_A_9_LSB = TWO_N9;
const std::vector<std::pair<int32_t, int32_t>> E_9_BIT({{91, 11}});
constexpr double E_9_LSB = TWO_N16;
const std::vector<std::pair<int32_t, int32_t>> OMEGA_9_BIT({{102, 16}});
constexpr double OMEGA_9_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> DELTA_I_9_BIT({{118, 11}});
constexpr double DELTA_I_9_LSB = TWO_N14;


/* Page 10 */
const std::vector<std::pair<int32_t, int32_t>> IOD_A_10_BIT({{7, 4}});
const std::vector<std::pair<int32_t, int32_t>> OMEGA0_10_BIT({{11, 16}});
constexpr double OMEGA0_10_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> OMEGA_DOT_10_BIT({{27, 11}});
constexpr double OMEGA_DOT_10_LSB = TWO_N33;
const std::vector<std::pair<int32_t, int32_t>> M0_10_BIT({{38, 16}});
constexpr double M0_10_LSB = TWO_N15;
const std::vector<std::pair<int32_t, int32_t>> AF0_10_BIT({{54, 16}});
constexpr double AF0_10_LSB = TWO_N19;
const std::vector<std::pair<int32_t, int32_t>> AF1_10_BIT({{70, 13}});
constexpr double AF1_10_LSB = TWO_N38;
const std::vector<std::pair<int32_t, int32_t>> E5B_HS_10_BIT({{83, 2}});
const std::vector<std::pair<int32_t, int32_t>> E1_B_HS_10_BIT({{85, 2}});
const std::vector<std::pair<int32_t, int32_t>> A_0_G_10_BIT({{87, 16}});
constexpr double A_0G_10_LSB = TWO_N35;
const std::vector<std::pair<int32_t, int32_t>> A_1_G_10_BIT({{103, 12}});
constexpr double A_1G_10_LSB = TWO_N51;
const std::vector<std::pair<int32_t, int32_t>> T_0_G_10_BIT({{115, 8}});
constexpr int32_t T_0_G_10_LSB = 3600;
const std::vector<std::pair<int32_t, int32_t>> WN_0_G_10_BIT({{123, 6}});


/* Page 0 */
const std::vector<std::pair<int32_t, int32_t>> TIME_0_BIT({{7, 2}});
const std::vector<std::pair<int32_t, int32_t>> WN_0_BIT({{97, 12}});
const std::vector<std::pair<int32_t, int32_t>> TOW_0_BIT({{109, 20}});


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_INAV_H
