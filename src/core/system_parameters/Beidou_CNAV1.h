/*!
 * \file Beidou_CNAV1.h
 * \brief B-CNAV1 navigation message constants (BDS-SIS-ICD-B1C-1.0 §6.2, §7)
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef GNSS_SDR_BEIDOU_CNAV1_H
#define GNSS_SDR_BEIDOU_CNAV1_H

#include <cmath>
#include <cstdint>

#ifndef GNSS_SDR_TWO_N8
constexpr double GNSS_SDR_TWO_N8 = 0.00390625;
#endif
#ifndef GNSS_SDR_TWO_N9
constexpr double GNSS_SDR_TWO_N9 = 0.001953125;
#endif
#ifndef GNSS_SDR_TWO_N21
constexpr double GNSS_SDR_TWO_N21 = 4.76837158203125e-7;
#endif
#ifndef GNSS_SDR_TWO_N30
constexpr double GNSS_SDR_TWO_N30 = 9.313225746154785e-10;
#endif
#ifndef GNSS_SDR_TWO_N32
constexpr double GNSS_SDR_TWO_N32 = 2.3283064365386963e-10;
#endif
#ifndef GNSS_SDR_TWO_N34
constexpr double GNSS_SDR_TWO_N34 = 5.820766091346741e-11;
#endif
#ifndef GNSS_SDR_TWO_N44
constexpr double GNSS_SDR_TWO_N44 = 5.684341886080802e-14;
#endif
#ifndef GNSS_SDR_TWO_N50
constexpr double GNSS_SDR_TWO_N50 = 8.881784197001252e-16;
#endif
#ifndef GNSS_SDR_TWO_N57
constexpr double GNSS_SDR_TWO_N57 = 6.938893903907228e-18;
#endif
#ifndef GNSS_SDR_TWO_N66
constexpr double GNSS_SDR_TWO_N66 = 1.3552527156068805e-20;
#endif

constexpr int32_t BEIDOU_CNAV1_FRAME_SYMBOLS = 1800;
constexpr int32_t BEIDOU_CNAV1_FRAME_PERIOD_S = 18;
constexpr int32_t BEIDOU_CNAV1_SYMBOL_RATE_SPS = 100;
constexpr int32_t BEIDOU_CNAV1_SUBFRAME1_SYMBOLS = 72;
constexpr int32_t BEIDOU_CNAV1_SUBFRAME2_SYMBOLS = 1200;
constexpr int32_t BEIDOU_CNAV1_SUBFRAME3_SYMBOLS = 528;
constexpr int32_t BEIDOU_CNAV1_SOH_LSB_S = 18;
constexpr int32_t BEIDOU_CNAV1_INTERLEAVED_SYMBOLS = 1728;
constexpr int32_t BEIDOU_CNAV1_INTERLEAVE_ROWS = 36;
constexpr int32_t BEIDOU_CNAV1_INTERLEAVE_COLS = 48;
constexpr int32_t BEIDOU_CNAV1_SF2_DATA_BITS = 600;
constexpr int32_t BEIDOU_CNAV1_SF3_DATA_BITS = 264;
constexpr int32_t BEIDOU_CNAV1_CRC_BITS = 24;

constexpr int32_t BEIDOU_CNAV1_PAGE_IONO_UTC = 1;

constexpr double BEIDOU_CNAV1_A_REF_MEO = 27906100.0;
constexpr double BEIDOU_CNAV1_A_REF_IGSO = 42162200.0;
constexpr double BEIDOU_CNAV1_TOE_TOC_LSB = 300.0;
constexpr double BEIDOU_CNAV1_DELTA_A_LSB = GNSS_SDR_TWO_N9;
constexpr double BEIDOU_CNAV1_A_DOT_LSB = GNSS_SDR_TWO_N21;  //!< 2^-21 m/s, Table 7-8
constexpr double BEIDOU_CNAV1_DELTA_N0_LSB = GNSS_SDR_TWO_N44 * M_PI;
constexpr double BEIDOU_CNAV1_DELTA_N0_DOT_LSB = GNSS_SDR_TWO_N57 * M_PI;
constexpr double BEIDOU_CNAV1_M0_LSB = GNSS_SDR_TWO_N32 * M_PI;
constexpr double BEIDOU_CNAV1_E_LSB = GNSS_SDR_TWO_N34;
constexpr double BEIDOU_CNAV1_OMEGA_LSB = GNSS_SDR_TWO_N32 * M_PI;
constexpr double BEIDOU_CNAV1_I0_LSB = GNSS_SDR_TWO_N32 * M_PI;
constexpr double BEIDOU_CNAV1_OMEGADOT_LSB = GNSS_SDR_TWO_N44 * M_PI;
constexpr double BEIDOU_CNAV1_IDOT_LSB = GNSS_SDR_TWO_N44 * M_PI;
constexpr double BEIDOU_CNAV1_CIS_LSB = GNSS_SDR_TWO_N30;
constexpr double BEIDOU_CNAV1_CIC_LSB = GNSS_SDR_TWO_N30;
constexpr double BEIDOU_CNAV1_CRS_LSB = GNSS_SDR_TWO_N8;
constexpr double BEIDOU_CNAV1_CRC_LSB = GNSS_SDR_TWO_N8;
constexpr double BEIDOU_CNAV1_CUS_LSB = GNSS_SDR_TWO_N30;
constexpr double BEIDOU_CNAV1_CUC_LSB = GNSS_SDR_TWO_N30;
constexpr double BEIDOU_CNAV1_AF0_LSB = GNSS_SDR_TWO_N34;
constexpr double BEIDOU_CNAV1_AF1_LSB = GNSS_SDR_TWO_N50;
constexpr double BEIDOU_CNAV1_AF2_LSB = GNSS_SDR_TWO_N66;
constexpr double BEIDOU_CNAV1_TGD_LSB = GNSS_SDR_TWO_N34;
constexpr double BEIDOU_CNAV1_ISC_LSB = GNSS_SDR_TWO_N34;

#endif  // GNSS_SDR_BEIDOU_CNAV1_H
