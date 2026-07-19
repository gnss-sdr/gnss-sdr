/*!
 * \file Beidou_CNAV1.h
 * \brief B-CNAV1 navigation message constants (BDS-SIS-ICD-B1C-1.0 §6.2, §7)
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
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
#ifndef GNSS_SDR_BEIDOU_CNAV1_H
#define GNSS_SDR_BEIDOU_CNAV1_H

#include "MATH_CONSTANTS.h"
#include <cstdint>

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
constexpr double BEIDOU_CNAV1_DELTA_A_LSB = TWO_N9;
constexpr double BEIDOU_CNAV1_A_DOT_LSB = TWO_N21;  //!< 2^-21 m/s, Table 7-8
constexpr double BEIDOU_CNAV1_DELTA_N0_LSB = TWO_N44 * GNSS_PI;
constexpr double BEIDOU_CNAV1_DELTA_N0_DOT_LSB = TWO_N57 * GNSS_PI;
constexpr double BEIDOU_CNAV1_M0_LSB = TWO_N32 * GNSS_PI;
constexpr double BEIDOU_CNAV1_E_LSB = TWO_N34;
constexpr double BEIDOU_CNAV1_OMEGA_LSB = TWO_N32 * GNSS_PI;
constexpr double BEIDOU_CNAV1_I0_LSB = TWO_N32 * GNSS_PI;
constexpr double BEIDOU_CNAV1_OMEGADOT_LSB = TWO_N44 * GNSS_PI;
constexpr double BEIDOU_CNAV1_IDOT_LSB = TWO_N44 * GNSS_PI;
constexpr double BEIDOU_CNAV1_CIS_LSB = TWO_N30;
constexpr double BEIDOU_CNAV1_CIC_LSB = TWO_N30;
constexpr double BEIDOU_CNAV1_CRS_LSB = TWO_N8;
constexpr double BEIDOU_CNAV1_CRC_LSB = TWO_N8;
constexpr double BEIDOU_CNAV1_CUS_LSB = TWO_N30;
constexpr double BEIDOU_CNAV1_CUC_LSB = TWO_N30;
constexpr double BEIDOU_CNAV1_AF0_LSB = TWO_N34;
constexpr double BEIDOU_CNAV1_AF1_LSB = TWO_N50;
constexpr double BEIDOU_CNAV1_AF2_LSB = TWO_N66;
constexpr double BEIDOU_CNAV1_TGD_LSB = TWO_N34;
constexpr double BEIDOU_CNAV1_ISC_LSB = TWO_N34;

#endif  // GNSS_SDR_BEIDOU_CNAV1_H
