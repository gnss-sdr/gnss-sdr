/*!
 * \file qzss.h
 * \brief  Defines system parameters for QZSS signals
 * \author Carles Fernández-Prades, 2026. cfernandez (at) cttc.es
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


#ifndef GNSS_SDR_QZSS_H
#define GNSS_SDR_QZSS_H

#include "gnss_frequencies.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

static constexpr double QZSS_L1_FREQ_HZ = FREQ1;
static constexpr double QZSS_L5_FREQ_HZ = FREQ5;
static constexpr double QZSS_L1_CHIP_RATE = 1.023e6;
static constexpr double QZSS_L5_CHIP_RATE = 10.23e6;
static constexpr double QZSS_L1_CA_CODE_PERIOD_S = 0.001;
static constexpr double QZSS_L5I_CODE_PERIOD_S = 0.001;

static constexpr int QZSS_L1_CODE_LENGTH = 1023;
static constexpr int QZSS_L5_CODE_LENGTH = 10230;
static constexpr int QZSS_L1_PERIOD_MS = 1;
static constexpr int QZSS_L5I_PERIOD_MS = 1;
static constexpr int32_t QZSS_CA_PREAMBLE_LENGTH_SYMBOLS = 160;
static constexpr int32_t QZSS_CA_TELEMETRY_SYMBOLS_PER_BIT = 20;
static constexpr int32_t QZSS_L5_SAMPLES_PER_SYMBOL = 10;
static constexpr int32_t QZSS_L5Q_NH_CODE_LENGTH = 20;
static constexpr int32_t QZSS_L5I_NH_CODE_LENGTH = 10;
static constexpr uint32_t QZSS_L1_OPT_ACQ_FS_SPS = 2000000;
static constexpr uint32_t QZSS_L5_OPT_ACQ_FS_SPS = 10000000;

static constexpr uint16_t XA_ALL_ONES = 0x1FFF;      // 13 bits all ones
static constexpr uint16_t XA_SHORT_DECODE = 0x1FFD;  // 1111111111101 (ICD)

constexpr const char QZSS_CA_PREAMBLE_SYMBOLS_STR[161] = "1111111111111111111100000000000000000000000000000000000000000000000000000000000011111111111111111111000000000000000000001111111111111111111111111111111111111111";
constexpr const char QZSS_L5Q_NH_CODE_STR[21] = "00000100110101001110";
constexpr const char QZSS_L5I_NH_CODE_STR[11] = "0000110101";

/** \} */
/** \} */

#endif  // GNSS_SDR_QZSS_H
