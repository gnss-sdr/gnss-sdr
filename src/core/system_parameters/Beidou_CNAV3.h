/*!
 * \file Beidou_CNAV3.h
 * \brief B-CNAV3 navigation message constants (BDS-SIS-ICD-B2b-1.0)
 * \author Chandoss, 2026. huangchh37(at)mail2.sysu.edu.cn
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

#ifndef GNSS_SDR_BEIDOU_CNAV3_H
#define GNSS_SDR_BEIDOU_CNAV3_H

#include "Beidou_CNAV1.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

constexpr int32_t BEIDOU_CNAV3_FRAME_SYMBOLS = 1000;
constexpr int32_t BEIDOU_CNAV3_FRAME_PERIOD_S = 1;
constexpr int32_t BEIDOU_CNAV3_SYMBOL_RATE_SPS = 1000;
constexpr int32_t BEIDOU_CNAV3_PREAMBLE_SYMBOLS = 16;
constexpr int32_t BEIDOU_CNAV3_PRN_SYMBOLS = 6;
constexpr int32_t BEIDOU_CNAV3_RESERVED_SYMBOLS = 6;
constexpr int32_t BEIDOU_CNAV3_LDPC_SYMBOLS = 972;
constexpr int32_t BEIDOU_CNAV3_INFO_BITS = 486;
constexpr int32_t BEIDOU_CNAV3_CRC_BITS = 24;
constexpr int32_t BEIDOU_CNAV3_DATA_BITS = BEIDOU_CNAV3_INFO_BITS - BEIDOU_CNAV3_CRC_BITS;
constexpr int32_t BEIDOU_CNAV3_MES_TYPE_BITS = 6;

constexpr int32_t BEIDOU_CNAV3_MSG_EPH = 10;
constexpr int32_t BEIDOU_CNAV3_MSG_CLK = 30;
constexpr int32_t BEIDOU_CNAV3_MSG_ALM = 40;

constexpr int32_t BDS_EPH_SOURCE_CNAV3 = 8;

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_CNAV3_H
