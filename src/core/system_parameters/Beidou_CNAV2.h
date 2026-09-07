/*!
 * \file Beidou_CNAV2.h
 * \brief B-CNAV2 navigation message constants (BDS-SIS-ICD-B2a-1.0).
 *        First cut skips 64-ary LDPC and uses the systematic 288 info bits.
 * \author huangchuhan, 2026. huangchh37(at)mail2.sysu.edu.cn
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

#ifndef GNSS_SDR_BEIDOU_CNAV2_H
#define GNSS_SDR_BEIDOU_CNAV2_H

#include "Beidou_CNAV1.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

constexpr int32_t BEIDOU_CNAV2_FRAME_SYMBOLS = 600;
constexpr int32_t BEIDOU_CNAV2_FRAME_PERIOD_S = 3;
constexpr int32_t BEIDOU_CNAV2_SYMBOL_RATE_SPS = 200;
constexpr int32_t BEIDOU_CNAV2_MS_PER_SYMBOL = 5;
constexpr int32_t BEIDOU_CNAV2_FRAME_MS = 3000;
constexpr int32_t BEIDOU_CNAV2_PREAMBLE_SYMBOLS = 24;
constexpr int32_t BEIDOU_CNAV2_PREAMBLE_MS = 120;   //!< 24 symbols * 5 ms secondary code
constexpr int32_t BEIDOU_CNAV2_LDPC_SYMBOLS = 576;  //!< encoded payload; LDPC not implemented yet
constexpr int32_t BEIDOU_CNAV2_INFO_BITS = 288;
constexpr int32_t BEIDOU_CNAV2_CRC_BITS = 24;
constexpr int32_t BEIDOU_CNAV2_DATA_BITS = BEIDOU_CNAV2_INFO_BITS - BEIDOU_CNAV2_CRC_BITS;
constexpr int32_t BEIDOU_CNAV2_MES_TYPE_BITS = 6;
constexpr int32_t BEIDOU_CNAV2_SOW_LSB_S = 3;
constexpr int32_t BEIDOU_CNAV2_PREAMBLE_CORR_THRESHOLD = 115;

constexpr int32_t BEIDOU_CNAV2_MSG_EPH1 = 10;
constexpr int32_t BEIDOU_CNAV2_MSG_EPH2 = 11;
constexpr int32_t BEIDOU_CNAV2_MSG_CLK_IONO = 30;
constexpr int32_t BEIDOU_CNAV2_MSG_CLK_ALM = 31;
constexpr int32_t BEIDOU_CNAV2_MSG_CLK_EOP = 32;
constexpr int32_t BEIDOU_CNAV2_MSG_CLK_UTC = 33;
constexpr int32_t BEIDOU_CNAV2_MSG_CLK_DC = 34;

//! RTKLIB eph_t::code / Beidou_Cnav1_Ephemeris::sig_type for B-CNAV2.
constexpr int32_t BDS_EPH_SOURCE_CNAV2 = 9;

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_CNAV2_H
