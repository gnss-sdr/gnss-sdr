/*!
 * \file Beidou_B2a.h
 * \brief System parameters for BeiDou B2a (BDS-SIS-ICD-B2a-1.0)
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

#ifndef GNSS_SDR_BEIDOU_B2A_H
#define GNSS_SDR_BEIDOU_B2A_H

#include "gnss_frequencies.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

constexpr double BEIDOU_B2A_FREQ_HZ = FREQ5;              //!< BeiDou B2a [Hz]
constexpr double BEIDOU_B2A_CODE_RATE_CPS = 10.23e6;      //!< B2a primary code rate [chips/s]
constexpr double BEIDOU_B2A_CODE_LENGTH_CHIPS = 10230.0;  //!< B2a primary code length [chips]
constexpr double BEIDOU_B2A_CODE_PERIOD_S = 0.001;        //!< B2a primary code period [seconds]
constexpr double BEIDOU_B2A_OPT_ACQ_FS_SPS = 20e6;
constexpr uint32_t BEIDOU_B2A_CODE_PERIOD_MS = 1;
constexpr uint32_t BEIDOU_B2A_PREAMBLE_LENGTH_SYMBOLS = 24;
constexpr int32_t BEIDOU_B2A_SYMBOLS_PER_BIT = 1;
constexpr int32_t BEIDOU_B2A_TELEMETRY_RATE_SYMBOLS_SECOND = 1000;
constexpr int32_t BEIDOU_B2A_SECONDARY_CODE_LENGTH = 5;
constexpr char BEIDOU_B2A_SECONDARY_CODE_STR[6] = "00010";                        //!< ICD data secondary code, MSB first
constexpr char BEIDOU_B2A_PREAMBLE_SYMBOLS_STR[25] = "111000100100110111101000";  //!< 0xE24DE8

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B2A_H
