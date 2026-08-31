/*!
 * \file Beidou_B2b.h
 * \brief System parameters for BeiDou B2b_I (BDS-SIS-ICD-B2b-1.0)
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

#ifndef GNSS_SDR_BEIDOU_B2B_H
#define GNSS_SDR_BEIDOU_B2B_H

#include "gnss_frequencies.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

constexpr double BEIDOU_B2B_FREQ_HZ = FREQ2_BDS;          //!< BeiDou B2b [Hz]
constexpr double BEIDOU_B2B_CODE_RATE_CPS = 10.23e6;      //!< B2b_I code rate [chips/s]
constexpr double BEIDOU_B2B_CODE_LENGTH_CHIPS = 10230.0;  //!< B2b_I code length [chips]
constexpr double BEIDOU_B2B_CODE_PERIOD_S = 0.001;        //!< B2b_I code period [seconds]
constexpr double BEIDOU_B2B_OPT_ACQ_FS_SPS = 20e6;
constexpr uint32_t BEIDOU_B2B_CODE_PERIOD_MS = 1;
constexpr uint32_t BEIDOU_B2B_PREAMBLE_LENGTH_SYMBOLS = 16;
constexpr int32_t BEIDOU_B2B_SYMBOLS_PER_BIT = 1;
constexpr int32_t BEIDOU_B2B_TELEMETRY_RATE_SYMBOLS_SECOND = 1000;
constexpr char BEIDOU_B2B_PREAMBLE_SYMBOLS_STR[17] = "1110101110010000";  //!< 0xEB90

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B2B_H
