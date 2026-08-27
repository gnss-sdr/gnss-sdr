/*!
 * \file Beidou_B1C.h
 * \brief Defines system parameters for BeiDou B1C open service signal
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
 *
 * Parameter values from BDS-SIS-ICD-B1C-1.0 (2017-12).
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
#ifndef GNSS_SDR_BEIDOU_B1C_H
#define GNSS_SDR_BEIDOU_B1C_H

#include "gnss_frequencies.h"
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

constexpr double BEIDOU_B1C_FREQ_HZ = FREQ1;                      //!< §4.2.2, Table 4-1 [Hz]
constexpr double BEIDOU_B1C_CODE_RATE_CPS = 1.023e6;              //!< §5.2.1 [chips/s]
constexpr double BEIDOU_B1C_CODE_LENGTH_CHIPS = 10230.0;          //!< §5.2.1 [chips]
constexpr double BEIDOU_B1C_CODE_PERIOD_S = 0.01;                 //!< §5.2.1, 10 ms [s]
constexpr uint32_t BEIDOU_B1C_CODE_PERIOD_MS = 10;                //!< §5.2.1 [ms]
constexpr uint32_t BEIDOU_B1C_OPT_ACQ_FS_SPS = 4000000;           //!< 4 samples/chip at 1.023 Mcps
constexpr int32_t BEIDOU_B1C_PILOT_SECONDARY_CODE_LENGTH = 1800;  //!< §5.2.2 [chips]
constexpr int32_t BEIDOU_B1C_TELEMETRY_RATE_SPS = 100;            //!< §6.2 [symbols/s]
constexpr int32_t BEIDOU_B1C_SYMBOLS_PER_BIT = 1;                 //!< 100 sps, 10 ms/code period
constexpr int32_t BEIDOU_B1C_FRAME_LENGTH_SYMBOLS = 1800;         //!< §6.2 [symbols]
constexpr int32_t BEIDOU_B1C_FRAME_PERIOD_S = 18;                 //!< §6.2 [s]
constexpr int32_t BEIDOU_B1C_NUMBER_OF_PRNS = 63;                 //!< §5.2.1

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B1C_H
