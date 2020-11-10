/*!
 * \file Galileo_CNAV.h
 * \brief Galileo CNAV mesage constants. Data from:
 * Galileo High Accuracy Service E6-B Signal-In-Space Message Specification v1.2
 * (April 2020).
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_CNAV_H
#define GNSS_SDR_GALILEO_CNAV_H

#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


// Galileo CNAV message structure
constexpr int32_t GALILEO_CNAV_SYMBOLS_PER_PAGE = 1000;  //!< Total numer of symbols per HAS page including the sync pattern
constexpr int32_t GALILEO_CNAV_PREAMBLE_PERIOD_SYMBOLS = 1000;
constexpr int32_t GALILEO_CNAV_PAGE_MS = 1;  //!< Duration in ms of a CNAV page
constexpr int32_t GALILEO_CNAV_INTERLEAVER_ROWS = 8;
constexpr int32_t GALILEO_CNAV_INTERLEAVER_COLS = 123;
constexpr int32_t GALILEO_CNAV_TELEMETRY_RATE_BITS_SECOND = 1000;  // bps
constexpr int32_t GALILEO_CNAV_HAS_PAGE_DATA_BITS = 448;
constexpr int32_t GALILEO_CNAV_PAGE_RESERVED_BITS = 14;
constexpr int32_t GALILEO_CNAV_BITS_FOR_CRC = GALILEO_CNAV_HAS_PAGE_DATA_BITS + GALILEO_CNAV_PAGE_RESERVED_BITS;  // 462
constexpr int32_t GALILEO_CNAV_BYTES_FOR_CRC = 60;
constexpr int32_t GALILEO_CNAV_MESSAGE_BITS_PER_PAGE = 424;
constexpr int32_t GALILEO_CNAV_PAGE_HEADER_BITS = 24;
constexpr int32_t GALILEO_CNAV_PREAMBLE_LENGTH_BITS = 16;
constexpr char GALILEO_CNAV_PREAMBLE[17] = "1011011101110000";


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_CNAV_H
