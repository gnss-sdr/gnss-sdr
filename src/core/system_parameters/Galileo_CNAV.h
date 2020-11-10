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
#include <utility>

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
constexpr int32_t GALILEO_CNAV_CRC_LENGTH = 24;
constexpr int32_t GALILEO_CNAV_MESSAGE_BITS_PER_PAGE = 424;
constexpr int32_t GALILEO_CNAV_PAGE_HEADER_BITS = 24;
constexpr int32_t GALILEO_CNAV_PREAMBLE_LENGTH_BITS = 16;
constexpr int32_t GALILEO_CNAV_MAX_NUMBER_ENCODED_BLOCKS = 255;
constexpr int32_t GALILEO_CNAV_MT1_HEADER_BITS = 32;

constexpr char GALILEO_CNAV_PREAMBLE[17] = "1011011101110000";

const std::pair<int32_t, int32_t> GALILEO_HAS_STATUS({1, 2});
const std::pair<int32_t, int32_t> GALILEO_HAS_MESSAGE_TYPE({5, 2});
const std::pair<int32_t, int32_t> GALILEO_HAS_MESSAGE_ID({7, 5});
const std::pair<int32_t, int32_t> GALILEO_HAS_MESSAGE_SIZE({12, 5});
const std::pair<int32_t, int32_t> GALILEO_HAS_MESSAGE_PAGE_ID({17, 8});

const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_TOH({1, 12});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_MASK_FLAG({13, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_ORBIT_CORRECTION_FLAG({14, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_CLOCK_FULLSET_FLAG({15, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_CLOCK_SUBSET_FLAG({16, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_CODE_BIAS_FLAG({17, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_PHASE_BIAS_FLAG({18, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_URA_FLAG({19, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_MASK_ID({23, 5});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_IOD_ID({28, 5});


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_CNAV_H
