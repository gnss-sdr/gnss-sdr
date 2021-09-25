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
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_CNAV_H
#define GNSS_SDR_GALILEO_CNAV_H

#include <cstddef>
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
constexpr int32_t GALILEO_CNAV_BYTES_FOR_CRC = 58;
constexpr int32_t GALILEO_CNAV_CRC_LENGTH = 24;
constexpr int32_t GALILEO_CNAV_MESSAGE_BITS_PER_PAGE = 424;
constexpr int32_t GALILEO_CNAV_PAGE_HEADER_BITS = 24;
constexpr int32_t GALILEO_CNAV_PREAMBLE_LENGTH_BITS = 16;
constexpr int32_t GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK = 255;
constexpr int32_t GALILEO_CNAV_MT1_HEADER_BITS = 32;
constexpr int32_t GALILEO_CNAV_OCTETS_IN_SUBPAGE = 53;
constexpr int32_t GALILEO_CNAV_INFORMATION_VECTOR_LENGTH = 32;

constexpr char GALILEO_CNAV_PREAMBLE[17] = "1011011101110000";

// Galileo HAS message field lengths
constexpr size_t HAS_MSG_NSYS_LENGTH = 4;
constexpr size_t HAS_MSG_ID_MASK_LENGTH = 4;
constexpr size_t HAS_MSG_SATELLITE_MASK_LENGTH = 40;
constexpr size_t HAS_MSG_SIGNAL_MASK_LENGTH = 16;
constexpr size_t HAS_MSG_NAV_MESSAGE_LENGTH = 3;
constexpr size_t HAS_MSG_MASK_RESERVED_LENGTH = 6;
constexpr size_t HAS_MSG_VALIDITY_INDEX_LENGTH = 4;
constexpr size_t HAS_MSG_IOD_GPS_LENGTH = 8;
constexpr size_t HAS_MSG_IOD_GAL_LENGTH = 10;
constexpr size_t HAS_MSG_DELTA_RADIAL_LENGTH = 14;
constexpr size_t HAS_MSG_DELTA_ALONG_TRACK_LENGTH = 12;
constexpr size_t HAS_MSG_DELTA_CROSS_TRACK_LENGTH = 12;
constexpr size_t HAS_MSG_DELTA_CLOCK_C0_MULTIPLIER_LENGTH = 2;
constexpr size_t HAS_MSG_DELTA_CLOCK_C0_LENGTH = 14;
constexpr size_t HAS_MSG_NSYSPRIME_LENGTH = 4;
constexpr size_t HAS_MSG_ID_CLOCK_SUBSET_LENGTH = 4;
constexpr size_t HAS_MSG_DELTA_CLOCK_MULTIPLIER_SUBSET_LENGTH = 2;
constexpr size_t HAS_MSG_DELTA_CLOCK_C0_SUBSET_LENGTH = 13;
constexpr size_t HAS_MSG_CODE_BIAS_LENGTH = 11;
constexpr size_t HAS_MSG_PHASE_BIAS_LENGTH = 11;
constexpr size_t HAS_MSG_PHASE_DISCONTINUITY_INDICATOR_LENGTH = 2;
constexpr size_t HAS_MSG_URA_LENGTH = 2;

constexpr int32_t HAS_MSG_NUMBER_MASK_IDS = 32;
constexpr int32_t HAS_MSG_NUMBER_GNSS_IDS = 16;
constexpr int32_t HAS_MSG_NUMBER_MESSAGE_IDS = 32;
constexpr int32_t HAS_MSG_NUMBER_SATELLITE_IDS = 40;
constexpr int32_t HAS_MSG_NUMBER_SIGNAL_MASKS = 16;

constexpr float HAS_MSG_DELTA_RADIAL_SCALE_FACTOR = 0.0025;
constexpr float HAS_MSG_DELTA_ALONG_TRACK_SCALE_FACTOR = 0.008;
constexpr float HAS_MSG_DELTA_CROSS_TRACK_SCALE_FACTOR = 0.008;
constexpr float HAS_MSG_DELTA_CLOCK_SCALE_FACTOR = 0.0025;
constexpr float HAS_MSG_CODE_BIAS_SCALE_FACTOR = 0.02;
constexpr float HAS_MSG_PHASE_BIAS_SCALE_FACTOR = 0.01;

constexpr uint16_t HAS_MSG_NUMBER_MAX_TOH = 3599;

constexpr uint8_t HAS_MSG_GPS_SYSTEM = 0;      // Table 8 ICD v1.2
constexpr uint8_t HAS_MSG_GALILEO_SYSTEM = 2;  // Table 8 ICD v1.2
constexpr uint8_t HAS_MSG_WRONG_SYSTEM = 255;

const std::pair<int32_t, int32_t> GALILEO_HAS_STATUS({1, 2});
const std::pair<int32_t, int32_t> GALILEO_HAS_RESERVED({3, 2});
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
