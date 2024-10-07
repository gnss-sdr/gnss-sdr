/*!
 * \file Galileo_CNAV.h
 * \brief Galileo CNAV mesage constants. Data from:
 * Galileo High Accuracy Service Signal-In-Space Interface Control Document
 * (HAS SIS ICD) Issue 1.0, May 2022
 * \author Carles Fernandez-Prades, 2020-2022. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_CNAV_H
#define GNSS_SDR_GALILEO_CNAV_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

// Galileo HAS message field lengths
constexpr size_t HAS_MSG_NSYS_LENGTH = 4;                            // HAS SIS ICD 1.0 Table 15
constexpr size_t HAS_MSG_ID_MASK_LENGTH = 4;                         // HAS SIS ICD 1.0 Table 16
constexpr size_t HAS_MSG_SATELLITE_MASK_LENGTH = 40;                 // HAS SIS ICD 1.0 Table 16
constexpr size_t HAS_MSG_SIGNAL_MASK_LENGTH = 16;                    // HAS SIS ICD 1.0 Table 16
constexpr size_t HAS_MSG_NAV_MESSAGE_LENGTH = 3;                     // HAS SIS ICD 1.0 Table 16
constexpr size_t HAS_MSG_MASK_RESERVED_LENGTH = 6;                   // HAS SIS ICD 1.0 Table 15
constexpr size_t HAS_MSG_VALIDITY_INDEX_LENGTH = 4;                  // HAS SIS ICD 1.0 Table 22
constexpr size_t HAS_MSG_IOD_GPS_LENGTH = 8;                         // HAS SIS ICD 1.0 Table 26
constexpr size_t HAS_MSG_IOD_GAL_LENGTH = 10;                        // HAS SIS ICD 1.0 Table 26
constexpr size_t HAS_MSG_DELTA_RADIAL_LENGTH = 13;                   // HAS SIS ICD 1.0 Table 25
constexpr size_t HAS_MSG_DELTA_IN_TRACK_LENGTH = 12;                 // HAS SIS ICD 1.0 Table 25
constexpr size_t HAS_MSG_DELTA_CROSS_TRACK_LENGTH = 12;              // HAS SIS ICD 1.0 Table 25
constexpr size_t HAS_MSG_DELTA_CLOCK_MULTIPLIER_LENGTH = 2;          // HAS SIS ICD 1.0 Table 28
constexpr size_t HAS_MSG_DELTA_CLOCK_CORRECTION_LENGTH = 13;         // HAS SIS ICD 1.0 Table 31
constexpr size_t HAS_MSG_NSYS_SUB_LENGTH = 4;                        // HAS SIS ICD 1.0 Table 32
constexpr size_t HAS_MSG_ID_CLOCK_SUBSET_LENGTH = 4;                 // HAS SIS ICD 1.0 Table 32
constexpr size_t HAS_MSG_DELTA_CLOCK_MULTIPLIER_SUBSET_LENGTH = 2;   // HAS SIS ICD 1.0 Table 33
constexpr size_t HAS_MSG_DELTA_CLOCK_CORRECTION_SUBSET_LENGTH = 13;  // HAS SIS ICD 1.0 Table 34
constexpr size_t HAS_MSG_CODE_BIAS_LENGTH = 11;                      // HAS SIS ICD 1.0 Table 37
constexpr size_t HAS_MSG_PHASE_BIAS_LENGTH = 11;                     // HAS SIS ICD 1.0 Table 40
constexpr size_t HAS_MSG_PHASE_DISCONTINUITY_INDICATOR_LENGTH = 2;   // HAS SIS ICD 1.0 Table 40

constexpr uint64_t MAX_SECONDS_REMEMBERING_MID = 150;  // HAS SIS ICD 1.0 Section 6.4.1 HAS Message Completion Time-out

// Galileo CNAV message structure
constexpr int32_t GALILEO_CNAV_SYMBOLS_PER_PAGE = 1000;  // Total numer of symbols per HAS page including the sync pattern
constexpr int32_t GALILEO_CNAV_PREAMBLE_PERIOD_SYMBOLS = 1000;
constexpr int32_t GALILEO_CNAV_PAGE_MS = 1;                             // Duration in ms of a CNAV page
constexpr int32_t GALILEO_CNAV_INTERLEAVER_ROWS = 8;                    // HAS SIS ICD 1.0 Table 4
constexpr int32_t GALILEO_CNAV_INTERLEAVER_COLS = 123;                  // HAS SIS ICD 1.0 Table 4
constexpr int32_t GALILEO_CNAV_TELEMETRY_RATE_BITS_SECOND = 1000;       // bps
constexpr int32_t GALILEO_CNAV_HAS_PAGE_DATA_BITS = 448;                // HAS SIS ICD 1.0 Table 5
constexpr int32_t GALILEO_CNAV_PAGE_RESERVED_BITS = 14;                 // HAS SIS ICD 1.0 Table 5
constexpr int32_t GALILEO_CNAV_BYTES_FOR_CRC = 58;                      // ceil(462 / 8)
constexpr int32_t GALILEO_CNAV_CRC_LENGTH = 24;                         // HAS SIS ICD 1.0 Table 5
constexpr int32_t GALILEO_CNAV_MESSAGE_BITS_PER_PAGE = 424;             // HAS SIS ICD 1.0 Table 6
constexpr int32_t GALILEO_CNAV_PAGE_HEADER_BITS = 24;                   // HAS SIS ICD 1.0 Table 6
constexpr int32_t GALILEO_CNAV_PREAMBLE_LENGTH_BITS = 16;               // HAS SIS ICD 1.0 Table 5
constexpr int32_t GALILEO_CNAV_MAX_NUMBER_SYMBOLS_ENCODED_BLOCK = 255;  // HAS SIS ICD 1.0 Section 6.2 Reed-Solomon Code
constexpr int32_t GALILEO_CNAV_MT1_HEADER_BITS = 32;                    // HAS SIS ICD 1.0 Table 11
constexpr int32_t GALILEO_CNAV_OCTETS_IN_SUBPAGE = 53;                  // HAS SIS ICD 1.0 Section 6.3 HAS Encoding and Transmission
constexpr int32_t GALILEO_CNAV_INFORMATION_VECTOR_LENGTH = 32;          // HAS SIS ICD 1.0 Section 6.2 Reed-Solomon Code

constexpr int32_t GALILEO_CNAV_BITS_FOR_CRC = GALILEO_CNAV_HAS_PAGE_DATA_BITS + GALILEO_CNAV_PAGE_RESERVED_BITS;  // 462

constexpr int32_t HAS_MSG_NUMBER_MASK_IDS = 32;       // HAS SIS ICD 1.0 Table 13
constexpr int32_t HAS_MSG_NUMBER_GNSS_IDS = 16;       // HAS SIS ICD 1.0 Table 18
constexpr int32_t HAS_MSG_NUMBER_MESSAGE_IDS = 32;    // HAS SIS ICD 1.0 Table 8
constexpr int32_t HAS_MSG_NUMBER_SATELLITE_IDS = 40;  // HAS SIS ICD 1.0 Table 19
constexpr int32_t HAS_MSG_NUMBER_SIGNAL_MASKS = 16;   // HAS SIS ICD 1.0 Table 20

constexpr float HAS_MSG_DELTA_RADIAL_SCALE_FACTOR = 0.0025;      // HAS SIS ICD 1.0 Table 25
constexpr float HAS_MSG_DELTA_IN_TRACK_SCALE_FACTOR = 0.008;     // HAS SIS ICD 1.0 Table 25
constexpr float HAS_MSG_DELTA_CROSS_TRACK_SCALE_FACTOR = 0.008;  // HAS SIS ICD 1.0 Table 25
constexpr float HAS_MSG_DELTA_CLOCK_SCALE_FACTOR = 0.0025;       // HAS SIS ICD 1.0 Table 31
constexpr float HAS_MSG_CODE_BIAS_SCALE_FACTOR = 0.02;           // HAS SIS ICD 1.0 Table 37
constexpr float HAS_MSG_PHASE_BIAS_SCALE_FACTOR = 0.01;          // HAS SIS ICD 1.0 Table 40

constexpr uint16_t HAS_MSG_NUMBER_MAX_TOH = 3599;  // HAS SIS ICD 1.0 Table 13

constexpr uint8_t HAS_MSG_GPS_SYSTEM = 0;      // HAS SIS ICD 1.0 Table 18
constexpr uint8_t HAS_MSG_GALILEO_SYSTEM = 2;  // HAS SIS ICD 1.0 Table 18
constexpr uint8_t HAS_MSG_WRONG_SYSTEM = 255;

constexpr char GALILEO_CNAV_PREAMBLE[17] = "1011011101110000";  // HAS SIS ICD 1.0 Section 2.3.1

// HAS SIS ICD 1.0 Table 7
const std::pair<int32_t, int32_t> GALILEO_HAS_STATUS({1, 2});
const std::pair<int32_t, int32_t> GALILEO_HAS_RESERVED({3, 2});
const std::pair<int32_t, int32_t> GALILEO_HAS_MESSAGE_TYPE({5, 2});
const std::pair<int32_t, int32_t> GALILEO_HAS_MESSAGE_ID({7, 5});
const std::pair<int32_t, int32_t> GALILEO_HAS_MESSAGE_SIZE({12, 5});
const std::pair<int32_t, int32_t> GALILEO_HAS_MESSAGE_PAGE_ID({17, 8});

// HAS SIS ICD 1.0 Table 12
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_TOH({1, 12});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_MASK_FLAG({13, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_ORBIT_CORRECTION_FLAG({14, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_CLOCK_FULLSET_FLAG({15, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_CLOCK_SUBSET_FLAG({16, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_CODE_BIAS_FLAG({17, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_PHASE_BIAS_FLAG({18, 1});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_RESERVED({19, 4});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_MASK_ID({23, 5});
const std::pair<int32_t, int32_t> GALILEO_MT1_HEADER_IOD_SET_ID({28, 5});

// HAS SIS ICD v1.0 Table 20
const std::unordered_map<uint8_t, std::unordered_map<uint8_t, std::string>> HAS_SIGNAL_INDEX_TABLE = {
    {0, {
            {0, "L1 C/A"},
            {1, "Reserved"},
            {2, "Reserved"},
            {3, "L1C(D)"},
            {4, "L1C(P)"},
            {5, "L1C(D+P)"},
            {6, "L2 CM"},
            {7, "L2 CL"},
            {8, "L2 CM+CL"},
            {9, "L2 P"},
            {10, "Reserved"},
            {11, "L5 I"},
            {12, "L5 Q"},
            {13, "L5 I + L5 Q"},
            {14, "Reserved"},
            {15, "Reserved"},
        }},
    {2, {
            {0, "E1-B I/NAV OS"},
            {1, "E1-C"},
            {2, "E1-B + E1-C"},
            {3, "E5a-I F/NAV OS"},
            {4, "E5a-Q"},
            {5, "E5a-I+E5a-Q"},
            {6, "E5b-I I/NAV OS"},
            {7, "E5b-Q"},
            {8, "E5b-I+E5b-Q"},
            {9, "E5-I"},
            {10, "E5-Q"},
            {11, "E5-I + E5-Q"},
            {12, "E6-B C/NAV HAS"},
            {13, "E6-C"},
            {14, "E6-B + E6-C"},
            {15, "Reserved"},
        }}};

// HAS SIS ICD v1.0 Table 23
const std::unordered_map<uint8_t, uint16_t> HAS_VALIDITY_INTERVALS = {
    {0, 5},
    {1, 10},
    {2, 15},
    {3, 20},
    {4, 30},
    {5, 60},
    {6, 90},
    {7, 120},
    {8, 180},
    {9, 240},
    {10, 300},
    {11, 600},
    {12, 900},
    {13, 1800},
    {14, 3600}};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_CNAV_H
