/*!
 * \file beidou_cnav2_ldpc.h
 * \brief B-CNAV2 LDPC(96,48) decoder (BDS-SIS-ICD-B2a-1.0 section 6.2.2)
 * \author Carles Fernandez, 2026 cfernandez(at)cttc.cat
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

#ifndef GNSS_SDR_BEIDOU_CNAV2_LDPC_H
#define GNSS_SDR_BEIDOU_CNAV2_LDPC_H

#include "beidou_ldpc.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

constexpr int32_t BEIDOU_CNAV2_LDPC_N = 96;
constexpr int32_t BEIDOU_CNAV2_LDPC_K = 48;
constexpr int32_t BEIDOU_CNAV2_LDPC_M = 48;
constexpr int32_t BEIDOU_CNAV2_LDPC_DC = 4;

// ICD section 6.2.2, pp. 20-21: printed order, four 4-column bands.
// beidou_ldpc_init_graph reads each band top to bottom before the next band.
constexpr uint16_t BEIDOU_CNAV2_H48_96_INDEX[192] = {
    19, 46, 49, 76, 5, 29, 53, 71, 17, 30, 64, 72, 22, 36, 59, 82,
    22, 41, 68, 94, 20, 44, 54, 75, 9, 41, 61, 86, 6, 47, 60, 89,
    8, 40, 60, 87, 15, 26, 66, 81, 19, 24, 67, 95, 2, 26, 50, 72,
    5, 38, 70, 89, 16, 34, 64, 92, 21, 45, 55, 74, 0, 24, 48, 78,
    23, 37, 58, 83, 15, 43, 56, 91, 18, 47, 48, 77, 14, 42, 57, 90,
    6, 30, 54, 76, 14, 27, 67, 80, 17, 35, 65, 93, 7, 46, 61, 88,
    1, 25, 49, 79, 12, 45, 69, 79, 18, 25, 66, 94, 23, 40, 69, 95,
    8, 36, 51, 84, 3, 38, 56, 86, 0, 29, 62, 85, 2, 39, 57, 87,
    11, 33, 59, 81, 20, 43, 74, 93, 13, 32, 63, 91, 11, 35, 52, 83,
    16, 31, 65, 73, 4, 28, 52, 70, 1, 28, 63, 84, 12, 33, 62, 90,
    21, 42, 75, 92, 7, 31, 55, 77, 9, 37, 50, 85, 10, 34, 53, 82,
    4, 39, 71, 88, 13, 44, 68, 78, 3, 27, 51, 73, 10, 32, 58, 80};

constexpr uint8_t BEIDOU_CNAV2_H48_96_ELEMENT[192] = {
    1, 45, 15, 6, 1, 44, 53, 24, 45, 15, 6, 1, 30, 24, 1, 44,
    18, 15, 32, 61, 3, 55, 9, 34, 35, 31, 50, 44, 45, 15, 6, 1,
    24, 1, 44, 53, 30, 24, 1, 44, 32, 42, 47, 37, 6, 1, 45, 15,
    44, 53, 24, 1, 39, 36, 34, 33, 44, 53, 24, 1, 44, 53, 24, 1,
    45, 15, 6, 1, 6, 1, 45, 15, 24, 1, 44, 53, 9, 41, 57, 58,
    32, 61, 18, 40, 1, 45, 15, 6, 22, 14, 2, 50, 24, 1, 44, 30,
    30, 24, 1, 44, 15, 46, 45, 44, 45, 15, 6, 1, 1, 44, 30, 24,
    24, 1, 44, 53, 15, 6, 1, 45, 53, 24, 1, 44, 7, 38, 23, 54,
    1, 45, 15, 6, 44, 53, 24, 1, 57, 25, 9, 41, 35, 13, 51, 60,
    33, 45, 36, 34, 6, 1, 45, 15, 6, 1, 45, 15, 6, 1, 45, 15,
    44, 35, 31, 50, 26, 27, 37, 5, 24, 1, 44, 30, 33, 42, 14, 5,
    24, 1, 44, 30, 24, 1, 44, 30, 1, 44, 53, 24, 1, 44, 30, 24};

const BeidouLdpcGraph& beidou_cnav2_ldpc_graph_96_48();

// Input: 576 bit LLRs (positive favors 1); output: 288 MSB-first information bits.
bool beidou_cnav2_ldpc_decode_96_48(const float* bit_llr, int32_t num_bits, uint8_t* info_bits288);
bool beidou_cnav2_ldpc_decode_96_48_codeword(const float* bit_llr, int32_t num_bits, uint8_t* codeword_bits576);

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_CNAV2_LDPC_H
