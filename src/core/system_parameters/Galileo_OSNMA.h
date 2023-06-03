/*!
 * \file Galileo_OSNMA.h
 * \brief Galileo OSNMA mesage constants
 * \author Carles Fernandez, 2023. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_OSNMA_H
#define GNSS_SDR_GALILEO_OSNMA_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

constexpr size_t SIZE_DSM_BLOCKS_BYTES = 13;

// OSNMA User ICD for the Test Phase, Issue 1.0, Table 2
const std::unordered_map<uint8_t, std::string> OSNMA_TABLE_2 = {
    {0, std::string("Reserved")},
    {1, std::string("Nominal")},
    {2, std::string("End of Chain (EOC)")},
    {3, std::string("Chain Revoked (CREV)")},
    {4, std::string("New Public Key (NPK)")},
    {5, std::string("Public Key Revoked (PKREV)")},
    {6, std::string("Reserved")},
    {7, std::string("Reserved")}};  // key: cpks, value: Chain and Public Key Status

// OSNMA User ICD for the Test Phase, Issue 1.0, Table 3
const std::unordered_map<uint8_t, std::pair<uint16_t, uint16_t>> OSNMA_TABLE_3 = {
    {0, {0, 0}},
    {1, {0, 0}},
    {2, {0, 0}},
    {3, {0, 0}},
    {4, {0, 0}},
    {5, {0, 0}},
    {6, {0, 0}},
    {7, {13, 1352}},
    {8, {14, 1456}},
    {9, {15, 1560}},
    {10, {16, 1664}},
    {11, {0, 0}},
    {12, {0, 0}},
    {13, {0, 0}},
    {14, {0, 0}},
    {15, {0, 0}}};  // key: nb_dp, value: {num_blocks, l_dp_bits}

const std::unordered_map<uint8_t, std::string> OSNMA_TABLE_5 = {
    {0, std::string("Reserved")},
    {1, std::string("ECDSA P-256")},
    {2, std::string("Reserved")},
    {3, std::string("ECDSA P-521")},
    {4, std::string("OAM")},
    {5, std::string("Reserved")},
    {6, std::string("Reserved")},
    {7, std::string("Reserved")},
    {8, std::string("Reserved")},
    {9, std::string("Reserved")},
    {10, std::string("Reserved")},
    {11, std::string("Reserved")},
    {12, std::string("Reserved")},
    {13, std::string("Reserved")},
    {14, std::string("Reserved")},
    {15, std::string("Reserved")}};  // key: nptk, value: message

const std::unordered_map<std::string, uint16_t> OSNMA_TABLE_6 = {
    {std::string("ECDSA P-256"), 264},
    {std::string("ECDSA P-521"), 536}};

// OSNMA User ICD for the Test Phase, Issue 1.0, Table 7
const std::unordered_map<uint8_t, std::pair<uint16_t, uint16_t>> OSNMA_TABLE_7 = {
    {0, {0, 0}},
    {1, {7, 728}},
    {2, {8, 832}},
    {3, {9, 936}},
    {4, {10, 1040}},
    {5, {11, 1144}},
    {6, {12, 1248}},
    {7, {13, 1352}},
    {8, {14, 1456}},
    {9, {0, 0}},
    {10, {0, 0}},
    {11, {0, 0}},
    {12, {0, 0}},
    {13, {0, 0}},
    {14, {0, 0}},
    {15, {0, 0}}};  // key: nb_dk, value: {num_blocks, l_dk_bits}

const std::unordered_map<uint8_t, std::string> OSNMA_TABLE_8 = {
    {0, std::string("SHA-256")},
    {1, std::string("Reserved")},
    {2, std::string("SHA3-256")},
    {3, std::string("Reserved")}};  // key: hs, value: hash_function

const std::unordered_map<uint8_t, uint16_t> OSNMA_TABLE_10 = {
    {0, 96},
    {1, 104},
    {2, 112},
    {3, 120},
    {4, 128},
    {5, 160},
    {6, 192},
    {7, 224},
    {8, 256},
    {9, 0},
    {10, 0},
    {11, 0},
    {12, 0},
    {13, 0},
    {15, 0},
    {15, 0}};  // key: ks, value: lk_bits

const std::unordered_map<uint8_t, uint8_t> OSNMA_TABLE_11 = {
    {0, 0},
    {1, 0},
    {2, 0},
    {3, 0},
    {4, 0},
    {5, 20},
    {6, 24},
    {7, 26},
    {8, 32},
    {9, 40},
    {10, 0},
    {11, 0},
    {12, 0},
    {13, 0},
    {14, 0},
    {15, 0},
};

const std::unordered_map<std::string, uint16_t> OSNMA_TABLE_15 = {
    {std::string("ECDSA P-256"), 512},
    {std::string("ECDSA P-521"), 1056},
    {std::string("SHA-256"), 512},
    {std::string("SHA-512"), 1056}};  // key: ECDSA Curve and hash function, value: {l_ds_bits}

/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_OSNMA_H