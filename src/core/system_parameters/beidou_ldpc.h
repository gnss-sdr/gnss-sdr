/*!
 * \file beidou_ldpc.h
 * \brief Shared BeiDou B-CNAV1/B-CNAV2 non-binary LDPC decoder over GF(64)
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
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

#ifndef GNSS_SDR_BEIDOU_LDPC_H
#define GNSS_SDR_BEIDOU_LDPC_H

#include <cstddef>
#include <cstdint>
#include <vector>

constexpr int32_t BEIDOU_LDPC_NM = 8;
constexpr int32_t BEIDOU_LDPC_MAX_ITER = 15;

// B1C and B2a use p(x) = x^6 + x + 1, with MSB-first vector representation.
constexpr uint8_t BEIDOU_GF64_EXP[64] = {
    1, 2, 4, 8, 16, 32, 3, 6, 12, 24, 48, 35, 5, 10, 20, 40, 19, 38, 15, 30, 60, 59, 53,
    41, 17, 34, 7, 14, 28, 56, 51, 37, 9, 18, 36, 11, 22, 44, 27, 54, 47, 29, 58, 55,
    45, 25, 50, 39, 13, 26, 52, 43, 21, 42, 23, 46, 31, 62, 63, 61, 57, 49, 33, 1};


constexpr int8_t BEIDOU_GF64_LOG[64] = {
    0, 0, 1, 6, 2, 12, 7, 26, 3, 32, 13, 35, 8, 48, 27, 18, 4, 24, 33, 16, 14, 52, 36,
    54, 9, 45, 49, 38, 28, 41, 19, 56, 5, 62, 25, 11, 34, 31, 17, 47, 15, 23, 53, 51,
    37, 44, 55, 40, 10, 61, 46, 30, 50, 22, 39, 43, 29, 60, 42, 21, 20, 59, 57, 58};

namespace GaloisField64
{
constexpr uint8_t kZero = 0U;
constexpr uint8_t kOrder = 63U;
constexpr uint8_t kFieldSize = 64U;

// LUT values follow the B1C and B2a ICD Annex GF(2^6) mapping rules.
inline bool valid_symbol(uint8_t x)
{
    return x < kFieldSize;
}


inline uint8_t add(uint8_t a, uint8_t b)
{
    if (!valid_symbol(a) || !valid_symbol(b))
        {
            return kZero;
        }
    return static_cast<uint8_t>(a ^ b);
}


inline uint8_t mul(uint8_t a, uint8_t b)
{
    if (!valid_symbol(a) || !valid_symbol(b) || a == kZero || b == kZero)
        {
            return kZero;
        }
    const int32_t log_sum = static_cast<int32_t>(BEIDOU_GF64_LOG[a]) +
                            static_cast<int32_t>(BEIDOU_GF64_LOG[b]);
    return BEIDOU_GF64_EXP[log_sum % kOrder];
}


inline uint8_t inv(uint8_t a)
{
    if (!valid_symbol(a) || a == kZero)
        {
            return kZero;
        }
    const int32_t exponent = static_cast<int32_t>(kOrder) - static_cast<int32_t>(BEIDOU_GF64_LOG[a]);
    return BEIDOU_GF64_EXP[exponent % kOrder];
}
}  // namespace GaloisField64


struct BeidouLdpcGraph
{
    int32_t num_checks = 0;
    int32_t num_variables = 0;
    int32_t row_weight = 0;

    // Check-node (CSR-like) adjacency: [check_offsets[i], check_offsets[i+1]).
    std::vector<uint32_t> check_offsets;
    std::vector<uint16_t> check_to_var;
    std::vector<uint8_t> check_to_h;

    // Variable-node (CSC-like) adjacency: [var_offsets[j], var_offsets[j+1]).
    std::vector<uint32_t> var_offsets;
    std::vector<uint16_t> var_to_check;
    std::vector<uint32_t> var_to_edge;
    std::vector<uint8_t> var_to_h;
    std::vector<uint8_t> var_to_h_inv;
};


// ICD tables are laid out in four bands of four entries per printed row.
// Only row weight four is supported by the fixed-path check update.
bool beidou_ldpc_init_graph(
    int32_t num_checks,
    int32_t num_variables,
    int32_t row_weight,
    const uint16_t* icd_index,
    const uint8_t* icd_element,
    int32_t num_entries,
    BeidouLdpcGraph& graph);


// Positive bit LLRs favor 1. Bits within each GF(64) symbol are MSB first.
// Supply at least one output: n-m information symbols or all n codeword symbols.
// Outputs are written only after the parity checks pass; CRC validation is separate.
// The optional full-alphabet retry adds up to 15 iterations after fixed-path failure.
// Disabled by default to preserve the existing B-CNAV1 decoding behavior.
bool beidou_ldpc_decode(const BeidouLdpcGraph& graph, const float* bit_llr,
    int32_t num_bits, uint8_t* info_bits, uint8_t* codeword_bits, bool enable_sum_product = false);

#endif  // GNSS_SDR_BEIDOU_LDPC_H
