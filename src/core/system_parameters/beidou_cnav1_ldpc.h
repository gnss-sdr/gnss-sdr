/*!
 * \file beidou_cnav1_ldpc.h
 * \brief B-CNAV1 NB-LDPC decoder (BDS-SIS-ICD-B1C-1.0 §6.2.2, Appendix)
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
#ifndef GNSS_SDR_BEIDOU_CNAV1_LDPC_DECODER_H
#define GNSS_SDR_BEIDOU_CNAV1_LDPC_DECODER_H

#include <cstddef>
#include <cstdint>
#include <vector>

constexpr int32_t BEIDOU_CNAV1_LDPC_N200 = 200;
constexpr int32_t BEIDOU_CNAV1_LDPC_K200 = 100;
constexpr int32_t BEIDOU_CNAV1_LDPC_M200 = 100;
constexpr int32_t BEIDOU_CNAV1_LDPC_DC200 = 4;
constexpr int32_t BEIDOU_CNAV1_LDPC_N88 = 88;
constexpr int32_t BEIDOU_CNAV1_LDPC_K88 = 44;
constexpr int32_t BEIDOU_CNAV1_LDPC_M88 = 44;
constexpr int32_t BEIDOU_CNAV1_LDPC_DC88 = 4;
constexpr int32_t BEIDOU_CNAV1_LDPC_NM = 8;
constexpr int32_t BEIDOU_CNAV1_LDPC_MAX_ITER = 15;

/*
 * GF(2^6) LUTs follow BDS-SIS-ICD-B1C-1.0 Appendix mapping tables.
 * Primitive polynomial: p(x)=x^6+x+1.
 */
constexpr uint16_t BEIDOU_CNAV1_H100_200_INDEX[400] = {
    11, 62, 102, 150, 9, 60, 100, 148, 0, 51, 142, 197, 22, 80, 116, 154, 4, 90,
    131, 177, 47, 95, 138, 191, 51, 79, 146, 195, 44, 75, 142, 190, 13, 57, 135,
    198, 24, 65, 120, 173, 6, 88, 129, 179, 7, 89, 130, 176, 6, 58, 106, 158, 8,
    60, 108, 160, 44, 92, 139, 188, 4, 56, 104, 156, 10, 61, 101, 149, 39, 87, 123,
    168, 15, 67, 105, 167, 50, 78, 145, 194, 17, 98, 151, 187, 46, 94, 137, 190,
    14, 66, 104, 166, 7, 59, 107, 159, 21, 83, 119, 153, 31, 87, 114, 167, 2, 49,
    140, 199, 12, 64, 106, 164, 40, 53, 132, 159, 19, 96, 149, 185, 16, 68, 112,
    168, 14, 58, 132, 199, 34, 69, 125, 162, 23, 75, 119, 175, 42, 96, 144, 192,
    8, 63, 103, 151, 23, 81, 117, 155, 24, 93, 111, 182, 20, 72, 116, 172, 17, 69,
    113, 169, 34, 82, 130, 182, 1, 53, 101, 153, 46, 73, 140, 188, 13, 65, 107,
    165, 2, 54, 102, 154, 18, 70, 114, 170, 26, 67, 122, 175, 29, 77, 125, 177,
    36, 84, 120, 169, 25, 94, 108, 183, 39, 89, 137, 185, 21, 73, 117, 173, 28,
    76, 124, 176, 36, 90, 138, 186, 33, 68, 124, 161, 12, 56, 134, 197, 29, 85,
    112, 165, 45, 93, 136, 189, 27, 64, 123, 172, 28, 84, 115, 164, 25, 66, 121,
    174, 37, 85, 121, 170, 3, 50, 141, 196, 48, 76, 147, 192, 35, 70, 126, 163,
    32, 80, 128, 180, 0, 52, 100, 152, 43, 52, 135, 158, 35, 83, 131, 183, 10,
    62, 110, 162, 19, 71, 115, 171, 15, 59, 133, 196, 33, 81, 129, 181, 41, 54,
    133, 156, 20, 82, 118, 152, 38, 86, 122, 171, 30, 78, 126, 178, 9, 61, 109,
    161, 26, 95, 109, 180, 45, 72, 143, 191, 1, 48, 143, 198, 40, 98, 146, 194,
    18, 99, 148, 184, 5, 57, 105, 157, 41, 99, 147, 195, 31, 79, 127, 179, 3, 55,
    103, 155, 22, 74, 118, 174, 37, 91, 139, 187, 5, 91, 128, 178, 30, 86, 113,
    166, 43, 97, 145, 193, 16, 97, 150, 186, 11, 63, 111, 163, 32, 71, 127, 160,
    42, 55, 134, 157, 38, 88, 136, 184, 47, 74, 141, 189, 49, 77, 144, 193, 27,
    92, 110, 181};
constexpr uint8_t BEIDOU_CNAV1_H100_200_ELEMENT[400] = {
    35, 13, 51, 60, 1, 44, 53, 24, 1, 45, 15, 6, 45, 15, 6, 1, 1, 44, 53, 24, 1,
    45, 15, 6, 35, 46, 56, 15, 6, 1, 45, 15, 15, 6, 1, 45, 44, 53, 24, 1, 24, 1,
    44, 30, 1, 45, 15, 6, 30, 24, 1, 44, 24, 1, 44, 30, 45, 15, 6, 1, 17, 38, 49,
    11, 24, 1, 44, 30, 24, 1, 44, 53, 24, 1, 44, 53, 30, 24, 1, 44, 33, 42, 14,
    24, 33, 42, 14, 24, 45, 15, 6, 1, 1, 45, 15, 6, 30, 24, 1, 44, 24, 1, 44, 53,
    1, 44, 30, 24, 57, 25, 9, 41, 1, 45, 15, 6, 1, 45, 15, 6, 42, 36, 12, 57, 6,
    1, 45, 15, 24, 1, 44, 53, 24, 1, 44, 30, 1, 45, 15, 6, 1, 45, 15, 6, 44, 53,
    24, 1, 30, 24, 1, 44, 1, 44, 30, 24, 53, 24, 1, 44, 1, 44, 53, 24, 27, 28,
    30, 31, 53, 24, 1, 44, 24, 1, 44, 30, 45, 15, 6, 1, 30, 24, 1, 44, 1, 45, 15,
    6, 26, 22, 14, 2, 35, 13, 18, 60, 45, 15, 6, 1, 30, 1, 44, 7, 6, 1, 45, 15,
    6, 1, 45, 15, 53, 24, 1, 44, 24, 1, 44, 53, 30, 24, 1, 44, 1, 44, 30, 24, 44,
    53, 24, 1, 53, 24, 1, 44, 44, 30, 24, 1, 30, 24, 1, 44, 1, 44, 30, 24, 1, 44,
    30, 24, 41, 16, 29, 51, 1, 44, 30, 24, 38, 23, 22, 7, 44, 53, 24, 1, 1, 45,
    15, 6, 30, 24, 1, 44, 53, 24, 1, 44, 6, 1, 45, 15, 24, 1, 44, 53, 35, 46, 56,
    15, 5, 33, 42, 14, 54, 7, 38, 23, 1, 45, 15, 6, 44, 30, 24, 1, 6, 1, 45, 15,
    53, 24, 1, 44, 44, 53, 24, 1, 1, 44, 53, 24, 1, 44, 30, 24, 44, 30, 24, 1, 1,
    44, 53, 24, 45, 15, 6, 1, 6, 1, 45, 15, 1, 44, 53, 24, 42, 47, 37, 32, 51, 60,
    35, 13, 29, 28, 30, 31, 6, 1, 45, 15, 24, 1, 44, 53, 44, 53, 24, 1, 44, 30,
    24, 1, 38, 49, 11, 17, 44, 30, 24, 1, 24, 1, 44, 30, 24, 1, 44, 30, 1, 44, 53,
    24, 53, 24, 1, 44};
constexpr uint16_t BEIDOU_CNAV1_H44_88_INDEX[176] = {
    14, 35, 56, 70, 11, 29, 55, 73, 13, 39, 53, 69, 15, 34, 57, 71, 1, 27, 45, 54,
    23, 41, 63, 87, 2, 20, 46, 68, 6, 24, 50, 61, 2, 26, 61, 79, 9, 33, 59, 77, 4,
    30, 48, 74, 22, 42, 59, 76, 12, 38, 52, 68, 23, 43, 58, 77, 19, 21, 63, 64, 11,
    25, 65, 82, 17, 39, 44, 75, 9, 35, 49, 72, 19, 29, 66, 84, 13, 36, 56, 82, 17,
    43, 67, 81, 22, 40, 62, 86, 3, 21, 47, 69, 10, 24, 64, 83, 0, 37, 70, 86, 5, 31,
    49, 75, 4, 40, 53, 84, 5, 41, 52, 85, 18, 28, 67, 85, 0, 26, 44, 55, 10, 28, 54,
    72, 7, 30, 50, 81, 1, 36, 71, 87, 16, 38, 45, 74, 8, 34, 48, 73, 8, 32, 58, 76,
    12, 37, 57, 83, 6, 31, 51, 80, 15, 33, 47, 79, 16, 42, 66, 80, 7, 25, 51, 60, 3,
    27, 60, 78, 14, 32, 46, 78, 18, 20, 62, 65};
constexpr uint8_t BEIDOU_CNAV1_H44_88_ELEMENT[176] = {
    30, 24, 1, 44, 24, 1, 44, 30, 40, 32, 61, 18, 53, 24, 1, 44, 51, 60, 35, 13, 18,
    15, 32, 61, 15, 6, 1, 45, 30, 24, 1, 44, 6, 1, 45, 15, 45, 15, 6, 1, 1, 45, 15,
    6, 1, 44, 53, 24, 24, 1, 44, 53, 44, 30, 24, 1, 34, 33, 45, 36, 55, 9, 34, 3,
    1, 44, 53, 24, 61, 47, 20, 8, 53, 24, 1, 44, 15, 6, 1, 45, 13, 18, 60, 35, 45,
    15, 6, 1, 24, 1, 44, 53, 37, 32, 52, 47, 44, 53, 24, 1, 39, 36, 34, 33, 44, 35,
    31, 50, 12, 25, 36, 14, 15, 35, 46, 56, 53, 24, 1, 44, 1, 44, 53, 24, 24, 1, 44,
    30, 44, 30, 24, 1, 15, 6, 1, 45, 30, 24, 1, 44, 2, 50, 22, 14, 33, 42, 14, 5, 34,
    3, 55, 9, 44, 35, 61, 50, 15, 6, 1, 45, 45, 15, 6, 1, 1, 44, 30, 24, 6, 1, 45,
    15, 1, 44, 53, 24};
constexpr uint8_t BEIDOU_CNAV1_GF64_EXP[64] = {
    1, 2, 4, 8, 16, 32, 3, 6, 12, 24, 48, 35, 5, 10, 20, 40, 19, 38, 15, 30, 60, 59, 53,
    41, 17, 34, 7, 14, 28, 56, 51, 37, 9, 18, 36, 11, 22, 44, 27, 54, 47, 29, 58, 55,
    45, 25, 50, 39, 13, 26, 52, 43, 21, 42, 23, 46, 31, 62, 63, 61, 57, 49, 33, 1};
constexpr int8_t BEIDOU_CNAV1_GF64_LOG[64] = {
    0, 0, 1, 6, 2, 12, 7, 26, 3, 32, 13, 35, 8, 48, 27, 18, 4, 24, 33, 16, 14, 52, 36,
    54, 9, 45, 49, 38, 28, 41, 19, 56, 5, 62, 25, 11, 34, 31, 17, 47, 15, 23, 53, 51,
    37, 44, 55, 40, 10, 61, 46, 30, 50, 22, 39, 43, 29, 60, 42, 21, 20, 59, 57, 58};

bool beidou_cnav1_ldpc_decode_200_100(const float* symbol_llr, int32_t num_bits, uint8_t* info_bits600);
bool beidou_cnav1_ldpc_decode_88_44(const float* symbol_llr, int32_t num_bits, uint8_t* info_bits264);
bool beidou_cnav1_ldpc_decode_200_100_codeword(const float* symbol_llr, int32_t num_bits, uint8_t* codeword_bits1200);
bool beidou_cnav1_ldpc_decode_88_44_codeword(const float* symbol_llr, int32_t num_bits, uint8_t* codeword_bits528);

namespace GaloisField64
{
constexpr uint8_t kZero = 0U;
constexpr uint8_t kOrder = 63U;
constexpr uint8_t kFieldSize = 64U;

// LUT values come from BDS-SIS-ICD-B1C-1.0 Appendix GF(2^6) mapping rules.
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
    const int32_t log_sum = static_cast<int32_t>(BEIDOU_CNAV1_GF64_LOG[a]) +
                            static_cast<int32_t>(BEIDOU_CNAV1_GF64_LOG[b]);
    return BEIDOU_CNAV1_GF64_EXP[log_sum % kOrder];
}

inline uint8_t inv(uint8_t a)
{
    if (!valid_symbol(a) || a == kZero)
        {
            return kZero;
        }
    const int32_t exponent = static_cast<int32_t>(kOrder) - static_cast<int32_t>(BEIDOU_CNAV1_GF64_LOG[a]);
    return BEIDOU_CNAV1_GF64_EXP[exponent % kOrder];
}
}  // namespace GaloisField64

struct BeidouCnav1LdpcGraph
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

bool beidou_cnav1_ldpc_init_graph(
    int32_t num_checks,
    int32_t num_variables,
    int32_t row_weight,
    const uint16_t* icd_index,
    const uint8_t* icd_element,
    int32_t num_entries,
    BeidouCnav1LdpcGraph& graph);

const BeidouCnav1LdpcGraph& beidou_cnav1_ldpc_graph_200_100();
const BeidouCnav1LdpcGraph& beidou_cnav1_ldpc_graph_88_44();

#endif  // GNSS_SDR_BEIDOU_CNAV1_LDPC_DECODER_H
