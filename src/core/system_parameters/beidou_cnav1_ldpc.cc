/*!
 * \file beidou_cnav1_ldpc.cc
 * \brief B-CNAV1 LDPC graphs and wrappers for the shared GF(64) decoder
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

#include "beidou_cnav1_ldpc.h"

bool beidou_cnav1_ldpc_init_graph(
    int32_t num_checks,
    int32_t num_variables,
    int32_t row_weight,
    const uint16_t* icd_index,
    const uint8_t* icd_element,
    int32_t num_entries,
    BeidouCnav1LdpcGraph& graph)
{
    return beidou_ldpc_init_graph(num_checks, num_variables, row_weight, icd_index, icd_element, num_entries, graph);
}


const BeidouCnav1LdpcGraph& beidou_cnav1_ldpc_graph_200_100()
{
    static const BeidouCnav1LdpcGraph graph = []() {
        BeidouCnav1LdpcGraph out;
        const bool ok = beidou_cnav1_ldpc_init_graph(
            BEIDOU_CNAV1_LDPC_M200,
            BEIDOU_CNAV1_LDPC_N200,
            BEIDOU_CNAV1_LDPC_DC200,
            BEIDOU_CNAV1_H100_200_INDEX,
            BEIDOU_CNAV1_H100_200_ELEMENT,
            BEIDOU_CNAV1_LDPC_M200 * BEIDOU_CNAV1_LDPC_DC200,
            out);
        if (!ok)
            {
                return BeidouCnav1LdpcGraph{};
            }
        return out;
    }();
    return graph;
}


const BeidouCnav1LdpcGraph& beidou_cnav1_ldpc_graph_88_44()
{
    static const BeidouCnav1LdpcGraph graph = []() {
        BeidouCnav1LdpcGraph out;
        const bool ok = beidou_cnav1_ldpc_init_graph(
            BEIDOU_CNAV1_LDPC_M88,
            BEIDOU_CNAV1_LDPC_N88,
            BEIDOU_CNAV1_LDPC_DC88,
            BEIDOU_CNAV1_H44_88_INDEX,
            BEIDOU_CNAV1_H44_88_ELEMENT,
            BEIDOU_CNAV1_LDPC_M88 * BEIDOU_CNAV1_LDPC_DC88,
            out);
        if (!ok)
            {
                return BeidouCnav1LdpcGraph{};
            }
        return out;
    }();
    return graph;
}


bool beidou_cnav1_ldpc_decode_200_100(const float* symbol_llr, int32_t num_bits, uint8_t* info_bits600)
{
    return beidou_ldpc_decode(beidou_cnav1_ldpc_graph_200_100(), symbol_llr, num_bits, info_bits600, nullptr);
}


bool beidou_cnav1_ldpc_decode_88_44(const float* symbol_llr, int32_t num_bits, uint8_t* info_bits264)
{
    return beidou_ldpc_decode(beidou_cnav1_ldpc_graph_88_44(), symbol_llr, num_bits, info_bits264, nullptr);
}


bool beidou_cnav1_ldpc_decode_200_100_codeword(const float* symbol_llr, int32_t num_bits, uint8_t* codeword_bits1200)
{
    return beidou_ldpc_decode(beidou_cnav1_ldpc_graph_200_100(), symbol_llr, num_bits, nullptr, codeword_bits1200);
}


bool beidou_cnav1_ldpc_decode_88_44_codeword(const float* symbol_llr, int32_t num_bits, uint8_t* codeword_bits528)
{
    return beidou_ldpc_decode(beidou_cnav1_ldpc_graph_88_44(), symbol_llr, num_bits, nullptr, codeword_bits528);
}
