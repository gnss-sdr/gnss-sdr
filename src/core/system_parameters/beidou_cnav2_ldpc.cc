/*!
 * \file beidou_cnav2_ldpc.cc
 * \brief B-CNAV2 LDPC(96,48) graph and decoder wrappers
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

#include "beidou_cnav2_ldpc.h"

const BeidouLdpcGraph& beidou_cnav2_ldpc_graph_96_48()
{
    static const BeidouLdpcGraph graph = []() {
        BeidouLdpcGraph out;
        if (!beidou_ldpc_init_graph(BEIDOU_CNAV2_LDPC_M, BEIDOU_CNAV2_LDPC_N,
                BEIDOU_CNAV2_LDPC_DC, BEIDOU_CNAV2_H48_96_INDEX,
                BEIDOU_CNAV2_H48_96_ELEMENT, BEIDOU_CNAV2_LDPC_M * BEIDOU_CNAV2_LDPC_DC, out))
            {
                return BeidouLdpcGraph{};
            }
        return out;
    }();
    return graph;
}

bool beidou_cnav2_ldpc_decode_96_48(const float* bit_llr, int32_t num_bits, uint8_t* info_bits288)
{
    return beidou_ldpc_decode(beidou_cnav2_ldpc_graph_96_48(), bit_llr, num_bits, info_bits288, nullptr, true);
}

bool beidou_cnav2_ldpc_decode_96_48_codeword(const float* bit_llr, int32_t num_bits, uint8_t* codeword_bits576)
{
    return beidou_ldpc_decode(beidou_cnav2_ldpc_graph_96_48(), bit_llr, num_bits, nullptr, codeword_bits576, true);
}
