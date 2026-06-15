/*!
 * \file ldpc_min_sum_decoder.h
 * \brief Class that implements a LDPC Min-Sum decoder with attenuation.
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
 *
 * Min-Sum with attenuation decoder for LDPC algorithm.
 *
 * The system can be visualized via the Tanner graph as two set of points laid
 * out horizontally, on top of the page situate the CNs, as many as H has rows,
 * on the bottom situate the VNs, as many as H has columns,
 * and connect CN_i to VN_j if H_ij is 1, where H is the LDPC matrix.
 *
 * Example for H = [1 0 0 0; 0 1 0 0; 0 1 1 1]:
 * CN1    CN2    CN3        ...
 *  |      |   /  |  \
 * VN1    VN2    VN3    VN4 ...
 *
 * Source: "Channel Codes: Classical and Modern", William E. Ryan and Shu Lin, 2009,
 * chapter 5, the algorithm is the Min-Sum with attenuation decoder.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_LDPC_MIN_SUM_DECODER_H
#define GNSS_SDR_LDPC_MIN_SUM_DECODER_H

#include <cstdint>
#include <optional>
#include <vector>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_libs
 * Utilities for the decoding of GNSS navigation messages.
 * \{ */

class LDPC_Min_Sum_Decoder
{
private:
    // VN -> CN edges values, messages going up from VNs to CNs,
    // where d_vn2cn_edges[e] is the message associated to edge "e"
    std::vector<float> d_vn2cn_edges;
    // CN -> VN edges values, messages going down from CNs to VNS,
    // where d_cn2vn_edges[e] is the message associated with the same
    // edge "i" as d_vn2cn_edges.
    std::vector<float> d_cn2vn_edges;

    // d_cn_neighbors[e] contains the VN connected to edge "e".
    std::vector<uint16_t> d_cn_neighbors;

    // e0=d_cn_row_ptr[i] and e1=d_cn_row_ptr[i+1] give the interval [e0, e1),
    // for the list of edges connected to CN i.
    // This vector has as many entries as rows in LDPC matrix (number of decoded bits) + 1
    std::vector<uint16_t> d_cn_row_ptr;

    // d_permutation[p] is a list of edge indices, see d_vn_col_ptr
    std::vector<uint16_t> d_permutation;

    // p0=d_vn_col_ptr[j] and p1=d_vn_col_ptr[j+1] give the interval [p0, p1),
    // for the list of entries in d_permutation associated to VM j.
    // This vector has as many entries as columns in LDPC matrix (number of encoded bits) + 1
    std::vector<uint16_t> d_vn_col_ptr;

    // As many as columns in the LDPC matrix (i.e. number of encoded bits)
    std::vector<float> d_inputs;

    std::vector<uint16_t> process_hex_str(const char* str, size_t str_len, bool hex_str_16_bit);

    // At each VN, for the output to CN j, we compute the sum of all inputs from connected
    // CNs != j, including the input array "inputs".
    void process_vn();

    // At each CN, for the output to VN j, we compute the product of the sign of all inputs
    // from connected VNs != j, and scale the result by the minimum input absolute magnitude.
    // c_atten is the attenuation parameter
    void process_cn(float c_atten);

    // At each VN, sums all the inputs on its edges, and the input array, and forms the hard
    // decision, 1 if the sum is < 0, 0 otherwise. (I.e. bit is likely 0 if the sum is large positive).
    std::vector<int> hard_decision();

    // Returns true if the hard decision "data" satisfies all parity constraints
    bool check_syndrome(const std::vector<int>& data);

public:
    // Returns the inputs array for data to be read into it
    std::vector<float>& get_inputs();

    // Clears all internal state, excluding inputs and LUTs. Must be called after
    // setting the inputs!
    void prepare_iteration();

    // Returns empty vector if not successful
    std::vector<int> run_decoder(size_t max_iterations, float c_atten);


    LDPC_Min_Sum_Decoder(
        const char* cn_neighbors, size_t cn_neighbors_len,
        const char* cn_row_ptr, size_t cn_row_ptr_len,
        const char* vn_col_ptr, size_t vn_col_ptr_len,
        const char* permutation, size_t permutation_len,
        bool hex_str_16_bit);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_LDPC_MIN_SUM_DECODER_H
