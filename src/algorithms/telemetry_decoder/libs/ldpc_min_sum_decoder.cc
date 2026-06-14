/*!
 * \file ldpc_min_sum_decoder.h
 * \brief Class that implements a LDPC Min-Sum decoder with attenuation.
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
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

#include "ldpc_min_sum_decoder.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <optional>


std::vector<float>& LDPC_Min_Sum_Decoder::get_inputs()
{
    return d_inputs;
}

void LDPC_Min_Sum_Decoder::process_vn()
{
    for (size_t j = 0; j < d_vn_col_ptr.size() - 1; j++)
        {
            const size_t p0 = d_vn_col_ptr[j];
            const size_t p1 = d_vn_col_ptr[j + 1];

            // First sum all "inbound" edges connected to VN
            float sum = d_inputs[j];
            for (size_t p = p0; p < p1; p++)
                {
                    const size_t e = d_permutation[p];
                    sum += d_cn2vn_edges[e];
                }

            // Each edge is then just sum - value on the same "inbound" edge
            for (size_t p = p0; p < p1; p++)
                {
                    const size_t e = d_permutation[p];
                    d_vn2cn_edges[e] = sum - d_cn2vn_edges[e];
                }
        }
}

void LDPC_Min_Sum_Decoder::process_cn(float c_atten)
{
    for (size_t i = 0; i < d_cn_row_ptr.size() - 1; i++)
        {
            const size_t e0 = d_cn_row_ptr[i];
            const size_t e1 = d_cn_row_ptr[i + 1];

            // First obtain product of all incoming edges and last two minimum magnitudes
            float alpha_prod = 1.0F;
            float min_beta = std::numeric_limits<float>::max();
            float second_min_beta = std::numeric_limits<float>::max();
            size_t min_beta_idx = std::numeric_limits<size_t>::max();
            for (size_t e = e0; e < e1; e++)
                {
                    const float val = d_vn2cn_edges[e];
                    if (std::signbit(val))
                        {
                            alpha_prod *= -1.0F;
                        }

                    const float abs = std::abs(val);

                    if (abs < min_beta)
                        {
                            second_min_beta = min_beta;
                            min_beta = abs;
                            min_beta_idx = e;
                        }
                    else if (abs < second_min_beta)
                        {
                            second_min_beta = abs;
                        }
                }

            // Each edge is then just the alpha_prod times itself (to undo its sign change)
            // and either min_beta or second_min_beta
            for (size_t e = e0; e < e1; e++)
                {
                    const float val = d_vn2cn_edges[e];
                    float edge = alpha_prod;
                    if (std::signbit(val))
                        {
                            edge *= -1.0F;
                        }

                    if (e == min_beta_idx)
                        {
                            edge *= second_min_beta;
                        }
                    else
                        {
                            edge *= min_beta;
                        }

                    d_cn2vn_edges[e] = edge * c_atten;
                }
        }
}

std::vector<int> LDPC_Min_Sum_Decoder::hard_decision()
{
    std::vector<int> out(d_vn_col_ptr.size() - 1);

    for (size_t j = 0; j < d_vn_col_ptr.size() - 1; j++)
        {
            const size_t p0 = d_vn_col_ptr[j];
            const size_t p1 = d_vn_col_ptr[j + 1];

            float sum = d_inputs[j];
            for (size_t p = p0; p < p1; p++)
                {
                    const size_t e = d_permutation[p];
                    sum += d_cn2vn_edges[e];
                }

            out[j] = sum < 0.0F ? 1 : 0;
        }

    return out;
}

bool LDPC_Min_Sum_Decoder::check_syndrome(const std::vector<int>& data)
{
    assert(data.size() == d_vn_col_ptr.size() - 1);

    // This is simply data * H^TT = 0 <=> H * data = 0, i.e. the sum (mod 2) of
    // every row in H multiplied by each entry in data is zero.
    for (size_t i = 0; i < d_cn_row_ptr.size() - 1; i++)
        {
            int sum = 0;
            const size_t e0 = d_cn_row_ptr[i];
            const size_t e1 = d_cn_row_ptr[i + 1];

            // Sum (mod 2) of each non-zero entry
            for (size_t e = e0; e < e1; e++)
                {
                    sum ^= data[d_cn_neighbors[e]];
                }

            if (sum != 0)
                {
                    return false;
                }
        }

    return true;
}

LDPC_Min_Sum_Decoder::LDPC_Min_Sum_Decoder(
    const char* cn_neighbors, size_t cn_neighbors_len,
    const char* cn_row_ptr, size_t cn_row_ptr_len,
    const char* vn_col_ptr, size_t vn_col_ptr_len,
    const char* permutation, size_t permutation_len,
    bool hex_str_16_bit)
{
    // Pre-parse all of the LUTs into the internal arrays
    d_cn_neighbors =
        process_hex_str(cn_neighbors, cn_neighbors_len, hex_str_16_bit);
    d_cn_row_ptr = process_hex_str(cn_row_ptr, cn_row_ptr_len, hex_str_16_bit);
    d_vn_col_ptr = process_hex_str(vn_col_ptr, vn_col_ptr_len, hex_str_16_bit);
    d_permutation = process_hex_str(permutation, permutation_len, hex_str_16_bit);

    // As many as we have VNs
    d_inputs.resize(d_vn_col_ptr.size() - 1);
    // As many as we have ones (edges)
    d_vn2cn_edges.resize(d_cn_neighbors.size());
    // As many as we have ones (edges)
    d_cn2vn_edges.resize(d_cn_neighbors.size());
}

// hex_to_binary_converter is unwieldy to use here
static uint16_t hex_to_val(char c)
{
    if (c >= '0' && c <= '9')
        {
            return c - '0';
        }
    if (c >= 'A' && c <= 'F')
        {
            return c - 'A' + 10;
        }
    if (c >= 'a' && c <= 'f')
        {
            return c - 'a' + 10;
        }

    return 0;
};

std::vector<uint16_t> LDPC_Min_Sum_Decoder::process_hex_str(const char* str, size_t str_len, bool hex_str_16_bit)
{
    size_t num_entries = str_len / 3;
    if (hex_str_16_bit)
        {
            num_entries = str_len / 4;
        }

    std::vector<uint16_t> out(num_entries);

    // The str contains densely packed big endian (written from MSB to LSB)
    // 12 bit data
    for (size_t i = 0; i < num_entries; i++)
        {
            out[i] = 0;
            if (hex_str_16_bit)
                {
                    out[i] = hex_to_val(str[i * 4 + 0]) << 12;
                    out[i] |= hex_to_val(str[i * 4 + 1]) << 8;
                    out[i] |= hex_to_val(str[i * 4 + 2]) << 4;
                    out[i] |= hex_to_val(str[i * 4 + 3]);
                }
            else
                {
                    out[i] = hex_to_val(str[i * 3 + 0]) << 8;
                    out[i] |= hex_to_val(str[i * 3 + 1]) << 4;
                    out[i] |= hex_to_val(str[i * 3 + 2]);
                }
        }

    return out;
}

void LDPC_Min_Sum_Decoder::prepare_iteration()
{
    // These are overwritten, but doesn't hurt
    for (float& d_cn2vn_edge : d_cn2vn_edges)
        {
            d_cn2vn_edge = 0.0F;
        }

    // Edges outbound from a VN inherit the input value there
    for (size_t j = 0; j < d_vn_col_ptr.size() - 1; j++)
        {
            const size_t p0 = d_vn_col_ptr[j];
            const size_t p1 = d_vn_col_ptr[j + 1];

            for (size_t p = p0; p < p1; p++)
                {
                    const size_t e = d_permutation[p];
                    d_vn2cn_edges[e] = d_inputs[j];
                }
        }
}

std::optional<std::vector<int>> LDPC_Min_Sum_Decoder::run_decoder(size_t max_iterations, float c_atten)
{
    for (size_t i = 0; i < max_iterations; i++)
        {
            process_cn(c_atten);
            process_vn();
            const std::vector<int>& llr_total = hard_decision();
            if (check_syndrome(llr_total))
                {
                    return llr_total;
                }
        }

    return std::nullopt;
}
