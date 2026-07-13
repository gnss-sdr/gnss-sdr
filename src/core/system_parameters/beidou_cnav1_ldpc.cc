/*!
 * \file beidou_cnav1_ldpc.cc
 * \brief B-CNAV1 LDPC graph and non-binary BP decoder over GF(64)
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
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace
{
constexpr int32_t BITS_PER_SYMBOL = 6;
constexpr int32_t Q = 64;
constexpr int32_t NM = BEIDOU_CNAV1_LDPC_NM;
constexpr int32_t FP_CANDIDATES = 8 + 2 * NM;
constexpr float LLR_FALLBACK_GAP = 25.0F;

struct TruncMessage
{
    std::array<uint8_t, NM> sym{};
    std::array<float, NM> llr{};
};

struct FixedPathCandidate
{
    uint8_t sym = 0;
    float llr = std::numeric_limits<float>::max();
};

uint8_t bits_to_symbol_msb_first(const float* bit_llr, int32_t bit_offset)
{
    uint8_t symbol = 0;
    for (int32_t bit = 0; bit < BITS_PER_SYMBOL; bit++)
        {
            symbol = static_cast<uint8_t>((symbol << 1U) | (bit_llr[bit_offset + bit] >= 0.0F ? 1U : 0U));
        }
    return symbol;
}

void normalize_message(TruncMessage& msg)
{
    const float min_llr = msg.llr[0];
    for (int32_t i = 0; i < NM; i++)
        {
            msg.llr[i] -= min_llr;
        }
}

float lookup_message_llr_var_domain(const TruncMessage& c2v_msg, uint8_t h_inv, uint8_t x_var)
{
    for (int32_t i = 0; i < NM; i++)
        {
            const uint8_t mapped = GaloisField64::mul(h_inv, c2v_msg.sym[i]);
            if (mapped == x_var)
                {
                    return c2v_msg.llr[i];
                }
        }
    return c2v_msg.llr[NM - 1] + LLR_FALLBACK_GAP;
}

void truncate_metric_vector(const std::array<float, Q>& metric, TruncMessage& out)
{
    std::array<bool, Q> used{};
    for (int32_t kept = 0; kept < NM; kept++)
        {
            float best_llr = std::numeric_limits<float>::max();
            int32_t best_sym = 0;
            for (int32_t x = 0; x < Q; x++)
                {
                    if (!used[static_cast<size_t>(x)] && metric[static_cast<size_t>(x)] < best_llr)
                        {
                            best_llr = metric[static_cast<size_t>(x)];
                            best_sym = x;
                        }
                }
            used[static_cast<size_t>(best_sym)] = true;
            out.sym[kept] = static_cast<uint8_t>(best_sym);
            out.llr[kept] = best_llr;
        }
    normalize_message(out);
}

uint8_t row_syndrome(const BeidouCnav1LdpcGraph& graph, int32_t check_index, const std::vector<uint8_t>& codeword)
{
    uint8_t syndrome = GaloisField64::kZero;
    const uint32_t begin = graph.check_offsets[static_cast<size_t>(check_index)];
    const uint32_t end = graph.check_offsets[static_cast<size_t>(check_index + 1)];
    for (uint32_t edge = begin; edge < end; edge++)
        {
            const uint16_t var = graph.check_to_var[edge];
            const uint8_t h_ij = graph.check_to_h[edge];
            syndrome = GaloisField64::add(syndrome, GaloisField64::mul(h_ij, codeword[var]));
        }
    return syndrome;
}

bool syndrome_is_zero(const BeidouCnav1LdpcGraph& graph, const std::vector<uint8_t>& codeword)
{
    for (int32_t row = 0; row < graph.num_checks; row++)
        {
            if (row_syndrome(graph, row, codeword) != GaloisField64::kZero)
                {
                    return false;
                }
        }
    return true;
}

void symbols_to_bits(const std::vector<uint8_t>& symbols, int32_t num_info_symbols, uint8_t* info_bits)
{
    for (int32_t symbol_index = 0; symbol_index < num_info_symbols; symbol_index++)
        {
            const uint8_t symbol = symbols[static_cast<size_t>(symbol_index)];
            for (int32_t bit = 0; bit < BITS_PER_SYMBOL; bit++)
                {
                    info_bits[symbol_index * BITS_PER_SYMBOL + bit] = static_cast<uint8_t>((symbol >> (5 - bit)) & 1U);
                }
        }
}

bool graph_is_valid(const BeidouCnav1LdpcGraph& graph)
{
    if (graph.num_checks <= 0 || graph.num_variables <= 0 || graph.row_weight <= 0)
        {
            return false;
        }
    const int32_t expected_edges = graph.num_checks * graph.row_weight;
    return graph.check_offsets.size() == static_cast<size_t>(graph.num_checks + 1) &&
           graph.var_offsets.size() == static_cast<size_t>(graph.num_variables + 1) &&
           graph.check_to_var.size() == static_cast<size_t>(expected_edges) &&
           graph.check_to_h.size() == static_cast<size_t>(expected_edges) &&
           graph.var_to_check.size() == static_cast<size_t>(expected_edges) &&
           graph.var_to_edge.size() == static_cast<size_t>(expected_edges) &&
           graph.var_to_h.size() == static_cast<size_t>(expected_edges) &&
           graph.var_to_h_inv.size() == static_cast<size_t>(expected_edges);
}

void initialize_channel_llr(
    const float* bit_llr,
    int32_t num_symbols,
    std::vector<uint8_t>& hard_symbol,
    std::vector<float>& channel_llr)
{
    hard_symbol.assign(static_cast<size_t>(num_symbols), 0U);
    channel_llr.assign(static_cast<size_t>(num_symbols) * Q, 0.0F);

    for (int32_t j = 0; j < num_symbols; j++)
        {
            const int32_t bit_offset = j * BITS_PER_SYMBOL;
            const uint8_t hard = bits_to_symbol_msb_first(bit_llr, bit_offset);
            hard_symbol[static_cast<size_t>(j)] = hard;
            std::array<float, BITS_PER_SYMBOL> abs_bit_llr{};
            for (int32_t b = 0; b < BITS_PER_SYMBOL; b++)
                {
                    abs_bit_llr[b] = std::fabs(bit_llr[bit_offset + b]);
                }
            for (int32_t x = 0; x < Q; x++)
                {
                    float metric = 0.0F;
                    const auto diff = static_cast<uint8_t>(x ^ hard);
                    for (int32_t b = 0; b < BITS_PER_SYMBOL; b++)
                        {
                            const auto bit_mask = static_cast<uint8_t>(1U << (BITS_PER_SYMBOL - 1 - b));
                            if ((diff & bit_mask) != 0U)
                                {
                                    metric += abs_bit_llr[b];
                                }
                        }
                    channel_llr[static_cast<size_t>(j) * Q + static_cast<size_t>(x)] = metric;
                }
        }
}

void initialize_v2c_messages(
    const BeidouCnav1LdpcGraph& graph,
    const std::vector<float>& channel_llr,
    std::vector<TruncMessage>& v2c)
{
    const int32_t edge_count = graph.num_checks * graph.row_weight;
    for (int32_t edge = 0; edge < edge_count; edge++)
        {
            const uint16_t var = graph.check_to_var[static_cast<size_t>(edge)];
            const uint8_t h_ij = graph.check_to_h[static_cast<size_t>(edge)];
            std::array<float, Q> mapped_metric{};
            for (int32_t x = 0; x < Q; x++)
                {
                    const uint8_t y = GaloisField64::mul(h_ij, static_cast<uint8_t>(x));
                    mapped_metric[static_cast<size_t>(y)] =
                        channel_llr[static_cast<size_t>(var) * Q + static_cast<size_t>(x)];
                }
            truncate_metric_vector(mapped_metric, v2c[static_cast<size_t>(edge)]);
        }
}

void variable_node_update(
    const BeidouCnav1LdpcGraph& graph,
    const std::vector<float>& channel_llr,
    std::vector<TruncMessage>& v2c,
    const std::vector<TruncMessage>& c2v,
    std::vector<uint8_t>& hard_codeword)
{
    std::array<float, Q> posterior{};
    std::array<float, Q> outgoing_metric{};

    for (int32_t var = 0; var < graph.num_variables; var++)
        {
            const uint32_t begin = graph.var_offsets[static_cast<size_t>(var)];
            const uint32_t end = graph.var_offsets[static_cast<size_t>(var + 1)];

            for (int32_t x = 0; x < Q; x++)
                {
                    float metric = channel_llr[static_cast<size_t>(var) * Q + static_cast<size_t>(x)];
                    for (uint32_t p = begin; p < end; p++)
                        {
                            const uint32_t edge = graph.var_to_edge[p];
                            const uint8_t h_fj_inv = graph.var_to_h_inv[p];
                            metric += lookup_message_llr_var_domain(c2v[edge], h_fj_inv, static_cast<uint8_t>(x));
                        }
                    posterior[static_cast<size_t>(x)] = metric;
                }

            int32_t hard_symbol = 0;
            float best_hard_llr = posterior[0];
            for (int32_t x = 1; x < Q; x++)
                {
                    if (posterior[static_cast<size_t>(x)] < best_hard_llr)
                        {
                            best_hard_llr = posterior[static_cast<size_t>(x)];
                            hard_symbol = x;
                        }
                }
            hard_codeword[static_cast<size_t>(var)] = static_cast<uint8_t>(hard_symbol);

            for (uint32_t p = begin; p < end; p++)
                {
                    const uint32_t edge_out = graph.var_to_edge[p];
                    const uint8_t h_ij = graph.var_to_h[p];
                    for (int32_t y = 0; y < Q; y++)
                        {
                            outgoing_metric[static_cast<size_t>(y)] = std::numeric_limits<float>::max();
                        }
                    for (int32_t x = 0; x < Q; x++)
                        {
                            float metric = channel_llr[static_cast<size_t>(var) * Q + static_cast<size_t>(x)];
                            for (uint32_t q = begin; q < end; q++)
                                {
                                    const uint32_t edge_in = graph.var_to_edge[q];
                                    if (edge_in == edge_out)
                                        {
                                            continue;
                                        }
                                    const uint8_t h_fj_inv = graph.var_to_h_inv[q];
                                    metric += lookup_message_llr_var_domain(c2v[edge_in], h_fj_inv, static_cast<uint8_t>(x));
                                }
                            const uint8_t y_out = GaloisField64::mul(h_ij, static_cast<uint8_t>(x));
                            outgoing_metric[static_cast<size_t>(y_out)] = metric;
                        }
                    truncate_metric_vector(outgoing_metric, v2c[edge_out]);
                }
        }
}

void build_fixed_path_candidates(
    const std::array<TruncMessage, 4>& in,
    std::array<FixedPathCandidate, FP_CANDIDATES>& cand)
{
    for (auto& c : cand)
        {
            c.sym = 0U;
            c.llr = std::numeric_limits<float>::max();
        }

    const uint8_t s0 = in[0].sym[0];
    const uint8_t s1 = in[1].sym[0];
    const uint8_t s2 = in[2].sym[0];
    const uint8_t s3 = in[3].sym[0];
    const float r0 = in[0].llr[0];
    const float r1 = in[1].llr[0];
    const float r2 = in[2].llr[0];
    const float r3 = in[3].llr[0];
    const uint8_t base_sym = GaloisField64::add(GaloisField64::add(s0, s1), GaloisField64::add(s2, s3));
    const float base_llr = r0 + r1 + r2 + r3;

    int32_t idx = 0;
    cand[idx++] = {base_sym, base_llr};

    for (int32_t l = 0; l < 4; l++)
        {
            const uint8_t sym = GaloisField64::add(base_sym, GaloisField64::add(in[l].sym[0], in[l].sym[1]));
            const float llr = base_llr - in[l].llr[0] + in[l].llr[1];
            cand[idx++] = {sym, llr};
        }

    constexpr std::array<std::pair<int32_t, int32_t>, 6> pairs = {
        std::pair<int32_t, int32_t>{0, 1},
        {0, 2},
        {0, 3},
        {1, 2},
        {1, 3},
        {2, 3},
    };
    for (const auto& pair : pairs)
        {
            const int32_t a = pair.first;
            const int32_t b = pair.second;
            const uint8_t sym = GaloisField64::add(
                base_sym,
                GaloisField64::add(
                    GaloisField64::add(in[a].sym[0], in[a].sym[1]),
                    GaloisField64::add(in[b].sym[0], in[b].sym[1])));
            const float llr = base_llr - in[a].llr[0] - in[b].llr[0] + in[a].llr[1] + in[b].llr[1];
            cand[idx++] = {sym, llr};
        }

    for (int32_t rank = 2; rank < NM && idx < FP_CANDIDATES; rank++)
        {
            for (int32_t l = 0; l < 4 && idx < FP_CANDIDATES; l++)
                {
                    const uint8_t sym = GaloisField64::add(base_sym, GaloisField64::add(in[l].sym[0], in[l].sym[rank]));
                    const float llr = base_llr - in[l].llr[0] + in[l].llr[rank];
                    cand[idx++] = {sym, llr};
                }
        }
}

void fixed_path_check_update(
    const BeidouCnav1LdpcGraph& graph,
    const std::vector<TruncMessage>& v2c,
    std::vector<TruncMessage>& c2v)
{
    std::array<int32_t, 4> edge_idx{};
    std::array<int32_t, 4> order{};
    std::array<TruncMessage, 4> in_sorted{};
    std::array<FixedPathCandidate, FP_CANDIDATES> candidates{};
    std::array<uint8_t, FP_CANDIDATES> T{};
    std::array<uint8_t, FP_CANDIDATES> Tbar{};

    for (int32_t check = 0; check < graph.num_checks; check++)
        {
            const uint32_t begin = graph.check_offsets[static_cast<size_t>(check)];
            const uint32_t end = graph.check_offsets[static_cast<size_t>(check + 1)];
            if (static_cast<int32_t>(end - begin) != 4)
                {
                    continue;
                }
            for (int32_t l = 0; l < 4; l++)
                {
                    edge_idx[l] = static_cast<int32_t>(begin) + l;
                    order[l] = l;
                }
            std::sort(order.begin(), order.end(), [&](int32_t a, int32_t b) {
                return v2c[static_cast<size_t>(edge_idx[a])].llr[1] < v2c[static_cast<size_t>(edge_idx[b])].llr[1];
            });
            for (int32_t l = 0; l < 4; l++)
                {
                    in_sorted[l] = v2c[static_cast<size_t>(edge_idx[order[l]])];
                }

            build_fixed_path_candidates(in_sorted, candidates);

            int32_t theta = 0;
            int32_t beta = 1;
            float min2 = in_sorted[0].llr[1];
            float second2 = in_sorted[1].llr[1];
            if (second2 < min2)
                {
                    std::swap(min2, second2);
                    std::swap(theta, beta);
                }
            for (int32_t l = 2; l < 4; l++)
                {
                    const float v = in_sorted[l].llr[1];
                    if (v < min2)
                        {
                            second2 = min2;
                            beta = theta;
                            min2 = v;
                            theta = l;
                        }
                    else if (v < second2)
                        {
                            second2 = v;
                            beta = l;
                        }
                }
            const int32_t ref_idx = NM / 2;
            const float theta_ref = in_sorted[theta].llr[ref_idx];
            const float beta_ref = in_sorted[beta].llr[ref_idx];
            for (int32_t k = 0; k < FP_CANDIDATES; k++)
                {
                    T[k] = (candidates[k].llr <= theta_ref) ? 1U : 0U;
                    Tbar[k] = (candidates[k].llr <= beta_ref) ? 1U : 0U;
                }

            for (int32_t l = 0; l < 4; l++)
                {
                    TruncMessage out{};
                    int32_t filled = 0;
                    for (int32_t k = 0; k < FP_CANDIDATES && filled < NM; k++)
                        {
                            bool pass_gate = false;
                            if (l == 0)
                                {
                                    pass_gate = (T[k] != 0U);
                                }
                            else if (l == theta)
                                {
                                    pass_gate = (Tbar[k] != 0U);
                                }
                            else
                                {
                                    pass_gate = (T[k] != 0U) || (Tbar[k] != 0U);
                                }
                            if (!pass_gate)
                                {
                                    continue;
                                }
                            out.sym[filled] = GaloisField64::add(in_sorted[l].sym[0], candidates[k].sym);
                            out.llr[filled] = candidates[k].llr - in_sorted[l].llr[0];
                            filled++;
                        }
                    while (filled < NM)
                        {
                            out.sym[filled] = out.sym[filled - 1];
                            out.llr[filled] = out.llr[filled - 1] + 1.0F;
                            filled++;
                        }
                    normalize_message(out);
                    c2v[static_cast<size_t>(edge_idx[order[l]])] = out;
                }
        }
}

void reorder_icd_column_bands_to_check_rows(
    int32_t num_checks,
    int32_t row_weight,
    int32_t num_entries,
    const uint16_t* icd_index,
    const uint8_t* icd_element,
    std::vector<uint16_t>& reordered_index,
    std::vector<uint8_t>& reordered_element)
{
    // ICD publishes a compact matrix with 4-column bands; decoding should read each band
    // top-to-bottom ("按栏读取"), then move left-to-right to the next band.
    const int32_t base_rows = num_checks / row_weight;
    const int32_t cols_per_base_row = row_weight * row_weight;

    reordered_index.assign(static_cast<size_t>(num_entries), 0U);
    reordered_element.assign(static_cast<size_t>(num_entries), 0U);

    for (int32_t band = 0; band < row_weight; band++)
        {
            for (int32_t row = 0; row < base_rows; row++)
                {
                    const int32_t src_offset = row * cols_per_base_row + band * row_weight;
                    const int32_t check_row = band * base_rows + row;
                    const int32_t dst_offset = check_row * row_weight;
                    for (int32_t k = 0; k < row_weight; k++)
                        {
                            reordered_index[static_cast<size_t>(dst_offset + k)] =
                                icd_index[static_cast<size_t>(src_offset + k)];
                            reordered_element[static_cast<size_t>(dst_offset + k)] =
                                icd_element[static_cast<size_t>(src_offset + k)];
                        }
                }
        }
}

bool decode_block(
    const BeidouCnav1LdpcGraph& graph,
    const float* bit_llr,
    int32_t num_bits,
    uint8_t* info_bits,
    uint8_t* codeword_bits)
{
    if (!graph_is_valid(graph))
        {
            return false;
        }

    const int32_t n = graph.num_variables;
    const int32_t k = n - graph.num_checks;
    if (num_bits < n * BITS_PER_SYMBOL)
        {
            return false;
        }

    std::vector<uint8_t> hard_codeword(static_cast<size_t>(n), 0U);
    std::vector<uint8_t> hard_symbol;
    std::vector<float> channel_llr;
    initialize_channel_llr(bit_llr, n, hard_symbol, channel_llr);
    hard_codeword = hard_symbol;

    const int32_t edge_count = graph.num_checks * graph.row_weight;
    std::vector<TruncMessage> v2c(static_cast<size_t>(edge_count));
    std::vector<TruncMessage> c2v(static_cast<size_t>(edge_count));
    initialize_v2c_messages(graph, channel_llr, v2c);
    c2v = v2c;

    for (int32_t iteration = 0; iteration < BEIDOU_CNAV1_LDPC_MAX_ITER; iteration++)
        {
            variable_node_update(graph, channel_llr, v2c, c2v, hard_codeword);
            if (syndrome_is_zero(graph, hard_codeword))
                {
                    if (info_bits != nullptr)
                        {
                            symbols_to_bits(hard_codeword, k, info_bits);
                        }
                    if (codeword_bits != nullptr)
                        {
                            symbols_to_bits(hard_codeword, n, codeword_bits);
                        }
                    return true;
                }
            fixed_path_check_update(graph, v2c, c2v);
        }

    return false;
}

bool init_graph_impl(
    int32_t num_checks,
    int32_t num_variables,
    int32_t row_weight,
    const uint16_t* icd_index,
    const uint8_t* icd_element,
    int32_t num_entries,
    BeidouCnav1LdpcGraph& graph)
{
    if (num_checks <= 0 || num_variables <= 0 || row_weight <= 0 || icd_index == nullptr || icd_element == nullptr)
        {
            return false;
        }
    if (num_entries != num_checks * row_weight)
        {
            return false;
        }
    if (num_checks % row_weight != 0)
        {
            return false;
        }

    const int32_t base_rows = num_checks / row_weight;
    const int32_t cols_per_base_row = row_weight * row_weight;
    if (base_rows * cols_per_base_row != num_entries)
        {
            return false;
        }

    std::vector<uint16_t> reordered_index;
    std::vector<uint8_t> reordered_element;
    reorder_icd_column_bands_to_check_rows(
        num_checks,
        row_weight,
        num_entries,
        icd_index,
        icd_element,
        reordered_index,
        reordered_element);
    const uint16_t* check_index = reordered_index.data();
    const uint8_t* check_element = reordered_element.data();

    graph.num_checks = num_checks;
    graph.num_variables = num_variables;
    graph.row_weight = row_weight;
    graph.check_offsets.assign(static_cast<size_t>(num_checks + 1), 0U);
    graph.check_to_var.assign(static_cast<size_t>(num_entries), 0U);
    graph.check_to_h.assign(static_cast<size_t>(num_entries), 0U);
    graph.var_offsets.assign(static_cast<size_t>(num_variables + 1), 0U);
    graph.var_to_check.assign(static_cast<size_t>(num_entries), 0U);
    graph.var_to_edge.assign(static_cast<size_t>(num_entries), 0U);
    graph.var_to_h.assign(static_cast<size_t>(num_entries), 0U);
    graph.var_to_h_inv.assign(static_cast<size_t>(num_entries), 0U);

    for (int32_t i = 0; i < num_checks; i++)
        {
            graph.check_offsets[static_cast<size_t>(i + 1)] =
                graph.check_offsets[static_cast<size_t>(i)] + static_cast<uint32_t>(row_weight);
        }

    std::vector<uint32_t> var_degree(static_cast<size_t>(num_variables), 0U);
    for (int32_t edge = 0; edge < num_entries; edge++)
        {
            const uint16_t var = check_index[edge];
            const uint8_t h_ij = check_element[edge];
            if (var >= static_cast<uint16_t>(num_variables) ||
                !GaloisField64::valid_symbol(h_ij) ||
                h_ij == GaloisField64::kZero)
                {
                    return false;
                }
            graph.check_to_var[static_cast<size_t>(edge)] = var;
            graph.check_to_h[static_cast<size_t>(edge)] = h_ij;
            var_degree[static_cast<size_t>(var)]++;
        }

    for (int32_t j = 0; j < num_variables; j++)
        {
            graph.var_offsets[static_cast<size_t>(j + 1)] =
                graph.var_offsets[static_cast<size_t>(j)] + var_degree[static_cast<size_t>(j)];
        }

    std::vector<uint32_t> cursor = graph.var_offsets;
    for (int32_t i = 0; i < num_checks; i++)
        {
            const uint32_t begin = graph.check_offsets[static_cast<size_t>(i)];
            const uint32_t end = graph.check_offsets[static_cast<size_t>(i + 1)];
            for (uint32_t edge = begin; edge < end; edge++)
                {
                    const uint16_t var = graph.check_to_var[edge];
                    const uint8_t h_ij = graph.check_to_h[edge];
                    const uint32_t pos = cursor[static_cast<size_t>(var)]++;
                    graph.var_to_check[pos] = static_cast<uint16_t>(i);
                    graph.var_to_edge[pos] = edge;
                    graph.var_to_h[pos] = h_ij;
                    graph.var_to_h_inv[pos] = GaloisField64::inv(h_ij);
                }
        }

    return true;
}
}  // namespace

bool beidou_cnav1_ldpc_init_graph(
    int32_t num_checks,
    int32_t num_variables,
    int32_t row_weight,
    const uint16_t* icd_index,
    const uint8_t* icd_element,
    int32_t num_entries,
    BeidouCnav1LdpcGraph& graph)
{
    return init_graph_impl(num_checks, num_variables, row_weight, icd_index, icd_element, num_entries, graph);
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
    return decode_block(beidou_cnav1_ldpc_graph_200_100(), symbol_llr, num_bits, info_bits600, nullptr);
}

bool beidou_cnav1_ldpc_decode_88_44(const float* symbol_llr, int32_t num_bits, uint8_t* info_bits264)
{
    return decode_block(beidou_cnav1_ldpc_graph_88_44(), symbol_llr, num_bits, info_bits264, nullptr);
}

bool beidou_cnav1_ldpc_decode_200_100_codeword(const float* symbol_llr, int32_t num_bits, uint8_t* codeword_bits1200)
{
    return decode_block(beidou_cnav1_ldpc_graph_200_100(), symbol_llr, num_bits, nullptr, codeword_bits1200);
}

bool beidou_cnav1_ldpc_decode_88_44_codeword(const float* symbol_llr, int32_t num_bits, uint8_t* codeword_bits528)
{
    return decode_block(beidou_cnav1_ldpc_graph_88_44(), symbol_llr, num_bits, nullptr, codeword_bits528);
}
