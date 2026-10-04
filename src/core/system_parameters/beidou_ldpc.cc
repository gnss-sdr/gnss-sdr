/*!
 * \file beidou_ldpc.cc
 * \brief Shared BeiDou LDPC graph and non-binary BP decoder over GF(64)
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
#include "beidou_ldpc.h"
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
constexpr int32_t NM = BEIDOU_LDPC_NM;
// LLR assigned to field elements missing from a truncated C2V message is its
// largest kept LLR plus this offset (ICD Annex, EMS variable node rule).
// Tuned by simulation for bit LLRs on the 2|y|/sigma^2 scale (B-CNAV1
// LDPC(200,100) over AWGN, 1.5 to 3 dB Eb/N0).
constexpr float EMS_TRUNCATION_OFFSET = 2.0F;

struct TruncMessage
{
    std::array<uint8_t, NM> sym{};
    std::array<float, NM> llr{};
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


// Keep the NM most reliable (smallest-LLR) distinct field elements of a
// full-alphabet metric vector, in ascending order, with the first LLR at zero.
void truncate_metric_vector(const std::array<float, Q>& metric, TruncMessage& out)
{
    int32_t kept = 0;
    for (int32_t x = 0; x < Q; x++)
        {
            const float llr = metric[static_cast<size_t>(x)];
            if (kept == NM && llr >= out.llr[NM - 1])
                {
                    continue;
                }
            int32_t pos = (kept < NM) ? kept++ : NM - 1;
            while (pos > 0 && out.llr[pos - 1] > llr)
                {
                    out.llr[pos] = out.llr[pos - 1];
                    out.sym[pos] = out.sym[pos - 1];
                    pos--;
                }
            out.llr[pos] = llr;
            out.sym[pos] = static_cast<uint8_t>(x);
        }
    const float min_llr = out.llr[0];
    for (auto& llr : out.llr)
        {
            llr -= min_llr;
        }
}


uint8_t row_syndrome(const BeidouLdpcGraph& graph, int32_t check_index, const std::vector<uint8_t>& codeword)
{
    uint8_t syndrome = GaloisField64::kZero;
    const uint32_t begin = graph.check_offsets[static_cast<size_t>(check_index)];
    const uint32_t end = graph.check_offsets[static_cast<size_t>(check_index) + 1];
    for (uint32_t edge = begin; edge < end; edge++)
        {
            const uint16_t var = graph.check_to_var[edge];
            const uint8_t h_ij = graph.check_to_h[edge];
            syndrome = GaloisField64::add(syndrome, GaloisField64::mul(h_ij, codeword[var]));
        }
    return syndrome;
}


bool syndrome_is_zero(const BeidouLdpcGraph& graph, const std::vector<uint8_t>& codeword)
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


bool graph_is_valid(const BeidouLdpcGraph& graph)
{
    if (graph.num_checks <= 0 || graph.num_variables <= graph.num_checks || graph.row_weight != 4)
        {
            return false;
        }
    const int32_t expected_edges = graph.num_checks * graph.row_weight;
    return graph.check_offsets.size() == static_cast<size_t>(graph.num_checks) + 1 &&
           graph.var_offsets.size() == static_cast<size_t>(graph.num_variables) + 1 &&
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


// Extended Min-Sum (ICD Annex, section 2.(1)).
// V2C/C2V messages live in the edge domain y = h_ij * x_j, so a check node
// only needs GF(64) additions of the incoming symbols.
void initialize_v2c_messages(
    const BeidouLdpcGraph& graph,
    const std::vector<float>& channel_llr,
    std::vector<TruncMessage>& v2c)
{
    const int32_t edge_count = graph.num_checks * graph.row_weight;
    std::array<float, Q> mapped_metric{};
    for (int32_t edge = 0; edge < edge_count; edge++)
        {
            const uint16_t var = graph.check_to_var[static_cast<size_t>(edge)];
            const uint8_t h_ij = graph.check_to_h[static_cast<size_t>(edge)];
            for (int32_t x = 0; x < Q; x++)
                {
                    const uint8_t y = GaloisField64::mul(h_ij, static_cast<uint8_t>(x));
                    mapped_metric[static_cast<size_t>(y)] =
                        channel_llr[static_cast<size_t>(var) * Q + static_cast<size_t>(x)];
                }
            truncate_metric_vector(mapped_metric, v2c[static_cast<size_t>(edge)]);
        }
}


// Elementary check-node operation V = U (+) W: every pair (d, delta) proposes
// the element Us[d] + Ws[delta] with LLR U[d] + W[delta]; keep the NM
// smallest distinct elements. With NM = 8 the full NM x NM matrix is cheap,
// so this is exact rather than a bubble-sort approximation.
void ems_combine(const TruncMessage& u, const TruncMessage& w, TruncMessage& out)
{
    std::array<float, Q> best{};
    best.fill(std::numeric_limits<float>::max());
    for (int32_t d = 0; d < NM; d++)
        {
            for (int32_t delta = 0; delta < NM; delta++)
                {
                    const auto sym = static_cast<size_t>(u.sym[d] ^ w.sym[delta]);
                    const float llr = u.llr[d] + w.llr[delta];
                    if (llr < best[sym])
                        {
                            best[sym] = llr;
                        }
                }
        }
    // Fixing delta = 0 already yields NM distinct elements, so NM finite entries exist.
    truncate_metric_vector(best, out);
}


// C2V_{i->j} = (+)_{j' in N(i), j' != j} V2C_{j'->i}, by forward-backward recursion.
void ems_check_update(
    const BeidouLdpcGraph& graph,
    const std::vector<TruncMessage>& v2c,
    std::vector<TruncMessage>& c2v)
{
    const int32_t dc = graph.row_weight;
    std::vector<TruncMessage> forward(static_cast<size_t>(dc));
    std::vector<TruncMessage> backward(static_cast<size_t>(dc));
    for (int32_t check = 0; check < graph.num_checks; check++)
        {
            const uint32_t begin = graph.check_offsets[static_cast<size_t>(check)];
            forward[0] = v2c[begin];
            for (int32_t l = 1; l < dc - 1; l++)
                {
                    ems_combine(forward[static_cast<size_t>(l) - 1], v2c[begin + static_cast<uint32_t>(l)], forward[static_cast<size_t>(l)]);
                }
            backward[static_cast<size_t>(dc) - 1] = v2c[begin + static_cast<uint32_t>(dc) - 1];
            for (int32_t l = dc - 2; l > 0; l--)
                {
                    ems_combine(v2c[begin + static_cast<uint32_t>(l)], backward[static_cast<size_t>(l) + 1], backward[static_cast<size_t>(l)]);
                }
            c2v[begin] = backward[1];
            c2v[begin + static_cast<uint32_t>(dc) - 1] = forward[static_cast<size_t>(dc) - 2];
            for (int32_t l = 1; l < dc - 1; l++)
                {
                    ems_combine(forward[static_cast<size_t>(l) - 1], backward[static_cast<size_t>(l) + 1], c2v[begin + static_cast<uint32_t>(l)]);
                }
        }
}


// V2C_{j->i} = h_ij * (sum_{f != i} C2V_{f->j} * h_fj^-1 + L_j)_NM.
// Elements missing from a truncated C2V get its largest LLR plus a fixed
// offset. The hard decision uses the full a-posteriori metric.
void ems_variable_update(
    const BeidouLdpcGraph& graph,
    const std::vector<float>& channel_llr,
    const std::vector<TruncMessage>& c2v,
    std::vector<TruncMessage>& v2c,
    std::vector<uint8_t>& hard_codeword)
{
    std::vector<std::array<float, Q>> incoming;
    std::array<float, Q> posterior{};
    std::array<float, Q> outgoing{};
    for (int32_t var = 0; var < graph.num_variables; var++)
        {
            const uint32_t begin = graph.var_offsets[static_cast<size_t>(var)];
            const uint32_t end = graph.var_offsets[static_cast<size_t>(var) + 1];
            incoming.resize(end - begin);
            for (int32_t x = 0; x < Q; x++)
                {
                    posterior[static_cast<size_t>(x)] = channel_llr[static_cast<size_t>(var) * Q + static_cast<size_t>(x)];
                }
            for (uint32_t p = begin; p < end; p++)
                {
                    const TruncMessage& msg = c2v[graph.var_to_edge[p]];
                    auto& in = incoming[p - begin];
                    in.fill(msg.llr[NM - 1] + EMS_TRUNCATION_OFFSET);
                    for (int32_t k = 0; k < NM; k++)
                        {
                            const uint8_t x = GaloisField64::mul(graph.var_to_h_inv[p], msg.sym[k]);
                            in[x] = std::min(in[x], msg.llr[k]);
                        }
                    for (int32_t x = 0; x < Q; x++)
                        {
                            posterior[static_cast<size_t>(x)] += in[static_cast<size_t>(x)];
                        }
                }
            hard_codeword[static_cast<size_t>(var)] =
                static_cast<uint8_t>(std::min_element(posterior.begin(), posterior.end()) - posterior.begin());
            for (uint32_t p = begin; p < end; p++)
                {
                    const auto& in = incoming[p - begin];
                    const uint8_t h_ij = graph.var_to_h[p];
                    for (int32_t x = 0; x < Q; x++)
                        {
                            outgoing[GaloisField64::mul(h_ij, static_cast<uint8_t>(x))] =
                                posterior[static_cast<size_t>(x)] - in[static_cast<size_t>(x)];
                        }
                    truncate_metric_vector(outgoing, v2c[graph.var_to_edge[p]]);
                }
        }
}


using ProbabilityMessage = std::array<double, Q>;

// XOR convolution over GF(2^6) becomes multiplication in the Walsh domain.
// This transform is its own inverse, up to a factor of Q.
void walsh_transform(ProbabilityMessage& message)
{
    for (int32_t stride = 1; stride < Q; stride *= 2)
        {
            for (int32_t start = 0; start < Q; start += 2 * stride)
                {
                    for (int32_t i = start; i < start + stride; i++)
                        {
                            const double a = message[i];
                            const double b = message[i + stride];
                            message[i] = a + b;
                            message[i + stride] = a - b;
                        }
                }
        }
}


void normalize_probability(ProbabilityMessage& message)
{
    double total = 0.0;
    for (auto& value : message)
        {
            // Roundoff in the inverse transform can produce tiny negatives.
            // A floor also prevents hard zeros from locking an erroneous bit.
            value = std::max(value, 1.0e-15);
            total += value;
        }
    for (auto& value : message)
        {
            value /= total;
        }
}


// Full-alphabet sum-product retry for blocks that defeat the truncated
// EMS decoder. Reuse the same graph, channel metrics and GF mapping.
bool sum_product_decode(const BeidouLdpcGraph& graph,
    const std::vector<float>& channel_llr, std::vector<uint8_t>& hard_codeword)
{
    const size_t edges = graph.check_to_var.size();
    std::vector<ProbabilityMessage> channel(static_cast<size_t>(graph.num_variables));
    std::vector<ProbabilityMessage> v2c(edges);
    std::vector<ProbabilityMessage> c2v(edges);
    std::vector<ProbabilityMessage> transformed(edges);
    for (int32_t var = 0; var < graph.num_variables; var++)
        {
            for (int32_t x = 0; x < Q; x++)
                {
                    channel[var][x] = std::exp(-static_cast<double>(channel_llr[static_cast<size_t>(var) * Q + x]));
                }
            normalize_probability(channel[var]);
        }
    for (size_t edge = 0; edge < edges; edge++)
        {
            const auto var = graph.check_to_var[edge];
            const auto h = graph.check_to_h[edge];
            for (int32_t x = 0; x < Q; x++)
                {
                    v2c[edge][GaloisField64::mul(h, static_cast<uint8_t>(x))] = channel[var][x];
                }
        }
    for (int32_t iteration = 0; iteration < BEIDOU_LDPC_MAX_ITER; iteration++)
        {
            transformed = v2c;
            for (auto& message : transformed)
                {
                    walsh_transform(message);
                }
            for (int32_t check = 0; check < graph.num_checks; check++)
                {
                    const auto begin = graph.check_offsets[check];
                    const auto end = graph.check_offsets[check + 1];
                    for (auto target = begin; target < end; target++)
                        {
                            auto& message = c2v[target];
                            message.fill(1.0);
                            for (auto edge = begin; edge < end; edge++)
                                {
                                    if (edge != target)
                                        {
                                            for (int32_t x = 0; x < Q; x++)
                                                {
                                                    message[x] *= transformed[edge][x];
                                                }
                                        }
                                }
                            walsh_transform(message);
                            for (auto& value : message)
                                {
                                    value /= Q;
                                }
                            normalize_probability(message);
                        }
                }
            for (int32_t var = 0; var < graph.num_variables; var++)
                {
                    const auto begin = graph.var_offsets[var];
                    const auto end = graph.var_offsets[var + 1];
                    ProbabilityMessage posterior = channel[var];
                    for (auto pos = begin; pos < end; pos++)
                        {
                            const auto edge = graph.var_to_edge[pos];
                            const auto h = graph.var_to_h[pos];
                            for (int32_t x = 0; x < Q; x++)
                                {
                                    posterior[x] *= c2v[edge][GaloisField64::mul(h, static_cast<uint8_t>(x))];
                                }
                        }
                    hard_codeword[var] = static_cast<uint8_t>(std::max_element(posterior.begin(), posterior.end()) - posterior.begin());
                    for (auto target = begin; target < end; target++)
                        {
                            ProbabilityMessage outgoing = channel[var];
                            for (auto pos = begin; pos < end; pos++)
                                {
                                    if (pos == target)
                                        {
                                            continue;
                                        }
                                    const auto edge = graph.var_to_edge[pos];
                                    const auto h = graph.var_to_h[pos];
                                    for (int32_t x = 0; x < Q; x++)
                                        {
                                            outgoing[x] *= c2v[edge][GaloisField64::mul(h, static_cast<uint8_t>(x))];
                                        }
                                }
                            normalize_probability(outgoing);
                            const auto edge = graph.var_to_edge[target];
                            const auto h = graph.var_to_h[target];
                            for (int32_t x = 0; x < Q; x++)
                                {
                                    v2c[edge][GaloisField64::mul(h, static_cast<uint8_t>(x))] = outgoing[x];
                                }
                        }
                }
            if (syndrome_is_zero(graph, hard_codeword))
                {
                    return true;
                }
        }
    return false;
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
    // ICD matrix is stored in 4-column bands: read each band top-to-bottom, then next band.
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
                            const size_t dst = static_cast<size_t>(dst_offset) + static_cast<size_t>(k);
                            const size_t src = static_cast<size_t>(src_offset) + static_cast<size_t>(k);
                            reordered_index[dst] = icd_index[src];
                            reordered_element[dst] = icd_element[src];
                        }
                }
        }
}


bool decode_block(
    const BeidouLdpcGraph& graph,
    const float* bit_llr,
    int32_t num_bits,
    uint8_t* info_bits,
    uint8_t* codeword_bits,
    bool enable_sum_product)
{
    if (!graph_is_valid(graph) || bit_llr == nullptr ||
        (info_bits == nullptr && codeword_bits == nullptr))
        {
            return false;
        }

    const int32_t n = graph.num_variables;
    const int32_t k = n - graph.num_checks;
    if (num_bits < n * BITS_PER_SYMBOL)
        {
            return false;
        }

    for (int32_t i = 0; i < n * BITS_PER_SYMBOL; i++)
        {
            if (!std::isfinite(bit_llr[i]))
                {
                    return false;
                }
        }

    std::vector<uint8_t> hard_codeword(static_cast<size_t>(n), 0U);
    std::vector<uint8_t> hard_symbol;
    std::vector<float> channel_llr;
    initialize_channel_llr(bit_llr, n, hard_symbol, channel_llr);
    hard_codeword = hard_symbol;

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

    const int32_t edge_count = graph.num_checks * graph.row_weight;
    std::vector<TruncMessage> v2c(static_cast<size_t>(edge_count));
    std::vector<TruncMessage> c2v(static_cast<size_t>(edge_count));
    initialize_v2c_messages(graph, channel_llr, v2c);
    for (int32_t iteration = 0; iteration < BEIDOU_LDPC_MAX_ITER; iteration++)
        {
            ems_check_update(graph, v2c, c2v);
            ems_variable_update(graph, channel_llr, c2v, v2c, hard_codeword);
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
        }

    if (enable_sum_product && sum_product_decode(graph, channel_llr, hard_codeword))
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

    return false;
}


bool init_graph_impl(
    int32_t num_checks,
    int32_t num_variables,
    int32_t row_weight,
    const uint16_t* icd_index,
    const uint8_t* icd_element,
    int32_t num_entries,
    BeidouLdpcGraph& graph)
{
    if (num_checks <= 0 || num_variables <= num_checks || row_weight != 4 || icd_index == nullptr || icd_element == nullptr)
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
    graph.check_offsets.assign(static_cast<size_t>(num_checks) + 1, 0U);
    graph.check_to_var.assign(static_cast<size_t>(num_entries), 0U);
    graph.check_to_h.assign(static_cast<size_t>(num_entries), 0U);
    graph.var_offsets.assign(static_cast<size_t>(num_variables) + 1, 0U);
    graph.var_to_check.assign(static_cast<size_t>(num_entries), 0U);
    graph.var_to_edge.assign(static_cast<size_t>(num_entries), 0U);
    graph.var_to_h.assign(static_cast<size_t>(num_entries), 0U);
    graph.var_to_h_inv.assign(static_cast<size_t>(num_entries), 0U);

    for (int32_t i = 0; i < num_checks; i++)
        {
            graph.check_offsets[static_cast<size_t>(i) + 1] =
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
            graph.var_offsets[static_cast<size_t>(j) + 1] =
                graph.var_offsets[static_cast<size_t>(j)] + var_degree[static_cast<size_t>(j)];
        }

    std::vector<uint32_t> cursor = graph.var_offsets;
    for (int32_t i = 0; i < num_checks; i++)
        {
            const uint32_t begin = graph.check_offsets[static_cast<size_t>(i)];
            const uint32_t end = graph.check_offsets[static_cast<size_t>(i) + 1];
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

bool beidou_ldpc_init_graph(
    int32_t num_checks,
    int32_t num_variables,
    int32_t row_weight,
    const uint16_t* icd_index,
    const uint8_t* icd_element,
    int32_t num_entries,
    BeidouLdpcGraph& graph)
{
    return init_graph_impl(num_checks, num_variables, row_weight, icd_index, icd_element, num_entries, graph);
}


bool beidou_ldpc_decode(const BeidouLdpcGraph& graph, const float* bit_llr,
    int32_t num_bits, uint8_t* info_bits, uint8_t* codeword_bits, bool enable_sum_product)
{
    return decode_block(graph, bit_llr, num_bits, info_bits, codeword_bits, enable_sum_product);
}
