/*!
 * \file beidou_cnav1_ldpc_test.cc
 * \brief Unit tests for B-CNAV1 NB-LDPC decoder
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
#include "Beidou_CNAV1.h"
#include "beidou_cnav1_ldpc.h"
#include "beidou_cnav_test_helpers.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

namespace
{
std::array<float, BEIDOU_CNAV1_SUBFRAME2_SYMBOLS> make_bit_llrs(const uint8_t* bits, float magnitude)
{
    std::array<float, BEIDOU_CNAV1_SUBFRAME2_SYMBOLS> llr{};
    for (int32_t i = 0; i < BEIDOU_CNAV1_SUBFRAME2_SYMBOLS; i++)
        {
            llr[static_cast<size_t>(i)] = (bits[i] != 0U) ? magnitude : -magnitude;
        }
    return llr;
}

std::array<float, BEIDOU_CNAV1_SUBFRAME3_SYMBOLS> make_sf3_bit_llrs(const uint8_t* bits, float magnitude)
{
    std::array<float, BEIDOU_CNAV1_SUBFRAME3_SYMBOLS> llr{};
    for (int32_t i = 0; i < BEIDOU_CNAV1_SUBFRAME3_SYMBOLS; i++)
        {
            llr[static_cast<size_t>(i)] = (bits[i] != 0U) ? magnitude : -magnitude;
        }
    return llr;
}

// Encode random information blocks, send them over BPSK/AWGN and count
// blocks the Extended Min-Sum decoder (no sum-product retry) fails to recover.
template <int N>
int32_t count_ems_block_errors(const BeidouCnav1LdpcGraph& graph, double ebn0_db, int32_t blocks, uint32_t seed)
{
    BeidouCnavTest::PortableGaussian gauss(seed);
    std::mt19937 bit_rng(seed + 1U);
    const double sigma = BeidouCnavTest::rate_half_bpsk_sigma(ebn0_db);
    int32_t errors = 0;
    for (int32_t block = 0; block < blocks; block++)
        {
            std::array<uint8_t, N / 2> info{};
            for (auto& bit : info)
                {
                    bit = static_cast<uint8_t>(bit_rng() & 1U);
                }
            const auto codeword = BeidouCnavTest::encode<N>(info);
            std::vector<float> llr(N);
            for (int32_t i = 0; i < N; i++)
                {
                    // Positive LLR favors bit 1; 2y/sigma^2 is the ICD Annex scale.
                    const double y = (codeword[static_cast<size_t>(i)] != 0U ? 1.0 : -1.0) + sigma * gauss();
                    llr[static_cast<size_t>(i)] = static_cast<float>(2.0 * y / (sigma * sigma));
                }
            std::vector<uint8_t> decoded(N, 0U);
            if (!beidou_ldpc_decode(graph, llr.data(), N, nullptr, decoded.data(), false) ||
                !std::equal(decoded.begin(), decoded.end(), codeword.begin()))
                {
                    errors++;
                }
        }
    return errors;
}
}  // namespace

TEST(BeidouCnav1LdpcTest, DecodeZeroCodeword200_100)
{
    std::array<uint8_t, BEIDOU_CNAV1_SUBFRAME2_SYMBOLS> bits{};
    const auto llr = make_bit_llrs(bits.data(), 4.0F);
    std::array<uint8_t, BEIDOU_CNAV1_SF2_DATA_BITS> info{};
    EXPECT_TRUE(beidou_cnav1_ldpc_decode_200_100(llr.data(), BEIDOU_CNAV1_SUBFRAME2_SYMBOLS, info.data()));
    for (const auto bit : info)
        {
            EXPECT_EQ(bit, 0U);
        }
}

TEST(BeidouCnav1LdpcTest, DecodeZeroCodeword88_44)
{
    std::array<uint8_t, BEIDOU_CNAV1_SUBFRAME3_SYMBOLS> bits{};
    const auto llr = make_sf3_bit_llrs(bits.data(), 4.0F);
    std::array<uint8_t, BEIDOU_CNAV1_SF3_DATA_BITS> info{};
    EXPECT_TRUE(beidou_cnav1_ldpc_decode_88_44(llr.data(), BEIDOU_CNAV1_SUBFRAME3_SYMBOLS, info.data()));
    for (const auto bit : info)
        {
            EXPECT_EQ(bit, 0U);
        }
}

TEST(BeidouCnav1LdpcTest, CorrectSingleSymbolError200_100)
{
    std::array<uint8_t, BEIDOU_CNAV1_SUBFRAME2_SYMBOLS> bits{};
    const uint8_t wrong_symbol = 5U;
    for (int32_t bit = 0; bit < 6; bit++)
        {
            bits[static_cast<size_t>(bit)] = static_cast<uint8_t>((wrong_symbol >> (5 - bit)) & 1U);
        }

    const auto llr = make_bit_llrs(bits.data(), 4.0F);
    std::array<uint8_t, BEIDOU_CNAV1_SF2_DATA_BITS> info{};
    EXPECT_TRUE(beidou_cnav1_ldpc_decode_200_100(llr.data(), BEIDOU_CNAV1_SUBFRAME2_SYMBOLS, info.data()));
    for (const auto bit : info)
        {
            EXPECT_EQ(bit, 0U);
        }
}

TEST(BeidouCnav1LdpcTest, CorrectSingleSymbolError88_44)
{
    std::array<uint8_t, BEIDOU_CNAV1_SUBFRAME3_SYMBOLS> bits{};
    const uint8_t wrong_symbol = 1U;  // single GF(64) symbol error
    for (int32_t bit = 0; bit < 6; bit++)
        {
            bits[static_cast<size_t>(bit)] = static_cast<uint8_t>((wrong_symbol >> (5 - bit)) & 1U);
        }

    const auto llr = make_sf3_bit_llrs(bits.data(), 4.0F);
    std::array<uint8_t, BEIDOU_CNAV1_SF3_DATA_BITS> info{};
    EXPECT_TRUE(beidou_cnav1_ldpc_decode_88_44(llr.data(), BEIDOU_CNAV1_SUBFRAME3_SYMBOLS, info.data()));
    for (const auto bit : info)
        {
            EXPECT_EQ(bit, 0U);
        }
}

TEST(BeidouCnav1LdpcTest, EmsCorrectsNoisyBlocks200_100)
{
    // About 115 hard-decision bit errors per block at this Eb/N0.
    EXPECT_EQ(count_ems_block_errors<BEIDOU_CNAV1_SUBFRAME2_SYMBOLS>(beidou_cnav1_ldpc_graph_200_100(), 3.5, 20, 2026U), 0);
}

TEST(BeidouCnav1LdpcTest, EmsCorrectsNoisyBlocks88_44)
{
    EXPECT_EQ(count_ems_block_errors<BEIDOU_CNAV1_SUBFRAME3_SYMBOLS>(beidou_cnav1_ldpc_graph_88_44(), 3.5, 20, 2027U), 0);
}
