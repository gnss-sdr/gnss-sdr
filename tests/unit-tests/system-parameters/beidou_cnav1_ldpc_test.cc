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
#include <gtest/gtest.h>
#include <array>
#include <cstring>

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
