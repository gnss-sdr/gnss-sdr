/*!
 * \file beidou_cnav2_ldpc_test.cc
 * \brief B-CNAV2 LDPC tests using the ICD Annex encoding example
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
#include "beidou_cnav2_test_helpers.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <limits>
#include <random>

namespace
{
// BDS-SIS-ICD-B2a-1.0 Annex, pp. 64-65. This is not a CRC-bearing NAV frame.
constexpr char CNAV2_ICD_CODEWORD[] =
    "001010110010010011100001001010100110010000101001"
    "101100101111011100000101001110111010001001110100"
    "100010111111000101011100000110111101000000110001"
    "110100110111000101011001010000110011011011111010"
    "001011010000001001001000110111100101100011001001"
    "110110100111010110100000011001000100001111000111"
    "100000001000101101111001001011110111101101111111"
    "000000100011000110101110101011001100100001100101"
    "010111010010000101000010111011001010101111101100"
    "011000101010010011000001000001001101111000001100"
    "111001110101100111110100101111010111111010111111"
    "101100011111101011000010000110000001110000101100";
static_assert(sizeof(CNAV2_ICD_CODEWORD) == 577, "ICD example must contain 576 bits");

std::array<uint8_t, 576> cnav2_icd_bits()
{
    std::array<uint8_t, 576> bits{};
    for (size_t i = 0; i < bits.size(); i++)
        {
            bits[i] = CNAV2_ICD_CODEWORD[i] == '1' ? 1U : 0U;
        }
    return bits;
}

std::array<float, 576> cnav2_icd_llrs()
{
    std::array<float, 576> llr{};
    for (size_t i = 0; i < llr.size(); i++)
        {
            llr[i] = CNAV2_ICD_CODEWORD[i] == '1' ? 4.0F : -4.0F;
        }
    return llr;
}
}  // namespace

TEST(BeidouCnav2LdpcTest, ReferenceEncoderMatchesIcdExample)
{
    const auto codeword = cnav2_icd_bits();
    std::array<uint8_t, 288> info{};
    std::copy_n(codeword.begin(), info.size(), info.begin());
    EXPECT_EQ(BeidouCnav2Test::encode(info), codeword);
}

TEST(BeidouCnav2LdpcTest, IcdExampleSatisfiesEveryParityCheck)
{
    const auto& graph = beidou_cnav2_ldpc_graph_96_48();
    ASSERT_EQ(graph.num_checks, 48);
    ASSERT_EQ(graph.num_variables, 96);
    ASSERT_EQ(graph.check_to_var.size(), 192U);
    const auto bits = cnav2_icd_bits();
    for (size_t row = 0; row < 48; row++)
        {
            uint8_t syndrome = 0;
            for (size_t e = row * 4; e < row * 4 + 4; e++)
                {
                    uint8_t symbol = 0;
                    for (size_t b = 0; b < 6; b++)
                        {
                            symbol = static_cast<uint8_t>((symbol << 1U) | bits[graph.check_to_var[e] * 6 + b]);
                        }
                    syndrome ^= BeidouCnav2Test::multiply(graph.check_to_h[e], symbol);
                }
            EXPECT_EQ(syndrome, 0U) << "check " << row;
        }
}

TEST(BeidouCnav2LdpcTest, DecodesIcdExampleAndZeroCodeword)
{
    auto llr = cnav2_icd_llrs();
    const auto expected = cnav2_icd_bits();
    std::array<uint8_t, 576> decoded{};
    ASSERT_TRUE(beidou_cnav2_ldpc_decode_96_48_codeword(llr.data(), llr.size(), decoded.data()));
    EXPECT_EQ(decoded, expected);
    std::array<uint8_t, 288> info{};
    ASSERT_TRUE(beidou_cnav2_ldpc_decode_96_48(llr.data(), llr.size(), info.data()));
    EXPECT_TRUE(std::equal(info.begin(), info.end(), expected.begin()));
    llr.fill(-4.0F);
    ASSERT_TRUE(beidou_cnav2_ldpc_decode_96_48_codeword(llr.data(), llr.size(), decoded.data()));
    EXPECT_EQ(decoded, (std::array<uint8_t, 576>{}));
}

TEST(BeidouCnav2LdpcTest, CorrectsSingleBitErrorAtEveryCodewordPosition)
{
    const auto expected = cnav2_icd_bits();
    for (size_t bit = 0; bit < 576; bit++)
        {
            SCOPED_TRACE(bit);
            auto llr = cnav2_icd_llrs();
            llr[bit] *= -1.0F;
            std::array<uint8_t, 576> decoded{};
            ASSERT_TRUE(beidou_cnav2_ldpc_decode_96_48_codeword(llr.data(), llr.size(), decoded.data()));
            EXPECT_EQ(decoded, expected);
        }
}

TEST(BeidouCnav2LdpcTest, CorrectsMultipleBitsInOneSymbolAtEveryPosition)
{
    const auto expected = cnav2_icd_bits();
    for (size_t bit = 0; bit < 576; bit += 6)
        {
            SCOPED_TRACE(bit);
            auto llr = cnav2_icd_llrs();
            llr[bit] *= -1.0F;
            llr[bit + 2] *= -1.0F;
            llr[bit + 5] *= -1.0F;
            std::array<uint8_t, 576> decoded{};
            ASSERT_TRUE(beidou_cnav2_ldpc_decode_96_48_codeword(llr.data(), llr.size(), decoded.data()));
            EXPECT_EQ(decoded, expected);
        }
}

TEST(BeidouCnav2LdpcTest, CorrectsMultipleLowConfidenceErrors)
{
    auto llr = cnav2_icd_llrs();
    for (size_t bit : {0U, 35U, 131U, 287U, 301U, 400U, 575U})
        {
            llr[bit] *= -0.1F;
        }
    std::array<uint8_t, 576> decoded{};
    ASSERT_TRUE(beidou_cnav2_ldpc_decode_96_48_codeword(llr.data(), llr.size(), decoded.data()));
    EXPECT_EQ(decoded, cnav2_icd_bits());
}

TEST(BeidouCnav2LdpcTest, RejectsInvalidInputWithoutWritingOutput)
{
    auto llr = cnav2_icd_llrs();
    std::array<uint8_t, 288> output{};
    output.fill(0xA5);
    const auto before = output;
    EXPECT_FALSE(beidou_cnav2_ldpc_decode_96_48(nullptr, 576, output.data()));
    EXPECT_FALSE(beidou_cnav2_ldpc_decode_96_48(llr.data(), 575, output.data()));
    EXPECT_FALSE(beidou_cnav2_ldpc_decode_96_48(llr.data(), -1, output.data()));
    EXPECT_FALSE(beidou_cnav2_ldpc_decode_96_48(llr.data(), 576, nullptr));
    EXPECT_FALSE(beidou_cnav2_ldpc_decode_96_48_codeword(llr.data(), 576, nullptr));
    llr[575] = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(beidou_cnav2_ldpc_decode_96_48(llr.data(), 576, output.data()));
    llr[575] = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(beidou_cnav2_ldpc_decode_96_48(llr.data(), 576, output.data()));
    EXPECT_EQ(output, before);
}

TEST(BeidouCnav2LdpcTest, RejectsUncorrectableNoiseWithoutWritingOutput)
{
    std::mt19937 rng(2026);
    std::array<float, 576> llr{};
    for (auto& bit : llr)
        {
            bit = (rng() & 1U) ? 4.0F : -4.0F;
        }
    std::array<uint8_t, 288> output{};
    output.fill(0xA5);
    const auto before = output;
    EXPECT_FALSE(beidou_cnav2_ldpc_decode_96_48(llr.data(), llr.size(), output.data()));
    EXPECT_EQ(output, before);
}
