/*!
 * \file beidou_cnav1_navigation_message_test.cc
 * \brief Unit tests for B-CNAV1 navigation message parser
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
#include "beidou_cnav1_navigation_message.h"
#include "rtklib_conversions.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace
{
// LFSR encoder matching beidou_cnav1_navigation_message.cc
std::vector<int8_t> encode_bch_lfsr_bipolar(
    uint16_t msg_bits,
    int32_t n,
    int32_t k,
    const std::vector<int32_t>& feedback_pos_1based)
{
    std::vector<int8_t> register_state(static_cast<size_t>(k), 1);
    std::vector<int8_t> encoded(static_cast<size_t>(n), 1);

    for (int32_t i = 0; i < k; i++)
        {
            const auto bit = static_cast<int32_t>((msg_bits >> static_cast<uint32_t>(k - 1 - i)) & 1U);
            register_state[static_cast<size_t>(k - 1 - i)] = (bit == 0) ? 1 : -1;
        }

    for (int32_t ind = 0; ind < n; ind++)
        {
            encoded[static_cast<size_t>(ind)] = register_state.back();
            int8_t feedback = 1;
            for (const int32_t pos : feedback_pos_1based)
                {
                    feedback = static_cast<int8_t>(feedback * register_state[static_cast<size_t>(pos - 1)]);
                }
            for (int32_t j = k - 1; j > 0; j--)
                {
                    register_state[static_cast<size_t>(j)] = register_state[static_cast<size_t>(j - 1)];
                }
            register_state[0] = feedback;
        }

    return encoded;
}

void append_bch_codeword(
    std::vector<float>& frame,
    uint16_t msg_bits,
    int32_t n,
    int32_t k,
    const std::vector<int32_t>& feedback_pos_1based)
{
    const auto encoded = encode_bch_lfsr_bipolar(msg_bits, n, k, feedback_pos_1based);
    for (const auto cw : encoded)
        {
            // Map bipolar codeword to float symbols expected by the hard-decision BCH decoder.
            frame.push_back(cw > 0 ? -1.0F : 1.0F);
        }
}

void append_bch21_6(std::vector<float>& frame, uint32_t prn)
{
    static const std::vector<int32_t> feedback_pos = {2, 4, 5, 6};
    append_bch_codeword(frame, static_cast<uint16_t>(prn & 0x3FU), 21, 6, feedback_pos);
}

void append_bch51_8(std::vector<float>& frame, uint32_t soh)
{
    static const std::vector<int32_t> feedback_pos = {1, 4, 5, 6, 7, 8};
    append_bch_codeword(frame, static_cast<uint16_t>(soh & 0xFFU), 51, 8, feedback_pos);
}

void deinterleave_like_icd(const std::vector<float>& sf2_bits, const std::vector<float>& sf3_bits, std::vector<float>& interleaved)
{
    std::array<float, BEIDOU_CNAV1_INTERLEAVE_ROWS * BEIDOU_CNAV1_INTERLEAVE_COLS> grid{};
    int32_t sf2_idx = 0;
    int32_t sf3_idx = 0;
    int32_t row = 0;
    while (row < 33)
        {
            for (int32_t r = 0; r < 2; r++)
                {
                    for (int32_t col = 0; col < BEIDOU_CNAV1_INTERLEAVE_COLS; col++)
                        {
                            grid[static_cast<size_t>(row) * BEIDOU_CNAV1_INTERLEAVE_COLS + col] = sf2_bits[static_cast<size_t>(sf2_idx++)];
                        }
                    row++;
                }
            for (int32_t col = 0; col < BEIDOU_CNAV1_INTERLEAVE_COLS; col++)
                {
                    grid[static_cast<size_t>(row) * BEIDOU_CNAV1_INTERLEAVE_COLS + col] = sf3_bits[static_cast<size_t>(sf3_idx++)];
                }
            row++;
        }
    for (int32_t r = 0; r < 3; r++)
        {
            for (int32_t col = 0; col < BEIDOU_CNAV1_INTERLEAVE_COLS; col++)
                {
                    grid[static_cast<size_t>(row) * BEIDOU_CNAV1_INTERLEAVE_COLS + col] = sf2_bits[static_cast<size_t>(sf2_idx++)];
                }
            row++;
        }

    interleaved.clear();
    for (int32_t col = 0; col < BEIDOU_CNAV1_INTERLEAVE_COLS; col++)
        {
            for (int32_t row_idx = 0; row_idx < BEIDOU_CNAV1_INTERLEAVE_ROWS; row_idx++)
                {
                    interleaved.push_back(grid[static_cast<size_t>(row_idx) * BEIDOU_CNAV1_INTERLEAVE_COLS + col]);
                }
        }
}

std::vector<float> encode_bits_to_llr(const std::vector<uint8_t>& bits)
{
    std::vector<float> llr;
    llr.reserve(bits.size());
    for (const auto bit : bits)
        {
            llr.push_back(bit != 0U ? 4.0F : -4.0F);
        }
    return llr;
}
}  // namespace

TEST(BeidouCnav1NavigationMessageTest, DecodeSubframe1OnlyFrameFailsWithoutSf2Crc)
{
    std::vector<float> frame;
    append_bch21_6(frame, 19U);
    append_bch51_8(frame, 1U);
    // Non-constant SF2/SF3 noise (constant LLRs can decode as the all-zero codeword).
    for (int32_t i = 0; i < BEIDOU_CNAV1_INTERLEAVED_SYMBOLS; i++)
        {
            frame.push_back(((i * 17 + 3) % 5 < 2) ? 4.0F : -4.0F);
        }

    Beidou_Cnav1_Navigation_Message nav;
    EXPECT_FALSE(nav.decode_frame_symbols(frame.data(), BEIDOU_CNAV1_FRAME_SYMBOLS));
}

TEST(BeidouCnav1NavigationMessageTest, DecodeEphemerisFromZeroSf2Payload)
{
    const auto sf2_llr = encode_bits_to_llr(std::vector<uint8_t>(BEIDOU_CNAV1_SUBFRAME2_SYMBOLS, 0U));
    const auto sf3_llr = encode_bits_to_llr(std::vector<uint8_t>(BEIDOU_CNAV1_SUBFRAME3_SYMBOLS, 0U));
    EXPECT_EQ(sf2_llr.size(), static_cast<size_t>(BEIDOU_CNAV1_SUBFRAME2_SYMBOLS));
    EXPECT_EQ(sf3_llr.size(), static_cast<size_t>(BEIDOU_CNAV1_SUBFRAME3_SYMBOLS));

    std::vector<float> interleaved;
    deinterleave_like_icd(sf2_llr, sf3_llr, interleaved);
    EXPECT_EQ(interleaved.size(), static_cast<size_t>(BEIDOU_CNAV1_INTERLEAVED_SYMBOLS));

    std::vector<float> frame;
    append_bch21_6(frame, 19U);
    append_bch51_8(frame, 1U);
    frame.insert(frame.end(), interleaved.begin(), interleaved.end());

    Beidou_Cnav1_Navigation_Message nav;
    ASSERT_TRUE(nav.decode_frame_symbols(frame.data(), BEIDOU_CNAV1_FRAME_SYMBOLS, 19));
    EXPECT_TRUE(nav.have_new_ephemeris());
    EXPECT_EQ(nav.get_ephemeris().PRN, 19);
    EXPECT_DOUBLE_EQ(nav.get_tow_s(), 18.0);

    const auto rtklib_eph = eph_to_rtklib(nav.get_ephemeris());
    EXPECT_EQ(rtklib_eph.iode, 0);
    EXPECT_EQ(rtklib_eph.iodc, 0);
    EXPECT_NEAR(rtklib_eph.A, BEIDOU_CNAV1_A_REF_MEO, 1.0);
    EXPECT_EQ(rtklib_eph.code, 7);
    EXPECT_DOUBLE_EQ(rtklib_eph.Adot, nav.get_ephemeris().Adot);
    EXPECT_DOUBLE_EQ(rtklib_eph.ndot, nav.get_ephemeris().delta_n0dot);
}

TEST(BeidouCnav1NavigationMessageTest, RejectsEphemerisWhenIodeIodcMismatch)
{
    // SF2 with IODC/IODE mismatch (ICD §7.4.3).
    std::vector<uint8_t> sf2_bits(static_cast<size_t>(BEIDOU_CNAV1_SUBFRAME2_SYMBOLS), 0U);
    const int32_t iodc_offset = 13 + 8;
    const uint32_t iodc = 0x101U;  // low 8 = 0x01
    for (int32_t b = 0; b < 10; b++)
        {
            sf2_bits[static_cast<size_t>(iodc_offset + b)] = static_cast<uint8_t>((iodc >> static_cast<uint32_t>(9 - b)) & 1U);
        }
    const int32_t iode_offset = iodc_offset + 10;
    const uint32_t iode = 0x02U;
    for (int32_t b = 0; b < 8; b++)
        {
            sf2_bits[static_cast<size_t>(iode_offset + b)] = static_cast<uint8_t>((iode >> static_cast<uint32_t>(7 - b)) & 1U);
        }
    const int32_t sat_type_offset = iode_offset + 8 + 11;  // MEO
    sf2_bits[static_cast<size_t>(sat_type_offset)] = 1U;
    sf2_bits[static_cast<size_t>(sat_type_offset + 1)] = 1U;

    auto crc24q_bits = [](const uint8_t* bits, int32_t num_bits) -> uint32_t {
        uint32_t crc = 0;
        const uint32_t POLY = 0x864CFBU;
        for (int32_t i = 0; i < num_bits; i++)
            {
                uint32_t msb = (crc >> 23) & 1U;
                uint32_t bit = (bits[i] != 0U) ? 1U : 0U;
                crc = (crc << 1) & 0xFFFFFFU;
                if (msb ^ bit)
                    {
                        crc ^= POLY;
                    }
            }
        return crc;
    };
    const uint32_t crc = crc24q_bits(sf2_bits.data(), BEIDOU_CNAV1_SF2_DATA_BITS - BEIDOU_CNAV1_CRC_BITS);
    for (int32_t b = 0; b < BEIDOU_CNAV1_CRC_BITS; b++)
        {
            sf2_bits[static_cast<size_t>(BEIDOU_CNAV1_SF2_DATA_BITS - BEIDOU_CNAV1_CRC_BITS + b)] =
                static_cast<uint8_t>((crc >> static_cast<uint32_t>(23 - b)) & 1U);
        }

    const auto sf2_llr = encode_bits_to_llr(sf2_bits);
    const auto sf3_llr = encode_bits_to_llr(std::vector<uint8_t>(BEIDOU_CNAV1_SUBFRAME3_SYMBOLS, 0U));
    std::vector<float> interleaved;
    deinterleave_like_icd(sf2_llr, sf3_llr, interleaved);

    std::vector<float> frame;
    append_bch21_6(frame, 19U);
    append_bch51_8(frame, 1U);
    frame.insert(frame.end(), interleaved.begin(), interleaved.end());

    Beidou_Cnav1_Navigation_Message nav;
    ASSERT_TRUE(nav.decode_frame_symbols(frame.data(), BEIDOU_CNAV1_FRAME_SYMBOLS, 19));
    EXPECT_FALSE(nav.have_new_ephemeris());
}
