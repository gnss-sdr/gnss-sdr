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

namespace
{
void append_bch21_6(std::vector<float>& frame, uint32_t prn)
{
    std::array<int32_t, 6> msg{};
    for (int32_t bit = 5; bit >= 0; bit--)
        {
            msg[5 - bit] = static_cast<int32_t>((prn >> bit) & 1U);
        }
    static const int32_t g[7] = {1, 0, 1, 0, 1, 1, 1};
    std::array<int32_t, 21> codeword{};
    for (int32_t i = 0; i < 6; i++)
        {
            codeword[i] = msg[i];
        }
    for (int32_t i = 0; i < 6; i++)
        {
            if (codeword[i] != 0)
                {
                    for (int32_t j = 0; j < 7; j++)
                        {
                            codeword[i + j] ^= g[j];
                        }
                }
        }
    for (const auto bit : codeword)
        {
            frame.push_back(bit != 0 ? 1.0F : -1.0F);
        }
}

void append_bch51_8(std::vector<float>& frame, uint32_t soh)
{
    std::array<int32_t, 8> msg{};
    for (int32_t bit = 7; bit >= 0; bit--)
        {
            msg[7 - bit] = static_cast<int32_t>((soh >> bit) & 1U);
        }
    static const int32_t g[9] = {1, 1, 0, 0, 1, 1, 1, 1, 1};
    std::array<int32_t, 51> codeword{};
    for (int32_t i = 0; i < 8; i++)
        {
            codeword[i] = msg[i];
        }
    for (int32_t i = 0; i < 8; i++)
        {
            if (codeword[i] != 0)
                {
                    for (int32_t j = 0; j < 9; j++)
                        {
                            codeword[i + j] ^= g[j];
                        }
                }
        }
    for (const auto bit : codeword)
        {
            frame.push_back(bit != 0 ? 1.0F : -1.0F);
        }
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
    frame.insert(frame.end(), BEIDOU_CNAV1_INTERLEAVED_SYMBOLS, 4.0F);

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
    /* Adot/ndot retained through parse → RTKLIB conversion (may be zero in synthetic frame) */
    EXPECT_DOUBLE_EQ(rtklib_eph.Adot, nav.get_ephemeris().Adot);
    EXPECT_DOUBLE_EQ(rtklib_eph.ndot, nav.get_ephemeris().delta_n0dot);
}

TEST(BeidouCnav1NavigationMessageTest, RejectsEphemerisWhenIodeIodcMismatch)
{
    // Build SF2 with IODC=0x101 (low 8 = 0x01) and IODE=0x02 → §7.4.3 mismatch.
    std::vector<uint8_t> sf2_bits(static_cast<size_t>(BEIDOU_CNAV1_SUBFRAME2_SYMBOLS), 0U);
    // WN=0, HOW=0 already zero; set IODC at bit 21 (13+8), 10 bits = 0x101
    const int32_t iodc_offset = 13 + 8;
    const uint32_t iodc = 0x101U;
    for (int32_t b = 0; b < 10; b++)
        {
            sf2_bits[static_cast<size_t>(iodc_offset + b)] = static_cast<uint8_t>((iodc >> static_cast<uint32_t>(9 - b)) & 1U);
        }
    // IODE at bit 31, 8 bits = 0x02
    const int32_t iode_offset = iodc_offset + 10;
    const uint32_t iode = 0x02U;
    for (int32_t b = 0; b < 8; b++)
        {
            sf2_bits[static_cast<size_t>(iode_offset + b)] = static_cast<uint8_t>((iode >> static_cast<uint32_t>(7 - b)) & 1U);
        }
    // SatType MEO = 0b11 at toe(11)+offset after IODE
    const int32_t sat_type_offset = iode_offset + 8 + 11;
    sf2_bits[static_cast<size_t>(sat_type_offset)] = 1U;
    sf2_bits[static_cast<size_t>(sat_type_offset + 1)] = 1U;

    // Append valid CRC24Q over first 576 bits (zeros+fields above) — use decoder's CRC by
    // zeroing CRC field and computing via a local copy of the same polynomial as the parser.
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
