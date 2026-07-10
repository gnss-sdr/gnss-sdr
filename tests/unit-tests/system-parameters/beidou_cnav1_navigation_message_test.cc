/*!
 * \file beidou_cnav1_navigation_message_test.cc
 * \brief Unit tests for B-CNAV1 navigation message parser
 * SPDX-License-Identifier: GPL-3.0-or-later
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
    EXPECT_DOUBLE_EQ(rtklib_eph.ndot, nav.get_ephemeris().delta_n_dot);
}
