/*!
 * \file beidou_cnav2_navigation_message_test.cc
 * \brief Unit tests for B-CNAV2 CRC and MT10/11/30 parsing (systematic bits)
 * \author huangchuhan, 2026. huangchh37(at)mail2.sysu.edu.cn
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
#include "Beidou_CNAV2.h"
#include "beidou_cnav2_navigation_message.h"
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <vector>

namespace
{
// Named distinctly from other CNAV tests included via test_main.cc.
uint32_t crc24q_cnav2(const uint8_t* bits, int32_t num_bits)
{
    uint32_t crc = 0;
    constexpr uint32_t POLY = 0x864CFBU;
    for (int32_t i = 0; i < num_bits; i++)
        {
            const uint32_t msb = (crc >> 23) & 1U;
            const uint32_t bit = (bits[i] != 0U) ? 1U : 0U;
            crc = (crc << 1) & 0xFFFFFFU;
            if (msb ^ bit)
                {
                    crc ^= POLY;
                }
        }
    return crc;
}

void write_unsigned_cnav2(uint8_t* bits, int32_t offset, int32_t length, uint64_t value)
{
    for (int32_t i = 0; i < length; i++)
        {
            bits[offset + i] = static_cast<uint8_t>((value >> (length - 1 - i)) & 1U);
        }
}

void write_signed(uint8_t* bits, int32_t offset, int32_t length, int64_t value)
{
    uint64_t raw = static_cast<uint64_t>(value);
    if (value < 0)
        {
            raw = static_cast<uint64_t>(value + (1LL << length));
        }
    write_unsigned_cnav2(bits, offset, length, raw);
}

std::vector<float> make_frame(const std::array<uint8_t, BEIDOU_CNAV2_INFO_BITS>& info)
{
    std::vector<float> symbols(static_cast<size_t>(BEIDOU_CNAV2_FRAME_SYMBOLS), 1.0F);
    const char* preamble = "111000100100110111101000";
    for (int i = 0; i < BEIDOU_CNAV2_PREAMBLE_SYMBOLS; i++)
        {
            symbols[static_cast<size_t>(i)] = (preamble[i] == '1') ? -1.0F : 1.0F;
        }
    for (int i = 0; i < BEIDOU_CNAV2_INFO_BITS; i++)
        {
            symbols[static_cast<size_t>(BEIDOU_CNAV2_PREAMBLE_SYMBOLS + i)] =
                info[static_cast<size_t>(i)] ? -1.0F : 1.0F;
        }
    return symbols;
}

std::array<uint8_t, BEIDOU_CNAV2_INFO_BITS> make_info(uint32_t prn, int32_t mes_type, uint32_t sow_counts)
{
    std::array<uint8_t, BEIDOU_CNAV2_INFO_BITS> info{};
    write_unsigned_cnav2(info.data(), 0, 6, prn);
    write_unsigned_cnav2(info.data(), 6, 6, static_cast<uint64_t>(mes_type));
    write_unsigned_cnav2(info.data(), 12, 18, sow_counts);
    if (mes_type == BEIDOU_CNAV2_MSG_EPH1)
        {
            write_unsigned_cnav2(info.data(), 30, 13, 900);  // WN
            write_unsigned_cnav2(info.data(), 61, 11, 10);   // toe = 3000 s
            write_unsigned_cnav2(info.data(), 72, 2, 3);     // MEO
            write_signed(info.data(), 74, 26, 512);          // deltaA = 1 m
            write_signed(info.data(), 165, 33, 1);           // M0
            write_unsigned_cnav2(info.data(), 198, 33, 1);   // e
        }
    else if (mes_type == BEIDOU_CNAV2_MSG_EPH2)
        {
            write_unsigned_cnav2(info.data(), 30, 2, 0);  // HS
            write_signed(info.data(), 42, 33, 1);         // OMEGA0
        }
    else if (mes_type == BEIDOU_CNAV2_MSG_CLK_IONO)
        {
            write_unsigned_cnav2(info.data(), 42, 11, 10);  // toc = 3000 s
            write_signed(info.data(), 53, 25, 1);           // af0
            write_signed(info.data(), 121, 12, 2);          // TGD_B2ap
            write_signed(info.data(), 133, 12, 3);          // ISC_B2ad
        }
    const uint32_t crc = crc24q_cnav2(info.data(), BEIDOU_CNAV2_DATA_BITS);
    write_unsigned_cnav2(info.data(), BEIDOU_CNAV2_DATA_BITS, BEIDOU_CNAV2_CRC_BITS, crc);
    return info;
}
}  // namespace


TEST(BeidouCnav2NavigationMessageTest, AcceptsValidCrcAndPreamble)
{
    const auto info = make_info(19, BEIDOU_CNAV2_MSG_EPH1, 12345);
    const auto frame = make_frame(info);
    Beidou_Cnav2_Navigation_Message nav;
    ASSERT_TRUE(nav.decode_frame_symbols(frame.data(), static_cast<int32_t>(frame.size()), 19));
    EXPECT_TRUE(nav.last_crc_ok());
    EXPECT_EQ(nav.last_mes_type(), BEIDOU_CNAV2_MSG_EPH1);
    EXPECT_EQ(nav.last_sow(), 12345 * 3);
    EXPECT_EQ(nav.last_frame_prn(), 19U);
    EXPECT_FALSE(nav.have_new_ephemeris());
    const auto eph = nav.get_ephemeris();
    EXPECT_EQ(eph.WN, 900);
    EXPECT_EQ(eph.toe, 3000);
    EXPECT_EQ(eph.sat_type, 3);
    EXPECT_NEAR(eph.A0, BEIDOU_CNAV1_A_REF_MEO + 512.0 * BEIDOU_CNAV1_DELTA_A_LSB, 1.0e-6);
}


TEST(BeidouCnav2NavigationMessageTest, RejectsBadCrc)
{
    auto info = make_info(19, BEIDOU_CNAV2_MSG_EPH1, 1000);
    info[100] ^= 1U;
    const auto frame = make_frame(info);
    Beidou_Cnav2_Navigation_Message nav;
    EXPECT_FALSE(nav.decode_frame_symbols(frame.data(), static_cast<int32_t>(frame.size()), 19));
}


TEST(BeidouCnav2NavigationMessageTest, EmitsEphemerisAfterMt10Mt11Mt30)
{
    Beidou_Cnav2_Navigation_Message nav;
    const auto mt10 = make_frame(make_info(21, BEIDOU_CNAV2_MSG_EPH1, 2000));
    const auto mt11 = make_frame(make_info(21, BEIDOU_CNAV2_MSG_EPH2, 2001));
    const auto mt30 = make_frame(make_info(21, BEIDOU_CNAV2_MSG_CLK_IONO, 2002));
    ASSERT_TRUE(nav.decode_frame_symbols(mt10.data(), static_cast<int32_t>(mt10.size()), 21));
    EXPECT_FALSE(nav.have_new_ephemeris());
    ASSERT_TRUE(nav.decode_frame_symbols(mt11.data(), static_cast<int32_t>(mt11.size()), 21));
    EXPECT_FALSE(nav.have_new_ephemeris());
    ASSERT_TRUE(nav.decode_frame_symbols(mt30.data(), static_cast<int32_t>(mt30.size()), 21));
    ASSERT_TRUE(nav.have_new_ephemeris());
    const auto eph = nav.get_ephemeris();
    EXPECT_EQ(eph.PRN, 21U);
    EXPECT_EQ(eph.WN, 900);
    EXPECT_EQ(eph.toc, 3000);
    EXPECT_EQ(eph.sig_type, BDS_EPH_SOURCE_CNAV2);
    EXPECT_NEAR(eph.TGD_B2ap, 2.0 * BEIDOU_CNAV1_TGD_LSB, 1.0e-20);
    EXPECT_NEAR(eph.ISC_B2ad, 3.0 * BEIDOU_CNAV1_ISC_LSB, 1.0e-20);
}


TEST(BeidouCnav2NavigationMessageTest, GeoDoesNotEmitEphemeris)
{
    Beidou_Cnav2_Navigation_Message nav;
    auto info10 = make_info(59, BEIDOU_CNAV2_MSG_EPH1, 3000);
    write_unsigned_cnav2(info10.data(), 72, 2, 1);  // GEO
    const uint32_t crc = crc24q_cnav2(info10.data(), BEIDOU_CNAV2_DATA_BITS);
    write_unsigned_cnav2(info10.data(), BEIDOU_CNAV2_DATA_BITS, BEIDOU_CNAV2_CRC_BITS, crc);
    const auto mt10 = make_frame(info10);
    const auto mt11 = make_frame(make_info(59, BEIDOU_CNAV2_MSG_EPH2, 3001));
    const auto mt30 = make_frame(make_info(59, BEIDOU_CNAV2_MSG_CLK_IONO, 3002));
    ASSERT_TRUE(nav.decode_frame_symbols(mt10.data(), static_cast<int32_t>(mt10.size()), 59));
    ASSERT_TRUE(nav.decode_frame_symbols(mt11.data(), static_cast<int32_t>(mt11.size()), 59));
    ASSERT_TRUE(nav.decode_frame_symbols(mt30.data(), static_cast<int32_t>(mt30.size()), 59));
    EXPECT_FALSE(nav.have_new_ephemeris());
}


TEST(BeidouCnav2NavigationMessageTest, DecodesConsecutiveMt10Mt11Mt30Frames)
{
    Beidou_Cnav2_Navigation_Message nav;
    int eph_count = 0;
    for (int k = 0; k < 8; k++)
        {
            const uint32_t sow0 = static_cast<uint32_t>(4000 + 3 * k);
            const auto mt10 = make_frame(make_info(27, BEIDOU_CNAV2_MSG_EPH1, sow0));
            const auto mt11 = make_frame(make_info(27, BEIDOU_CNAV2_MSG_EPH2, sow0 + 1));
            const auto mt30 = make_frame(make_info(27, BEIDOU_CNAV2_MSG_CLK_IONO, sow0 + 2));
            ASSERT_TRUE(nav.decode_frame_symbols(mt10.data(), BEIDOU_CNAV2_FRAME_SYMBOLS, 27));
            ASSERT_TRUE(nav.decode_frame_symbols(mt11.data(), BEIDOU_CNAV2_FRAME_SYMBOLS, 27));
            ASSERT_TRUE(nav.decode_frame_symbols(mt30.data(), BEIDOU_CNAV2_FRAME_SYMBOLS, 27));
            if (nav.have_new_ephemeris())
                {
                    eph_count++;
                    EXPECT_EQ(nav.get_ephemeris().sig_type, BDS_EPH_SOURCE_CNAV2);
                    nav.clear_flags();
                }
        }
    EXPECT_GE(eph_count, 8);
}
