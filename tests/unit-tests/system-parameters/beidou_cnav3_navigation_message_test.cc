/*!
 * \file beidou_cnav3_navigation_message_test.cc
 * \brief Unit tests for B-CNAV3 CRC and MT10/MT30 parsing (systematic bits)
 * \author Chandoss, 2026. huangchh37(at)mail2.sysu.edu.cn
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
#include "Beidou_CNAV3.h"
#include "beidou_cnav3_navigation_message.h"
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <vector>

namespace
{
uint32_t crc24q(const uint8_t* bits, int32_t num_bits)
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

void write_unsigned(uint8_t* bits, int32_t offset, int32_t length, uint64_t value)
{
    for (int32_t i = 0; i < length; i++)
        {
            bits[offset + i] = static_cast<uint8_t>((value >> (length - 1 - i)) & 1U);
        }
}

std::vector<float> make_frame(uint32_t prn, const std::array<uint8_t, BEIDOU_CNAV3_INFO_BITS>& info)
{
    std::vector<float> symbols(static_cast<size_t>(BEIDOU_CNAV3_FRAME_SYMBOLS), -1.0F);
    const char* preamble = "1110101110010000";
    for (int i = 0; i < 16; i++)
        {
            symbols[static_cast<size_t>(i)] = (preamble[i] == '1') ? 1.0F : -1.0F;
        }
    for (int i = 0; i < 6; i++)
        {
            const int bit = static_cast<int>((prn >> (5 - i)) & 1U);
            symbols[static_cast<size_t>(16 + i)] = bit ? 1.0F : -1.0F;
        }
    const int info0 = 16 + 6 + 6;
    for (int i = 0; i < BEIDOU_CNAV3_INFO_BITS; i++)
        {
            symbols[static_cast<size_t>(info0 + i)] = info[static_cast<size_t>(i)] ? 1.0F : -1.0F;
        }
    return symbols;
}

std::array<uint8_t, BEIDOU_CNAV3_INFO_BITS> make_info(int32_t mes_type, uint32_t sow)
{
    std::array<uint8_t, BEIDOU_CNAV3_INFO_BITS> info{};
    write_unsigned(info.data(), 0, 6, static_cast<uint64_t>(mes_type));
    write_unsigned(info.data(), 6, 20, sow);
    if (mes_type == BEIDOU_CNAV3_MSG_CLK)
        {
            write_unsigned(info.data(), 26, 13, 900);  // WN
        }
    const uint32_t crc = crc24q(info.data(), BEIDOU_CNAV3_DATA_BITS);
    write_unsigned(info.data(), BEIDOU_CNAV3_DATA_BITS, BEIDOU_CNAV3_CRC_BITS, crc);
    return info;
}
}  // namespace


TEST(BeidouCnav3NavigationMessageTest, AcceptsValidCrcAndPreamble)
{
    const auto info = make_info(BEIDOU_CNAV3_MSG_EPH, 12345);
    const auto frame = make_frame(19, info);
    Beidou_Cnav3_Navigation_Message nav;
    ASSERT_TRUE(nav.decode_frame_symbols(frame.data(), static_cast<int32_t>(frame.size()), 19));
    EXPECT_TRUE(nav.last_crc_ok());
    EXPECT_EQ(nav.last_mes_type(), BEIDOU_CNAV3_MSG_EPH);
    EXPECT_EQ(nav.last_sow(), 12345);
    EXPECT_EQ(nav.last_frame_prn(), 19U);
    EXPECT_FALSE(nav.have_new_ephemeris());  // MT30 still missing
}


TEST(BeidouCnav3NavigationMessageTest, RejectsBadCrc)
{
    auto info = make_info(BEIDOU_CNAV3_MSG_EPH, 1000);
    info[100] ^= 1U;
    const auto frame = make_frame(19, info);
    Beidou_Cnav3_Navigation_Message nav;
    EXPECT_FALSE(nav.decode_frame_symbols(frame.data(), static_cast<int32_t>(frame.size()), 19));
}


TEST(BeidouCnav3NavigationMessageTest, EmitsEphemerisAfterMt10AndMt30)
{
    Beidou_Cnav3_Navigation_Message nav;
    const auto mt10 = make_frame(21, make_info(BEIDOU_CNAV3_MSG_EPH, 2000));
    const auto mt30 = make_frame(21, make_info(BEIDOU_CNAV3_MSG_CLK, 2001));
    ASSERT_TRUE(nav.decode_frame_symbols(mt10.data(), static_cast<int32_t>(mt10.size()), 21));
    EXPECT_FALSE(nav.have_new_ephemeris());
    ASSERT_TRUE(nav.decode_frame_symbols(mt30.data(), static_cast<int32_t>(mt30.size()), 21));
    ASSERT_TRUE(nav.have_new_ephemeris());
    const auto eph = nav.get_ephemeris();
    EXPECT_EQ(eph.PRN, 21U);
    EXPECT_EQ(eph.WN, 900);
    EXPECT_EQ(eph.sig_type, BDS_EPH_SOURCE_CNAV3);
}


TEST(BeidouCnav3NavigationMessageTest, GeoDoesNotEmitEphemeris)
{
    Beidou_Cnav3_Navigation_Message nav;
    const auto mt10 = make_frame(59, make_info(BEIDOU_CNAV3_MSG_EPH, 3000));
    const auto mt30 = make_frame(59, make_info(BEIDOU_CNAV3_MSG_CLK, 3001));
    ASSERT_TRUE(nav.decode_frame_symbols(mt10.data(), static_cast<int32_t>(mt10.size()), 59));
    ASSERT_TRUE(nav.decode_frame_symbols(mt30.data(), static_cast<int32_t>(mt30.size()), 59));
    EXPECT_FALSE(nav.have_new_ephemeris());
}


TEST(BeidouCnav3NavigationMessageTest, DecodesConsecutiveMt10Mt30Frames)
{
    Beidou_Cnav3_Navigation_Message nav;
    int eph_count = 0;
    for (int k = 0; k < 12; k++)
        {
            const int32_t sow0 = 4000 + 2 * k;
            const auto mt10 = make_frame(21, make_info(BEIDOU_CNAV3_MSG_EPH, static_cast<uint32_t>(sow0)));
            const auto mt30 = make_frame(21, make_info(BEIDOU_CNAV3_MSG_CLK, static_cast<uint32_t>(sow0 + 1)));
            ASSERT_TRUE(nav.decode_frame_symbols(mt10.data(), BEIDOU_CNAV3_FRAME_SYMBOLS, 21));
            ASSERT_TRUE(nav.decode_frame_symbols(mt30.data(), BEIDOU_CNAV3_FRAME_SYMBOLS, 21));
            if (nav.have_new_ephemeris())
                {
                    eph_count++;
                    EXPECT_EQ(nav.get_ephemeris().sig_type, BDS_EPH_SOURCE_CNAV3);
                    nav.clear_flags();
                }
        }
    EXPECT_GE(eph_count, 12);
}
