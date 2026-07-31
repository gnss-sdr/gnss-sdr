/*!
 * \file beidou_dnav_navigation_message_test.cc
 * \brief Tests for BeiDou DNAV navigation message consistency checks
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
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

#include "Beidou_DNAV.h"
#include "beidou_dnav_navigation_message.h"
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace
{
void set_unsigned_field(std::string& bits, const std::vector<std::pair<int32_t, int32_t>>& field, uint64_t value)
{
    int32_t field_length = 0;
    for (const auto& part : field)
        {
            field_length += part.second;
        }

    int32_t bit_offset = 0;
    for (const auto& part : field)
        {
            for (int32_t j = 0; j < part.second; ++j)
                {
                    const auto value_bit = static_cast<uint64_t>(1ULL << (field_length - bit_offset - 1));
                    bits[static_cast<std::size_t>(part.first + j - 1)] = ((value & value_bit) != 0U) ? '1' : '0';
                    ++bit_offset;
                }
        }
}


std::string make_d2_subframe1_page(int32_t page_ID, uint32_t sow)
{
    std::string subframe(BEIDOU_DNAV_SUBFRAME_DATA_BITS, '0');
    subframe.replace(0, BEIDOU_DNAV_PREAMBLE_LENGTH_BITS, BEIDOU_DNAV_PREAMBLE);
    set_unsigned_field(subframe, D2_FRAID, 1);
    set_unsigned_field(subframe, D2_SOW, sow);
    set_unsigned_field(subframe, D2_PNUM, static_cast<uint64_t>(page_ID));
    return subframe;
}
}  // namespace


TEST(BeidouDnavNavigationMessageTest, D2Subframe1PagesAcceptContiguousSow)
{
    auto dnav_msg = Beidou_Dnav_Navigation_Message();
    dnav_msg.set_satellite_PRN(1);

    constexpr uint32_t first_sow = 1000;
    for (int32_t page_ID = 1; page_ID <= 10; ++page_ID)
        {
            const auto sow = first_sow + static_cast<uint32_t>((page_ID - 1) * BEIDOU_DNAV_D2_SUBFRAME1_PAGE_PERIOD_SECONDS);
            EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(page_ID, sow)));
            EXPECT_TRUE(dnav_msg.get_flag_CRC_test());
            EXPECT_EQ(static_cast<double>(sow), dnav_msg.get_SOW());
        }
}


TEST(BeidouDnavNavigationMessageTest, D2Subframe1PageRejectsUnexpectedSow)
{
    auto dnav_msg = Beidou_Dnav_Navigation_Message();
    dnav_msg.set_satellite_PRN(1);

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(1, 1000)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());

    EXPECT_EQ(0, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(2, 1004)));
    EXPECT_FALSE(dnav_msg.get_flag_CRC_test());
    EXPECT_EQ(1000.0, dnav_msg.get_SOW());

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(1, 2000)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());
    EXPECT_EQ(2000.0, dnav_msg.get_SOW());
}


TEST(BeidouDnavNavigationMessageTest, D2Subframe1PageRejectsUnexpectedPageOrder)
{
    auto dnav_msg = Beidou_Dnav_Navigation_Message();
    dnav_msg.set_satellite_PRN(1);

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(1, 1000)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());

    EXPECT_EQ(0, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(3, 1003)));
    EXPECT_FALSE(dnav_msg.get_flag_CRC_test());
}


TEST(BeidouDnavNavigationMessageTest, D2Subframe1PageRejectsSowOutOfRange)
{
    auto dnav_msg = Beidou_Dnav_Navigation_Message();
    dnav_msg.set_satellite_PRN(1);

    EXPECT_EQ(0, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(1, 604800)));
    EXPECT_FALSE(dnav_msg.get_flag_CRC_test());
}


TEST(BeidouDnavNavigationMessageTest, D2Subframe1PagesAcceptSowWeekRollover)
{
    auto dnav_msg = Beidou_Dnav_Navigation_Message();
    dnav_msg.set_satellite_PRN(1);

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(1, 604797)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(2, 0)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());

    EXPECT_EQ(1, dnav_msg.d2_subframe_decoder(make_d2_subframe1_page(3, 3)));
    EXPECT_TRUE(dnav_msg.get_flag_CRC_test());
}


TEST(BeidouDnavNavigationMessageTest, D1ToeAndSqrtAIcdLimits)
{
    // ICD B1I Table 5-10: toe max 604792 s, sqrt(A) max 8192 m^1/2.
    // The corrupt PRN7 DNAV case (toe=988032, sqrtA≈1698) must be rejected.
    EXPECT_GT(988032.0, D1_TOE_MAX_S);
    EXPECT_LT(1698.0, D1_SQRT_A_MIN_SANE);
    EXPECT_LE(201600.0, D1_TOE_MAX_S);
    EXPECT_GE(5282.0, D1_SQRT_A_MIN_SANE);
    EXPECT_LE(6493.0, D1_SQRT_A_MAX);
}
