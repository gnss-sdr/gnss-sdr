/*!
 * \file beidou_dnav_navigation_message_test.cc
 * \brief  Tests for BeiDou DNAV navigation message parity checking
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

#include "beidou_dnav_navigation_message.h"

namespace
{
std::string valid_dnav_subframe()
{
    const std::string word1 =
        "11100010010"  // preamble
        "0000"         // reserved bits
        "00100000000"  // FraID = 1, SOW MSBs = 0
        "1111";        // BCH parity for the previous 11 bits

    const std::string word =
        "10101010101"  // first BCH block data
        "01010101010"  // second BCH block data
        "1011"         // first BCH block parity
        "0100";        // second BCH block parity

    std::string subframe = word1;
    for (uint32_t i = 1; i < BEIDOU_DNAV_WORDS_SUBFRAME; ++i)
        {
            subframe += word;
        }
    return subframe;
}
}  // namespace


TEST(BeidouDnavNavigationMessageTest, CRCTestSuccess)
{
    const std::string subframe = valid_dnav_subframe();
    const Beidou_Dnav_Navigation_Message dnav_nav_message;

    ASSERT_TRUE(dnav_nav_message.CRC_test(subframe));
}


TEST(BeidouDnavNavigationMessageTest, CRCTestFailure)
{
    std::string subframe = valid_dnav_subframe();
    Beidou_Dnav_Navigation_Message dnav_nav_message;

    subframe[29] = (subframe[29] == '0') ? '1' : '0';

    ASSERT_FALSE(dnav_nav_message.CRC_test(subframe));
    ASSERT_EQ(0, dnav_nav_message.d1_subframe_decoder(subframe));
    ASSERT_FALSE(dnav_nav_message.get_flag_CRC_test());
}


TEST(BeidouDnavNavigationMessageTest, D1DecoderAcceptsValidParity)
{
    const std::string subframe = valid_dnav_subframe();
    Beidou_Dnav_Navigation_Message dnav_nav_message;

    ASSERT_EQ(1, dnav_nav_message.d1_subframe_decoder(subframe));
    ASSERT_TRUE(dnav_nav_message.get_flag_CRC_test());
}
