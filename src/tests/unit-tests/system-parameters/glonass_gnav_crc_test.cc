/*!
 * \file glonass_gnav_crc_test.cc
 * \brief Test fot GLONASS GNAV CRC
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "glonass_gnav_navigation_message.h"
#include <bitset>
#include <string>

TEST(GlonassCrcTest, GnssSdrCRCTest)
{
    // test data
    std::string string1Real_14_18_00("0000100000111001001001000011101010101100010101001000001001011101010101110011110110101");
    std::string string1Real_14_18_30("0000100000111001001011000011101010101100010101001000001001011101010101110011100001010");
    std::string string1Wrong_14_18_00("0000100000111001001001000011101010101100010101001000001001011101010101110011100001010");

    auto gnav_msg = Glonass_Gnav_Navigation_Message();
    std::bitset<GLONASS_GNAV_STRING_BITS> bits;
    bits = std::bitset<GLONASS_GNAV_STRING_BITS>(string1Real_14_18_00);
    ASSERT_TRUE(gnav_msg.CRC_test(bits));
    bits = std::bitset<GLONASS_GNAV_STRING_BITS>(string1Real_14_18_30);
    ASSERT_TRUE(gnav_msg.CRC_test(bits));
    bits = std::bitset<GLONASS_GNAV_STRING_BITS>(string1Wrong_14_18_00);
    ASSERT_FALSE(gnav_msg.CRC_test(bits));
}
