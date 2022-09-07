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
#include <iostream>

void to_bitset(const uint32_t from[3], std::bitset<GLONASS_GNAV_STRING_BITS> &to)
{
    to.reset();
    for (int k = 0; k < 3; k++)
        {
            std::bitset<GLONASS_GNAV_STRING_BITS> tmp(from[k]);
            to |= (tmp << (k * 32));
        }
}


TEST(GlonassCrcTest, GnssSdrCRCTest)
{
    // test data
    std::string string1Real_14_18_00("0000100000111001001001000011101010101100010101001000001001011101010101110011110110101");
    std::string string1Real_14_18_30("0000100000111001001011000011101010101100010101001000001001011101010101110011100001010");
    std::string string1Wrong_14_18_00("0000100000111001001001100011101010101100010101001000001001011101010101110011100001010");
    const uint32_t test_case_good[][3] = {
        /* Test data from libswiftnav (github.com/swift-nav/libswiftnav) */
        /* First, simply test one GLO nav message received from Novatel,
         * we trust Novatel, so no errors must be */
        {0xc90cfb3e, 0x9743a301, 0x010749},
        {0xdd39f5fc, 0x24542d0c, 0x021760},
        {0x653bc7e9, 0x1e8ead92, 0x038006},
        {0x60342dfc, 0x41000002, 0x0481c7},
        {0x40000895, 0x00000003, 0x050d10},
        {0x530a7ecf, 0x059c4415, 0x06b082},
        {0xfd94beb6, 0x7a577e97, 0x070f46},
        {0xba02de6f, 0x988e6814, 0x08b101},
        {0x12064831, 0x87767698, 0x09e1a6},
        {0xaf870be5, 0x54ef2617, 0x0ab286},
        {0x0f06ba41, 0x9a3f2698, 0x0b8f7c},
        {0x2f012204, 0xf0c3c81a, 0x0cb309},
        {0x1c858601, 0x10c47e98, 0x0da065},
        {0x5205980b, 0xf49abc1a, 0x0eb40e},
        {0x15454437, 0x2504e698, 0x0f8c09},
        /* Second, take 1st string from other GLO nav message and introduce an
         * error in data bits */
        {0xc90cfb81, 0x9743a301, 0x010748}, /* case 15, no errors  */
    };
    const uint32_t test_case_correctable[][3] = {
        /* single bit errors are correctable */
        {0xc90cfb81, 0x9743a301, 0x110748},
        {0xc90cfb81, 0x1743a301, 0x010748},
        {0x490cfb81, 0x9743a301, 0x010748},
        {0xc90cfb81, 0x9743a300, 0x010748},
        {0xc90cfb81, 0x9743a301, 0x010749},
        {0xc90cfb81, 0x9743a301, 0x000748},
    };
    const uint32_t test_case_uncorrectable[][3] = {
        /* multiple bit errors are uncorrectable */
        {0xc90c3b81, 0x9743a301, 0x010748},
        {0xc90cfb81, 0x974fa301, 0x010748},
        {0xc90cfb81, 0x9743a301, 0x01074b},
        {0xc90cfb81, 0x9743a301, 0x010744},
        {0xc90cfb81, 0x9aaaa301, 0x010748},
    };
    auto gnav_msg = Glonass_Gnav_Navigation_Message();
    std::bitset<GLONASS_GNAV_STRING_BITS> bits;
    bits = std::bitset<GLONASS_GNAV_STRING_BITS>(string1Real_14_18_00);
    ASSERT_TRUE(gnav_msg.CRC_test(bits));
    bits = std::bitset<GLONASS_GNAV_STRING_BITS>(string1Real_14_18_30);
    ASSERT_TRUE(gnav_msg.CRC_test(bits));
    bits = std::bitset<GLONASS_GNAV_STRING_BITS>(string1Wrong_14_18_00);
    ASSERT_FALSE(gnav_msg.CRC_test(bits));
    bits = std::bitset<GLONASS_GNAV_STRING_BITS>(string1Real_14_18_30);
    for (int k = 8; k < 85; k++)
        {
            std::bitset<GLONASS_GNAV_STRING_BITS> corrupt_bits = bits;
            corrupt_bits[k] = corrupt_bits[k] ? false : true;
            ASSERT_TRUE(gnav_msg.CRC_test(corrupt_bits));
            ASSERT_TRUE(corrupt_bits == bits);
        }
    for (unsigned k = 0; k < sizeof(test_case_good) / sizeof(test_case_good[0]); k++)
        {
            to_bitset(test_case_good[k], bits);
            ASSERT_TRUE(gnav_msg.CRC_test(bits));
        }
    for (unsigned k = 0; k < sizeof(test_case_correctable) / sizeof(test_case_correctable[0]); k++)
        {
            std::bitset<GLONASS_GNAV_STRING_BITS> corrupt_bits;
            to_bitset(test_case_correctable[k], corrupt_bits);
            ASSERT_TRUE(gnav_msg.CRC_test(corrupt_bits));
            ASSERT_TRUE(corrupt_bits == bits);
        }
    for (unsigned k = 0; k < sizeof(test_case_uncorrectable) / sizeof(test_case_uncorrectable[0]); k++)
        {
            std::bitset<GLONASS_GNAV_STRING_BITS> corrupt_bits;
            to_bitset(test_case_uncorrectable[k], corrupt_bits);
            ASSERT_FALSE(gnav_msg.CRC_test(corrupt_bits));
        }
}
