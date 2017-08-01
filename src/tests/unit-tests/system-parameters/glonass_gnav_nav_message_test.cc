/*!
 * \file glonass_gnav_navigation_message_test.cc
 * \brief  This file implements tests for the decoding of the GLONASS GNAV navigation message
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include <complex>
#include <ctime>
#include <complex>
#include <ctime>
#include "gnss_signal_processing.h"
#include "glonass_gnav_navigation_message.h"

/*!
 * \brief Testing CRC computation for GLONASS GNAV data bits of a string
 * \test The provided string was generated with a version of MATLAB GNSS-SDR that
 * the author coded to perform proper decoding of GLONASS GNAV signals.
 */
TEST(GlonassGnavNavigationMessageTest, CRCTest)
{
    // Variables declarations in code
    bool test_result;
    std::string str5("0010100100001100000000000000000000000000110011110001100000000000000001100100011000000");
    Glonass_Gnav_Navigation_Message gnav_nav_message;
    gnav_nav_message.reset();

    // Call function to test
    test_result = gnav_nav_message.CRC_test(std::bitset<GLONASS_GNAV_STRING_BITS> (str5));

    // Check results in unit test assetions
    ASSERT_TRUE(test_result);
}

/*!
 * \brief Testing string decoding for GLONASS GNAV messages
 * \test The provided string (str1.....str15) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of GLONASS
 * GNAV signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(GlonassGnavNavigationMessageTest, String1Decoder)
{
    // Variable declarations
    std::string str1("0000100000001000011001000011111011010101110100000010101011000100011010101011000101111");
    Glonass_Gnav_Navigation_Message gnav_nav_message;
    Glonass_Gnav_Ephemeris gnav_ephemeris;

    // Fill out ephemeris values for truth
    gnav_ephemeris.d_P_1    = static_cast<double>(read_navigation_unsigned(string_bits, P1));
    gnav_ephemeris.d_t_k    =  static_cast<double>(read_navigation_unsigned(string_bits, T_K_HR)) * 3600 +
    gnav_ephemeris.d_VXn    = static_cast<double>(read_navigation_signed(string_bits, X_N_DOT)) * 2e-20;
    gnav_ephemeris.d_AXn    = static_cast<double>(read_navigation_signed(string_bits, X_N_DOT_DOT)) * 2e-30;
    gnav_ephemeris.d_Xn     = static_cast<double>(read_navigation_signed(string_bits, X_N)) * 2e-11;

    // Call target test method
    gnav_nav_message.string_decoder(str1.c_str());

    // Perform assertions of decoded fields
    ASSERT_TRUE(gnav_ephemeris.d_t_k - gnav_nav_message.gnav_ephemeris.d_t_k < DBL_EPSILON );
    ASSERT_TRUE(gnav_ephemeris.d_VXn - gnav_nav_message.gnav_ephemeris.d_VXn < DBL_EPSILON );
    ASSERT_TRUE(gnav_ephemeris.d_AXn - gnav_nav_message.gnav_ephemeris.d_AXn < DBL_EPSILON );
    ASSERT_TRUE(gnav_ephemeris.d_Xn -  gnav_nav_message.gnav_ephemeris.d_Xn < DBL_EPSILON );
}

/*!
 * \brief Testing string decoding for GLONASS GNAV messages
 * \test The provided string (str1.....str15) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of GLONASS
 * GNAV signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(GlonassGnavNavigationMessageTest, String2Decoder)
{
    // Variable declarations
    std::string str2("0001000010001001000001010101100001011001011000000010101100110000001011110000110011110");
    Glonass_Gnav_Navigation_Message gnav_nav_message;
    Glonass_Gnav_Ephemeris gnav_ephemeris;

    // Fill out ephemeris values for truth
    gnav_ephemeris.d_B_n = static_cast<double>(read_navigation_unsigned(string_bits, B_N));
    gnav_ephemeris.d_P_2 = static_cast<double>(read_navigation_unsigned(string_bits, P2));
    gnav_ephemeris.d_t_b = static_cast<double>(read_navigation_unsigned(string_bits, T_B))*15*60;
    gnav_ephemeris.d_VYn = static_cast<double>(read_navigation_signed(string_bits, Y_N_DOT))* 2e-20;
    gnav_ephemeris.d_AYn = static_cast<double>(read_navigation_signed(string_bits, Y_N_DOT_DOT)) * 2e-30;
    gnav_ephemeris.d_Yn  = static_cast<double>(read_navigation_signed(string_bits, X_N)) * 2e-11;

    // Call target test method
    gnav_nav_message.string_decoder(str2.c_str())

    // Perform assertions of decoded fields
    ASSERT_TRUE(gnav_ephemeris.d_B_n - gnav_nav_message.gnav_ephemeris.d_B_n < DBL_EPSILON );
    ASSERT_TRUE(gnav_ephemeris.d_P_2 - gnav_nav_message.gnav_ephemeris.d_P_2 < DBL_EPSILON );
    ASSERT_TRUE(gnav_ephemeris.d_t_b - gnav_nav_message.gnav_ephemeris.d_t_b < DBL_EPSILON );
    ASSERT_TRUE(gnav_ephemeris.d_VYn -  gnav_nav_message.gnav_ephemeris.d_VYn < DBL_EPSILON );
    ASSERT_TRUE(gnav_ephemeris.d_AYn - gnav_nav_message.gnav_ephemeris.d_AYn < DBL_EPSILON );
    ASSERT_TRUE(gnav_ephemeris.d_Yn -  gnav_nav_message.gnav_ephemeris.d_Yn < DBL_EPSILON );
}

std::string str2("0001000010001001000001010101100001011001011000000010101100110000001011110000110011110");
std::string str3("0001110000000001001101001110100011111011010011001101001101110110010011110011100100011");
std::string str4("0010010000101011100100000100000100000000000000000000011000100100001100101010100011101");
std::string str5("0010100100001100000000000000000000000000110011110001100000000000000001100100011000000");
std::string str6("0011010100110100001100111100011100001101011000000110101111001000000101100011111011001");
std::string str7("0011101101010001000010000110101111110000101101001011111110101110100010111100010001101");
std::string str8("0100010100111000000001111110001101000000110000001000100111011100001010101111010011010");
std::string str9("0100111010001001011100010000010100010101111101001011111110101011100010100101000110101");
std::string str10("0101010101000000000011101111111101111001011000001000101010001100001111000110101111110");
std::string str11("0101110111011011011100011001111011101111001101001011111111000110100100000110010001111");
std::string str12("0110010101001100000011110110100110100100010100001000111110000100001110001010111000001");
std::string str13("0110111011100100111110100001000110100010011101001011111110100100101010011010001101001");
std::string str14("0111010101010000000100011000011110100110111100001110110100001000001111001101010000101");
std::string str15("0111101110101010001110101010100111101100001101001011111111100010101010011001010011101");
