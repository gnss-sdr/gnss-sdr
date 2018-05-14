/*!
 * \file glonass_gnav_navigation_message_test.cc
 * \brief  This file implements tests for the decoding of the GLONASS GNAV navigation message
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "gnss_signal_processing.h"
#include "glonass_gnav_navigation_message.h"

/*!
 * \brief Testing CRC computation for GLONASS GNAV data bits of a string
 * \test The provided string was generated with a version of MATLAB GNSS-SDR that
 * the author coded to perform proper decoding of GLONASS GNAV signals.
 */
TEST(GlonassGnavNavigationMessageTest, CRCTestSuccess)
{
    // Variables declarations in code
    bool test_result;
    std::bitset<GLONASS_GNAV_STRING_BITS> string_bits(std::string("0010100100001100000000000000000000000000110011110001100000000000000001100100011000000"));
    Glonass_Gnav_Navigation_Message gnav_nav_message;
    gnav_nav_message.reset();

    // Call function to test
    test_result = gnav_nav_message.CRC_test(string_bits);

    // Check results in unit test assetions
    ASSERT_TRUE(test_result);
}


/*!
 * \brief Testing CRC computation for GLONASS GNAV data bits of a string
 * \test The provided string was generated with a version of MATLAB GNSS-SDR that
 * the author coded to perform proper decoding of GLONASS GNAV signals.
 */
TEST(GlonassGnavNavigationMessageTest, CRCTestFailure)
{
    // Variables declarations in code
    bool test_result;
    // Constructor of string to bitset will flip the order of the bits. Needed for CRC computation
    std::bitset<GLONASS_GNAV_STRING_BITS> string_bits(std::string("0111100100001100000000000000000000000000110011110001100000000000000001100100011000000"));
    Glonass_Gnav_Navigation_Message gnav_nav_message;
    gnav_nav_message.reset();

    // Call function to test
    test_result = gnav_nav_message.CRC_test(string_bits);

    // Check results in unit test assetions
    ASSERT_FALSE(test_result);
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
    gnav_ephemeris.d_P_1 = 15;
    gnav_ephemeris.d_t_k = 7560;
    gnav_ephemeris.d_VXn = -0.490900039672852;
    gnav_ephemeris.d_AXn = 0;
    gnav_ephemeris.d_Xn = -11025.6669921875;

    // Call target test method
    gnav_nav_message.string_decoder(str1);

    // Perform assertions of decoded fields
    ASSERT_TRUE(gnav_ephemeris.d_P_1 - gnav_nav_message.gnav_ephemeris.d_P_1 < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_t_k - gnav_nav_message.gnav_ephemeris.d_t_k < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_VXn - gnav_nav_message.gnav_ephemeris.d_VXn < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_AXn - gnav_nav_message.gnav_ephemeris.d_AXn < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_Xn - gnav_nav_message.gnav_ephemeris.d_Xn < FLT_EPSILON);
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
    gnav_ephemeris.d_B_n = 0;
    gnav_ephemeris.d_P_2 = 1;
    gnav_ephemeris.d_t_b = 8100;
    gnav_ephemeris.d_VYn = -2.69022750854492;
    gnav_ephemeris.d_AYn = 0;
    gnav_ephemeris.d_Yn = -11456.7348632812;

    // Call target test method
    gnav_nav_message.flag_ephemeris_str_1 = true;
    gnav_nav_message.gnav_ephemeris.d_P_1 = 15;
    gnav_nav_message.string_decoder(str2);

    // Perform assertions of decoded fields
    ASSERT_TRUE(gnav_ephemeris.d_B_n - gnav_nav_message.gnav_ephemeris.d_B_n < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_P_2 - gnav_nav_message.gnav_ephemeris.d_P_2 < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_t_b - gnav_nav_message.gnav_ephemeris.d_t_b < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_VYn - gnav_nav_message.gnav_ephemeris.d_VYn < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_AYn - gnav_nav_message.gnav_ephemeris.d_AYn < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_Yn - gnav_nav_message.gnav_ephemeris.d_Yn < FLT_EPSILON);
}


/*!
 * \brief Testing string decoding for GLONASS GNAV messages
 * \test The provided string (str1.....str15) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of GLONASS
 * GNAV signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(GlonassGnavNavigationMessageTest, String3Decoder)
{
    // Variable declarations
    std::string str3("0001110000000001001101001110100011111011010011001101001101110110010011110011100100011");
    Glonass_Gnav_Navigation_Message gnav_nav_message;
    Glonass_Gnav_Ephemeris gnav_ephemeris;

    // Fill out ephemeris values for truth
    gnav_ephemeris.d_P_3 = 1;
    gnav_ephemeris.d_gamma_n = 1.81898940354586e-12;
    gnav_ephemeris.d_P = 3;
    gnav_ephemeris.d_l3rd_n = 0;
    gnav_ephemeris.d_VZn = -1.82016849517822;
    gnav_ephemeris.d_AZn = -2.79396772384644e-09;
    gnav_ephemeris.d_Zn = 19929.2377929688;

    // Call target test method
    gnav_nav_message.flag_ephemeris_str_2 = true;
    gnav_nav_message.string_decoder(str3);

    // Perform assertions of decoded fields
    ASSERT_TRUE(gnav_ephemeris.d_P_3 - gnav_nav_message.gnav_ephemeris.d_P_3 < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_gamma_n - gnav_nav_message.gnav_ephemeris.d_gamma_n < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_P - gnav_nav_message.gnav_ephemeris.d_P < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_l3rd_n - gnav_nav_message.gnav_ephemeris.d_l3rd_n < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_VZn - gnav_nav_message.gnav_ephemeris.d_VZn < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_AZn - gnav_nav_message.gnav_ephemeris.d_AZn < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_Zn - gnav_nav_message.gnav_ephemeris.d_Zn < FLT_EPSILON);
}


/*!
 * \brief Testing string decoding for GLONASS GNAV messages
 * \test The provided string (str1.....str15) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of GLONASS
 * GNAV signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(GlonassGnavNavigationMessageTest, String4Decoder)
{
    // Variable declarations
    std::string str4("0010010000101011100100000100000100000000000000000000011000100100001100101010100011101");
    Glonass_Gnav_Navigation_Message gnav_nav_message;
    Glonass_Gnav_Ephemeris gnav_ephemeris;

    // Fill out ephemeris values for truth
    gnav_ephemeris.d_tau_n = -8.30907374620438e-05;
    gnav_ephemeris.d_Delta_tau_n = 9.31322574615479e-10;
    gnav_ephemeris.d_E_n = 0;
    gnav_ephemeris.d_P_4 = 0;
    gnav_ephemeris.d_F_T = 6;
    gnav_ephemeris.d_N_T = 268;
    gnav_ephemeris.d_n = 21;
    gnav_ephemeris.d_M = 1;

    // Call target test method
    gnav_nav_message.flag_ephemeris_str_3 = true;
    gnav_nav_message.string_decoder(str4);

    // Perform assertions of decoded fields
    ASSERT_TRUE(gnav_ephemeris.d_tau_n - gnav_nav_message.gnav_ephemeris.d_tau_n < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_Delta_tau_n - gnav_nav_message.gnav_ephemeris.d_Delta_tau_n < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_E_n - gnav_nav_message.gnav_ephemeris.d_E_n < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_P_4 - gnav_nav_message.gnav_ephemeris.d_P_4 < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_F_T - gnav_nav_message.gnav_ephemeris.d_F_T < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_N_T - gnav_nav_message.gnav_ephemeris.d_N_T < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_n - gnav_nav_message.gnav_ephemeris.d_n < FLT_EPSILON);
    ASSERT_TRUE(gnav_ephemeris.d_M - gnav_nav_message.gnav_ephemeris.d_M < FLT_EPSILON);
}


/*!
 * \brief Testing string decoding for GLONASS GNAV messages
 * \test The provided string (str1.....str15) was generated with a version of
 * MATLAB GNSS-SDR that the author coded to perform proper decoding of GLONASS
 * GNAV signals. The same assumption is to be applied for ephemeris and almanac
 * data provided.
 */
TEST(GlonassGnavNavigationMessageTest, String5Decoder)
{
    // Variable declarations
    std::string str5("0010100100001100000000000000000000000000110011110001100000000000000001100100011000000");
    Glonass_Gnav_Navigation_Message gnav_nav_message;
    Glonass_Gnav_Utc_Model gnav_utc_model;

    // Fill out ephemeris values for truth
    gnav_utc_model.d_N_A = 268;
    gnav_utc_model.d_tau_c = 9.6391886472702e-08;
    gnav_utc_model.d_N_4 = 6;
    gnav_utc_model.d_tau_gps = 9.313225746154785e-08;

    // Call target test method
    gnav_nav_message.flag_ephemeris_str_4 = true;
    gnav_nav_message.string_decoder(str5);

    // Perform assertions of decoded fields
    ASSERT_TRUE(gnav_utc_model.d_N_A - gnav_nav_message.gnav_utc_model.d_N_A < FLT_EPSILON);
    ASSERT_TRUE(gnav_utc_model.d_tau_c - gnav_nav_message.gnav_utc_model.d_tau_c < FLT_EPSILON);
    ASSERT_TRUE(gnav_utc_model.d_N_4 - gnav_nav_message.gnav_utc_model.d_N_4 < FLT_EPSILON);
    ASSERT_TRUE(gnav_utc_model.d_tau_gps - gnav_nav_message.gnav_utc_model.d_tau_gps < FLT_EPSILON);
}

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
