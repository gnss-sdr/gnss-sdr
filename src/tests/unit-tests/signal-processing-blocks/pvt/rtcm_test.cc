/*!
 * \file rtcm_test.cc
 * \brief  This file implements unit tests for the Rtcm class.
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
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

#include <memory>
#include <thread>
#include "rtcm.h"
#include "Galileo_E1.h"

TEST(RtcmTest, HexToBin)
{
    auto rtcm = std::make_shared<Rtcm>();

    std::string test1 = "2A";
    std::string test1_bin = rtcm->hex_to_bin(test1);
    EXPECT_EQ(0, test1_bin.compare("00101010"));

    std::string test2 = "FF";
    std::string test2_bin = rtcm->hex_to_bin(test2);
    EXPECT_EQ(0, test2_bin.compare("11111111"));

    std::string test3 = "ff";
    std::string test3_bin = rtcm->hex_to_bin(test3);
    EXPECT_EQ(0, test3_bin.compare("11111111"));

    std::string test4 = "100";
    std::string test4_bin = rtcm->hex_to_bin(test4);
    EXPECT_EQ(0, test4_bin.compare("000100000000"));

    std::string test5 = "1101";
    std::string test5_bin = rtcm->hex_to_bin(test5);
    EXPECT_EQ(0, test5_bin.compare("0001000100000001"));

    std::string test6 = "3";
    std::string test6_bin = rtcm->hex_to_bin(test6);
    EXPECT_EQ(0, test6_bin.compare("0011"));
}


TEST(RtcmTest, BinToHex)
{
    auto rtcm = std::make_shared<Rtcm>();

    std::string test1 = "00101010";
    std::string test1_hex = rtcm->bin_to_hex(test1);
    EXPECT_EQ(0, test1_hex.compare("2A"));

    std::string test2 = "11111111";
    std::string test2_hex = rtcm->bin_to_hex(test2);
    EXPECT_EQ(0, test2_hex.compare("FF"));

    std::string test4 = "000100000000";
    std::string test4_hex = rtcm->bin_to_hex(test4);
    EXPECT_EQ(0, test4_hex.compare("100"));

    std::string test5 = "0001000100000001";
    std::string test5_hex = rtcm->bin_to_hex(test5);
    EXPECT_EQ(0, test5_hex.compare("1101"));

    std::string test6 = "0011";
    std::string test6_hex = rtcm->bin_to_hex(test6);
    EXPECT_EQ(0, test6_hex.compare("3"));

    std::string test7 = "11";
    std::string test7_hex = rtcm->bin_to_hex(test7);
    EXPECT_EQ(0, test7_hex.compare("3"));

    std::string test8 = "1000100000001";
    std::string test8_hex = rtcm->bin_to_hex(test8);
    EXPECT_EQ(0, test8_hex.compare("1101"));
}


TEST(RtcmTest, HexToInt)
{
    auto rtcm = std::make_shared<Rtcm>();

    std::string test1 = "2A";
    long int test1_int = rtcm->hex_to_int(test1);
    long int expected1 = 42;
    EXPECT_EQ(expected1, test1_int);
}


TEST(RtcmTest, HexToUint)
{
    auto rtcm = std::make_shared<Rtcm>();
    long unsigned int expected1 = 42;
    EXPECT_EQ(expected1, rtcm->hex_to_uint(rtcm->bin_to_hex("00101010")));
}


TEST(RtcmTest, BinToDouble)
{
    auto rtcm = std::make_shared<Rtcm>();

    std::bitset<4> test1(5);
    long int test1_int = static_cast<long int>(rtcm->bin_to_double(test1.to_string()));
    long int expected1 = 5;
    EXPECT_EQ(expected1, test1_int);

    std::bitset<4> test2(-5);
    EXPECT_DOUBLE_EQ(-5, rtcm->bin_to_double(test2.to_string()));

    std::bitset<65> test3(-5);
    EXPECT_DOUBLE_EQ(0, rtcm->bin_to_double(test3.to_string()));
}


TEST(RtcmTest, BinToUint)
{
    auto rtcm = std::make_shared<Rtcm>();
    long unsigned int expected1 = 42;
    EXPECT_EQ(expected1, rtcm->bin_to_uint("00101010"));
    long unsigned int expected2 = 214;
    EXPECT_EQ(expected2, rtcm->bin_to_uint("11010110"));
}


TEST(RtcmTest, BinToInt)
{
    auto rtcm = std::make_shared<Rtcm>();
    long int expected1 = 42;
    EXPECT_EQ(expected1, rtcm->bin_to_int("00101010"));
    long int expected2 = -42;
    EXPECT_EQ(expected2, rtcm->bin_to_int("11010110"));
}


TEST(RtcmTest, BinToBinaryData)
{
    auto rtcm = std::make_shared<Rtcm>();
    std::string bin_str("1101101011010110");
    std::string data_str = rtcm->bin_to_binary_data(bin_str);

    std::string test_binary = data_str.substr(0, 1);
    std::string test_bin = rtcm->binary_data_to_bin(test_binary);
    std::string test_hex = rtcm->bin_to_hex(test_bin);
    EXPECT_EQ(0, test_hex.compare("DA"));

    std::string recovered_str = rtcm->binary_data_to_bin(data_str);
    EXPECT_EQ(0, recovered_str.compare(bin_str));
}


TEST(RtcmTest, CheckCRC)
{
    auto rtcm = std::make_shared<Rtcm>();
    bool expected_true = true;
    bool expected_false = false;
    std::string good_crc = rtcm->bin_to_binary_data(rtcm->hex_to_bin("D300133ED7D30202980EDEEF34B4BD62AC0941986F33360B98"));
    std::string bad_crc = rtcm->bin_to_binary_data(rtcm->hex_to_bin("D300133ED7D30202980EDEEF34B4BD62AC0941986F33360B99"));
    EXPECT_EQ(expected_true, rtcm->check_CRC(good_crc));
    EXPECT_EQ(expected_false, rtcm->check_CRC(bad_crc));

    EXPECT_EQ(expected_true, rtcm->check_CRC(rtcm->print_MT1005_test()));
    EXPECT_EQ(expected_true, rtcm->check_CRC(rtcm->print_MT1005_test()));  // Run twice to check that CRC has no memory
}


TEST(RtcmTest, MT1001)
{
    auto rtcm = std::make_shared<Rtcm>();
    Gps_Ephemeris gps_eph = Gps_Ephemeris();
    Gnss_Synchro gnss_synchro;
    gnss_synchro.PRN = 2;
    std::string sys = "G";
    bool expected_true = true;
    unsigned short station_id = 1234;

    std::string sig = "1C";
    gnss_synchro.System = *sys.c_str();
    std::memcpy(static_cast<void*>(gnss_synchro.Signal), sig.c_str(), 3);
    gnss_synchro.Pseudorange_m = 20000000.0;
    double obs_time = 25.0;
    std::map<int, Gnss_Synchro> pseudoranges;
    pseudoranges.insert(std::pair<int, Gnss_Synchro>(1, gnss_synchro));

    std::string MT1001 = rtcm->print_MT1001(gps_eph, obs_time, pseudoranges, station_id);
    EXPECT_EQ(expected_true, rtcm->check_CRC(MT1001));
}


TEST(RtcmTest, MT1005)
{
    auto rtcm = std::make_shared<Rtcm>();
    std::string reference_msg = rtcm->print_MT1005_test();
    std::string reference_msg2 = rtcm->print_MT1005(2003, 1114104.5999, -4850729.7108, 3975521.4643, true, false, false, false, false, 0);
    EXPECT_EQ(0, reference_msg.compare(reference_msg2));

    unsigned int ref_id;
    double ecef_x;
    double ecef_y;
    double ecef_z;
    bool gps;
    bool glonass;
    bool galileo;
    bool expected_true = true;
    bool expected_false = false;

    rtcm->read_MT1005(reference_msg, ref_id, ecef_x, ecef_y, ecef_z, gps, glonass, galileo);

    EXPECT_EQ(expected_true, gps);
    EXPECT_EQ(expected_false, glonass);
    EXPECT_EQ(expected_false, galileo);

    EXPECT_EQ(static_cast<unsigned int>(2003), ref_id);
    EXPECT_DOUBLE_EQ(1114104.5999, ecef_x);
    EXPECT_DOUBLE_EQ(-4850729.7108, ecef_y);
    EXPECT_DOUBLE_EQ(3975521.4643, ecef_z);

    gps = false;
    ecef_x = 0.0;

    rtcm->read_MT1005(rtcm->bin_to_binary_data(rtcm->hex_to_bin("D300133ED7D30202980EDEEF34B4BD62AC0941986F33360B98")), ref_id, ecef_x, ecef_y, ecef_z, gps, glonass, galileo);

    EXPECT_EQ(expected_true, gps);
    EXPECT_EQ(expected_false, glonass);
    EXPECT_EQ(expected_false, galileo);

    EXPECT_EQ(static_cast<unsigned int>(2003), ref_id);
    EXPECT_DOUBLE_EQ(1114104.5999, ecef_x);
    EXPECT_DOUBLE_EQ(-4850729.7108, ecef_y);
    EXPECT_DOUBLE_EQ(3975521.4643, ecef_z);
}


TEST(RtcmTest, MT1019)
{
    auto rtcm = std::make_shared<Rtcm>();
    bool expected_true = true;

    Gps_Ephemeris gps_eph = Gps_Ephemeris();
    Gps_Ephemeris gps_eph_read = Gps_Ephemeris();

    gps_eph.i_satellite_PRN = 3;
    gps_eph.d_IODC = 4;
    gps_eph.d_e_eccentricity = 2.0 * E_LSB;
    gps_eph.b_fit_interval_flag = true;
    std::string tx_msg = rtcm->print_MT1019(gps_eph);

    EXPECT_EQ(0, rtcm->read_MT1019(tx_msg, gps_eph_read));
    EXPECT_EQ(static_cast<unsigned int>(3), gps_eph_read.i_satellite_PRN);
    EXPECT_DOUBLE_EQ(4, gps_eph_read.d_IODC);
    EXPECT_DOUBLE_EQ(2.0 * E_LSB, gps_eph_read.d_e_eccentricity);
    EXPECT_EQ(expected_true, gps_eph_read.b_fit_interval_flag);
    EXPECT_EQ(1, rtcm->read_MT1019(rtcm->bin_to_binary_data(rtcm->hex_to_bin("FFFFFFFFFFF")), gps_eph_read));
}


TEST(RtcmTest, MT1020)
{
    auto rtcm = std::make_shared<Rtcm>();

    // Objects to populate the ephemeris and utc fields
    Glonass_Gnav_Ephemeris gnav_ephemeris = Glonass_Gnav_Ephemeris();
    Glonass_Gnav_Utc_Model gnav_utc_model = Glonass_Gnav_Utc_Model();
    // Objects read, used for comparison
    Glonass_Gnav_Ephemeris gnav_ephemeris_read = Glonass_Gnav_Ephemeris();
    Glonass_Gnav_Utc_Model gnav_utc_model_read = Glonass_Gnav_Utc_Model();

    // Perform data read and print of special values types
    gnav_ephemeris.d_P_1 = 15;
    // Bit distribution per fields
    gnav_ephemeris.d_t_k = 7560;
    // Glonass signed values
    gnav_ephemeris.d_VXn = -0.490900039672852;
    // Bit distribution per fields dependent on other factors
    gnav_ephemeris.d_t_b = 8100;
    // Binary flag representation
    gnav_ephemeris.d_P_3 = 1;

    std::string tx_msg = rtcm->print_MT1020(gnav_ephemeris, gnav_utc_model);

    EXPECT_EQ(0, rtcm->read_MT1020(tx_msg, gnav_ephemeris_read, gnav_utc_model_read));
    EXPECT_EQ(gnav_ephemeris.d_P_1, gnav_ephemeris_read.d_P_1);
    EXPECT_TRUE(gnav_ephemeris.d_t_b - gnav_ephemeris_read.d_t_b < FLT_EPSILON);
    EXPECT_TRUE(gnav_ephemeris.d_VXn - gnav_ephemeris_read.d_VXn < FLT_EPSILON);
    EXPECT_TRUE(gnav_ephemeris.d_t_k - gnav_ephemeris.d_t_k < FLT_EPSILON);
    EXPECT_EQ(gnav_ephemeris.d_P_3, gnav_ephemeris_read.d_P_3);
    EXPECT_EQ(1, rtcm->read_MT1020(rtcm->bin_to_binary_data(rtcm->hex_to_bin("FFFFFFFFFFF")), gnav_ephemeris_read, gnav_utc_model_read));
}


TEST(RtcmTest, MT1029)
{
    auto rtcm = std::make_shared<Rtcm>();
    std::string s_test("UTF-8 проверка wörter");
    unsigned int ref_id = 23;
    double obs_time = 0;
    Gps_Ephemeris gps_eph = Gps_Ephemeris();
    std::string m1029 = rtcm->bin_to_hex(rtcm->binary_data_to_bin(rtcm->print_MT1029(ref_id, gps_eph, obs_time, s_test)));
    std::string encoded_text = m1029.substr(24, 60);
    std::string expected_encoded_text("5554462D3820D0BFD180D0BED0B2D0B5D180D0BAD0B02077C3B672746572");
    EXPECT_EQ(0, expected_encoded_text.compare(encoded_text));

    std::string characters_to_follow = m1029.substr(22, 2);
    std::string expected_characters_to_follow("1E");
    EXPECT_EQ(0, expected_characters_to_follow.compare(characters_to_follow));
}


TEST(RtcmTest, MT1045)
{
    auto rtcm = std::make_shared<Rtcm>();
    bool expected_true = true;

    Galileo_Ephemeris gal_eph = Galileo_Ephemeris();
    Galileo_Ephemeris gal_eph_read = Galileo_Ephemeris();

    gal_eph.i_satellite_PRN = 5;
    gal_eph.OMEGA_dot_3 = 53.0 * OMEGA_dot_3_LSB;
    gal_eph.E5a_DVS = true;

    std::string tx_msg = rtcm->print_MT1045(gal_eph);

    EXPECT_EQ(0, rtcm->read_MT1045(tx_msg, gal_eph_read));
    EXPECT_EQ(expected_true, gal_eph_read.E5a_DVS);
    EXPECT_DOUBLE_EQ(53.0 * OMEGA_dot_3_LSB, gal_eph_read.OMEGA_dot_3);
    EXPECT_EQ(static_cast<unsigned int>(5), gal_eph_read.i_satellite_PRN);
    EXPECT_EQ(1, rtcm->read_MT1045(rtcm->bin_to_binary_data(rtcm->hex_to_bin("FFFFFFFFFFF")), gal_eph_read));
}


TEST(RtcmTest, MSMCell)
{
    auto rtcm = std::make_shared<Rtcm>();
    Gps_Ephemeris gps_eph = Gps_Ephemeris();
    Galileo_Ephemeris gal_eph = Galileo_Ephemeris();
    //Glonass_Gnav_Ephemeris glo_gnav_eph = Glonass_Gnav_Ephemeris();
    std::map<int, Gnss_Synchro> pseudoranges;

    Gnss_Synchro gnss_synchro;
    Gnss_Synchro gnss_synchro2;
    Gnss_Synchro gnss_synchro3;
    Gnss_Synchro gnss_synchro4;
    Gnss_Synchro gnss_synchro5;
    Gnss_Synchro gnss_synchro6;

    gnss_synchro.PRN = 4;
    gnss_synchro2.PRN = 8;
    gnss_synchro3.PRN = 32;
    gnss_synchro4.PRN = 10;
    gnss_synchro5.PRN = 10;
    gnss_synchro6.PRN = 10;

    std::string gps = "G";
    std::string gal = "E";
    std::string glo = "R";

    std::string c1 = "1C";
    std::string s2 = "2S";
    std::string x5 = "5X";

    gnss_synchro.System = *gal.c_str();
    gnss_synchro2.System = *gps.c_str();
    gnss_synchro3.System = *gps.c_str();
    gnss_synchro4.System = *gal.c_str();
    gnss_synchro5.System = *gps.c_str();
    gnss_synchro6.System = *glo.c_str();

    std::memcpy((void*)gnss_synchro.Signal, x5.c_str(), 3);
    std::memcpy((void*)gnss_synchro2.Signal, s2.c_str(), 3);
    std::memcpy((void*)gnss_synchro3.Signal, c1.c_str(), 3);
    std::memcpy((void*)gnss_synchro4.Signal, x5.c_str(), 3);
    std::memcpy((void*)gnss_synchro5.Signal, c1.c_str(), 3);
    std::memcpy((void*)gnss_synchro6.Signal, c1.c_str(), 3);

    gnss_synchro.Pseudorange_m = 20000000.0;
    gnss_synchro2.Pseudorange_m = 20001010.0;
    gnss_synchro3.Pseudorange_m = 24002020.0;
    gnss_synchro4.Pseudorange_m = 20003010.1;
    gnss_synchro5.Pseudorange_m = 22003010.1;
    gnss_synchro6.Pseudorange_m = 22003010.1;

    pseudoranges.insert(std::pair<int, Gnss_Synchro>(1, gnss_synchro));
    pseudoranges.insert(std::pair<int, Gnss_Synchro>(2, gnss_synchro2));
    pseudoranges.insert(std::pair<int, Gnss_Synchro>(3, gnss_synchro3));
    pseudoranges.insert(std::pair<int, Gnss_Synchro>(4, gnss_synchro4));
    pseudoranges.insert(std::pair<int, Gnss_Synchro>(5, gnss_synchro5));
    pseudoranges.insert(std::pair<int, Gnss_Synchro>(6, gnss_synchro5));

    unsigned int ref_id = 1234;
    unsigned int clock_steering_indicator = 0;
    unsigned int external_clock_indicator = 0;
    int smooth_int = 0;
    bool divergence_free = false;
    bool more_messages = false;
    double obs_time = 25.0;

    gps_eph.i_satellite_PRN = gnss_synchro2.PRN;
    gal_eph.i_satellite_PRN = gnss_synchro.PRN;
    //glo_gnav_eph.i_satellite_PRN = gnss_synchro.PRN;

    std::string MSM1 = rtcm->print_MSM_1(gps_eph,
        {},
        gal_eph,
        {},
        obs_time,
        pseudoranges,
        ref_id,
        clock_steering_indicator,
        external_clock_indicator,
        smooth_int,
        divergence_free,
        more_messages);

    std::string MSM1_bin = rtcm->binary_data_to_bin(MSM1);
    unsigned int Nsat = 4;
    unsigned int Nsig = 3;
    unsigned int size_header = 14;
    unsigned int size_msg_length = 10;
    EXPECT_EQ(0, MSM1_bin.substr(size_header + size_msg_length + 169, Nsat * Nsig).compare("001010101100"));  // check cell mask

    std::map<int, Gnss_Synchro> pseudoranges2;
    pseudoranges2.insert(std::pair<int, Gnss_Synchro>(1, gnss_synchro6));
    pseudoranges2.insert(std::pair<int, Gnss_Synchro>(1, gnss_synchro5));
    pseudoranges2.insert(std::pair<int, Gnss_Synchro>(2, gnss_synchro4));
    pseudoranges2.insert(std::pair<int, Gnss_Synchro>(3, gnss_synchro3));
    pseudoranges2.insert(std::pair<int, Gnss_Synchro>(4, gnss_synchro2));
    pseudoranges2.insert(std::pair<int, Gnss_Synchro>(5, gnss_synchro));
    pseudoranges2.insert(std::pair<int, Gnss_Synchro>(6, gnss_synchro));
    std::string MSM1_2 = rtcm->print_MSM_1(gps_eph,
        {},
        gal_eph,
        {},
        obs_time,
        pseudoranges2,
        ref_id,
        clock_steering_indicator,
        external_clock_indicator,
        smooth_int,
        divergence_free,
        more_messages);
    std::string MSM1_bin_2 = rtcm->binary_data_to_bin(MSM1_2);
    EXPECT_EQ(0, MSM1_bin_2.substr(size_header + size_msg_length + 169, Nsat * Nsig).compare("001010001100"));  // check cell mask

    Gnss_Synchro gnss_synchro7;
    gnss_synchro7.PRN = 10;
    gnss_synchro7.System = *gps.c_str();
    std::memcpy((void*)gnss_synchro7.Signal, s2.c_str(), 3);
    gnss_synchro7.Pseudorange_m = 24000000.0;

    std::map<int, Gnss_Synchro> pseudoranges3;
    pseudoranges3.insert(std::pair<int, Gnss_Synchro>(1, gnss_synchro));
    pseudoranges3.insert(std::pair<int, Gnss_Synchro>(2, gnss_synchro2));
    pseudoranges3.insert(std::pair<int, Gnss_Synchro>(3, gnss_synchro7));
    pseudoranges3.insert(std::pair<int, Gnss_Synchro>(4, gnss_synchro4));
    pseudoranges3.insert(std::pair<int, Gnss_Synchro>(5, gnss_synchro5));

    std::string MSM1_3 = rtcm->print_MSM_1(gps_eph,
        {},
        gal_eph,
        {},
        obs_time,
        pseudoranges3,
        ref_id,
        clock_steering_indicator,
        external_clock_indicator,
        smooth_int,
        divergence_free,
        more_messages);
    std::string MSM1_bin_3 = rtcm->binary_data_to_bin(MSM1_3);
    EXPECT_EQ(0, MSM1_bin_3.substr(size_header + size_msg_length + 169, (Nsat - 1) * Nsig).compare("001010111"));  // check cell mask
}


TEST(RtcmTest, MSM1)
{
    auto rtcm = std::make_shared<Rtcm>();
    bool expected_true = true;
    Gps_Ephemeris gps_eph = Gps_Ephemeris();
    std::map<int, Gnss_Synchro> pseudoranges;

    Gnss_Synchro gnss_synchro;
    Gnss_Synchro gnss_synchro2;
    Gnss_Synchro gnss_synchro3;
    Gnss_Synchro gnss_synchro4;

    gnss_synchro.PRN = 2;
    gnss_synchro2.PRN = 4;
    gnss_synchro3.PRN = 32;
    gnss_synchro4.PRN = 4;

    std::string sys = "G";

    std::string sig = "1C";
    std::string sig2 = "2S";

    gnss_synchro.System = *sys.c_str();
    gnss_synchro2.System = *sys.c_str();
    gnss_synchro3.System = *sys.c_str();
    gnss_synchro4.System = *sys.c_str();

    std::memcpy(static_cast<void*>(gnss_synchro.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gnss_synchro2.Signal), sig.c_str(), 3);
    std::memcpy(static_cast<void*>(gnss_synchro3.Signal), sig2.c_str(), 3);
    std::memcpy(static_cast<void*>(gnss_synchro4.Signal), sig2.c_str(), 3);

    gnss_synchro.Pseudorange_m = 20000000.0;
    gnss_synchro2.Pseudorange_m = 20001010.0;
    gnss_synchro3.Pseudorange_m = 24002020.0;
    gnss_synchro4.Pseudorange_m = 20003010.1;

    pseudoranges.insert(std::pair<int, Gnss_Synchro>(1, gnss_synchro));
    pseudoranges.insert(std::pair<int, Gnss_Synchro>(2, gnss_synchro2));
    pseudoranges.insert(std::pair<int, Gnss_Synchro>(3, gnss_synchro3));
    pseudoranges.insert(std::pair<int, Gnss_Synchro>(4, gnss_synchro4));

    unsigned short ref_id = 1234;
    unsigned int clock_steering_indicator = 0;
    unsigned int external_clock_indicator = 0;
    int smooth_int = 0;
    bool divergence_free = false;
    bool more_messages = false;
    double obs_time = 25.0;

    gps_eph.i_satellite_PRN = gnss_synchro.PRN;

    std::string MSM1 = rtcm->print_MSM_1(gps_eph,
        {}, {}, {},
        obs_time,
        pseudoranges,
        ref_id,
        clock_steering_indicator,
        external_clock_indicator,
        smooth_int,
        divergence_free,
        more_messages);

    EXPECT_EQ(expected_true, rtcm->check_CRC(MSM1));

    std::string MSM1_bin = rtcm->binary_data_to_bin(MSM1);
    unsigned int Nsat = 3;
    unsigned int Nsig = 2;
    unsigned int size_header = 14;
    unsigned int size_crc = 24;
    unsigned int size_msg_length = 10;
    unsigned int upper_bound = 169 + Nsat * 10 + 43 * Nsig;
    unsigned int data_size = MSM1_bin.length() - size_header - size_msg_length - size_crc;
    EXPECT_EQ(expected_true, upper_bound >= data_size);
    EXPECT_EQ(0, MSM1_bin.substr(0, size_header).compare("11010011000000"));
    EXPECT_EQ(ref_id, rtcm->bin_to_uint(MSM1_bin.substr(size_header + size_msg_length + 12, 12)));
    EXPECT_EQ(0, MSM1_bin.substr(size_header + size_msg_length + 169, Nsat * Nsig).compare("101101"));  // check cell mask

    double meters_to_miliseconds = GPS_C_m_s * 0.001;
    unsigned int rough_range_1 = static_cast<unsigned int>(std::floor(std::round(gnss_synchro.Pseudorange_m / meters_to_miliseconds / TWO_N10)) + 0.5) & 0x3FFu;
    unsigned int rough_range_2 = static_cast<unsigned int>(std::floor(std::round(gnss_synchro2.Pseudorange_m / meters_to_miliseconds / TWO_N10)) + 0.5) & 0x3FFu;
    unsigned int rough_range_4 = static_cast<unsigned int>(std::floor(std::round(gnss_synchro3.Pseudorange_m / meters_to_miliseconds / TWO_N10)) + 0.5) & 0x3FFu;
    unsigned int read_pseudorange_1 = rtcm->bin_to_uint(MSM1_bin.substr(size_header + size_msg_length + 169 + Nsat * Nsig, 10));
    unsigned int read_pseudorange_2 = rtcm->bin_to_uint(MSM1_bin.substr(size_header + size_msg_length + 169 + Nsat * Nsig + 10, 10));
    unsigned int read_pseudorange_4 = rtcm->bin_to_uint(MSM1_bin.substr(size_header + size_msg_length + 169 + Nsat * Nsig + 20, 10));

    EXPECT_EQ(rough_range_1, read_pseudorange_1);
    EXPECT_EQ(rough_range_2, read_pseudorange_2);
    EXPECT_EQ(rough_range_4, read_pseudorange_4);

    int psrng4_s = static_cast<int>(std::round((gnss_synchro3.Pseudorange_m - std::round(gnss_synchro3.Pseudorange_m / meters_to_miliseconds / TWO_N10) * meters_to_miliseconds * TWO_N10) / meters_to_miliseconds / TWO_N24));
    int read_psrng4_s = rtcm->bin_to_int(MSM1_bin.substr(size_header + size_msg_length + 169 + (Nsat * Nsig) + 30 + 15 * 3, 15));
    EXPECT_EQ(psrng4_s, read_psrng4_s);

    std::map<int, Gnss_Synchro> pseudoranges2;
    pseudoranges2.insert(std::pair<int, Gnss_Synchro>(1, gnss_synchro4));
    pseudoranges2.insert(std::pair<int, Gnss_Synchro>(2, gnss_synchro3));
    pseudoranges2.insert(std::pair<int, Gnss_Synchro>(3, gnss_synchro2));
    pseudoranges2.insert(std::pair<int, Gnss_Synchro>(4, gnss_synchro));
    std::string MSM1_2 = rtcm->print_MSM_1(gps_eph,
        {}, {}, {},
        obs_time,
        pseudoranges2,
        ref_id,
        clock_steering_indicator,
        external_clock_indicator,
        smooth_int,
        divergence_free,
        more_messages);
    std::string MSM1_bin2 = rtcm->binary_data_to_bin(MSM1_2);
    int read_psrng4_s_2 = rtcm->bin_to_int(MSM1_bin2.substr(size_header + size_msg_length + 169 + (Nsat * Nsig) + 30 + 15 * 3, 15));
    EXPECT_EQ(psrng4_s, read_psrng4_s_2);
}


TEST(RtcmTest, InstantiateServer)
{
    auto rtcm = std::make_shared<Rtcm>();
    rtcm->run_server();
    std::string msg("Hello");
    rtcm->send_message(msg);
    std::string test3 = "ff";
    std::string test3_bin = rtcm->hex_to_bin(test3);
    EXPECT_EQ(0, test3_bin.compare("11111111"));
    rtcm->stop_server();
    std::string test6 = "0011";
    std::string test6_hex = rtcm->bin_to_hex(test6);
    EXPECT_EQ(0, test6_hex.compare("3"));
    long unsigned int expected1 = 42;
    EXPECT_EQ(expected1, rtcm->bin_to_uint("00101010"));
    rtcm->run_server();
    std::string test4_bin = rtcm->hex_to_bin(test3);
    std::string s = rtcm->bin_to_binary_data(test4_bin);
    rtcm->send_message(s);
    rtcm->stop_server();
    EXPECT_EQ(0, test4_bin.compare("11111111"));
}


TEST(RtcmTest, InstantiateServerWithoutClosing)
{
    auto rtcm = std::make_shared<Rtcm>();
    rtcm->run_server();
    std::string msg("Hello");
    rtcm->send_message(msg);
    std::string test3 = "ff";
    std::string test3_bin = rtcm->hex_to_bin(test3);
    EXPECT_EQ(0, test3_bin.compare("11111111"));
}
