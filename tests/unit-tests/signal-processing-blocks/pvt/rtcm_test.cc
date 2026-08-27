/*!
 * \file rtcm_test.cc
 * \brief  This file implements unit tests for the Rtcm class.
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "Galileo_FNAV.h"
#include "Galileo_INAV.h"
#include "rtcm.h"
#include "rtklib_rtcm.h"
#include "rtklib_rtkcmn.h"
#include <cmath>
#include <cstring>
#include <memory>
#include <thread>

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
    int64_t test1_int = rtcm->hex_to_int(test1);
    int64_t expected1 = 42;
    EXPECT_EQ(expected1, test1_int);
}


TEST(RtcmTest, HexToUint)
{
    auto rtcm = std::make_shared<Rtcm>();
    uint64_t expected1 = 42;
    EXPECT_EQ(expected1, rtcm->hex_to_uint(rtcm->bin_to_hex("00101010")));
}


TEST(RtcmTest, BinToDouble)
{
    auto rtcm = std::make_shared<Rtcm>();

    std::bitset<4> test1(5);
    int64_t test1_int = static_cast<int64_t>(rtcm->bin_to_double(test1.to_string()));
    int64_t expected1 = 5;
    EXPECT_EQ(expected1, test1_int);

    std::bitset<4> test2(-5);
    EXPECT_DOUBLE_EQ(-5, rtcm->bin_to_double(test2.to_string()));

    std::bitset<65> test3(-5);
    EXPECT_DOUBLE_EQ(0, rtcm->bin_to_double(test3.to_string()));
}


TEST(RtcmTest, BinToUint)
{
    auto rtcm = std::make_shared<Rtcm>();
    uint32_t expected1 = 42;
    EXPECT_EQ(expected1, rtcm->bin_to_uint("00101010"));
    uint32_t expected2 = 214;
    EXPECT_EQ(expected2, rtcm->bin_to_uint("11010110"));
}


TEST(RtcmTest, BinToInt)
{
    auto rtcm = std::make_shared<Rtcm>();
    int32_t expected1 = 42;
    EXPECT_EQ(expected1, rtcm->bin_to_int("00101010"));
    int32_t expected2 = -42;
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

    gps_eph.PRN = 3;
    gps_eph.IODC = 4;
    gps_eph.ecc = 2.0 * ECCENTRICITY_LSB;
    gps_eph.fit_interval_flag = true;
    std::string tx_msg = rtcm->print_MT1019(gps_eph);

    EXPECT_EQ(0, rtcm->read_MT1019(tx_msg, gps_eph_read));
    EXPECT_EQ(static_cast<unsigned int>(3), gps_eph_read.PRN);
    EXPECT_DOUBLE_EQ(4, gps_eph_read.IODC);
    EXPECT_DOUBLE_EQ(2.0 * ECCENTRICITY_LSB, gps_eph_read.ecc);
    EXPECT_EQ(expected_true, gps_eph_read.fit_interval_flag);
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
    gnav_ephemeris.d_P_1 = 0.;
    // Bit distribution per fields
    gnav_ephemeris.d_t_k = 7560.;
    // Glonass signed values
    gnav_ephemeris.d_VXn = -0.490900039672852;
    // Bit distribution per fields dependent on other factors
    gnav_ephemeris.d_t_b = 8100.;
    // Binary flag representation
    gnav_ephemeris.d_P_3 = true;

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

    std::string oversized_text(1100, 'A');
    EXPECT_TRUE(rtcm->print_MT1029(ref_id, gps_eph, obs_time, oversized_text).empty());
}


TEST(RtcmTest, MT1045)
{
    auto rtcm = std::make_shared<Rtcm>();

    const int32_t idot_raw = -1234;
    const int32_t af2_raw = -17;
    const int32_t af1_raw = 123456;
    const int32_t af0_raw = -123456789;
    const int32_t crs_raw = -1234;
    const int32_t delta_n_raw = 2345;
    const int32_t m0_raw = -123456789;
    const int32_t cuc_raw = -2345;
    const uint32_t ecc_raw = 123456789U;
    const int32_t cus_raw = 3456;
    const uint32_t sqrt_a_raw = 2850000000U;
    const int32_t cic_raw = -4567;
    const int32_t omega0_raw = 987654321;
    const int32_t cis_raw = 5678;
    const int32_t i0_raw = 678901234;
    const int32_t crc_raw = -6789;
    const int32_t omega_raw = -789012345;
    const int32_t omega_dot_raw = -123456;
    const int32_t bgd_raw = -123;

    Galileo_Ephemeris gal_eph;
    gal_eph.nav_message_type = Galileo_Nav_Message_Type::FNAV;
    gal_eph.PRN = 5U;
    gal_eph.WN = 2345U;
    gal_eph.IOD_ephemeris = 511;
    gal_eph.IOD_nav = 17;  // MT1045 must use the F/NAV ephemeris IOD.
    gal_eph.SISA = 42;
    gal_eph.idot = static_cast<double>(idot_raw) * FNAV_IDOT_2_LSB;
    gal_eph.toc = 345600.0;
    gal_eph.af2 = static_cast<double>(af2_raw) * FNAV_AF2_1_LSB;
    gal_eph.af1 = static_cast<double>(af1_raw) * FNAV_AF1_1_LSB;
    gal_eph.af0 = static_cast<double>(af0_raw) * FNAV_AF0_1_LSB;
    gal_eph.Crs = static_cast<double>(crs_raw) * FNAV_CRS_3_LSB;
    gal_eph.delta_n = static_cast<double>(delta_n_raw) * FNAV_DELTAN_3_LSB;
    gal_eph.M_0 = static_cast<double>(m0_raw) * FNAV_M0_2_LSB;
    gal_eph.Cuc = static_cast<double>(cuc_raw) * FNAV_CUC_3_LSB;
    gal_eph.ecc = static_cast<double>(ecc_raw) * FNAV_E_2_LSB;
    gal_eph.Cus = static_cast<double>(cus_raw) * FNAV_CUS_3_LSB;
    gal_eph.sqrtA = static_cast<double>(sqrt_a_raw) * FNAV_A12_2_LSB;
    gal_eph.toe = 432000.0;
    gal_eph.Cic = static_cast<double>(cic_raw) * FNAV_CIC_4_LSB;
    gal_eph.OMEGA_0 = static_cast<double>(omega0_raw) * FNAV_OMEGA0_2_LSB;
    gal_eph.Cis = static_cast<double>(cis_raw) * FNAV_CIS_4_LSB;
    gal_eph.i_0 = static_cast<double>(i0_raw) * FNAV_I0_3_LSB;
    gal_eph.Crc = static_cast<double>(crc_raw) * FNAV_CRC_3_LSB;
    gal_eph.omega = static_cast<double>(omega_raw) * FNAV_W_3_LSB;
    gal_eph.OMEGAdot = static_cast<double>(omega_dot_raw) * FNAV_OMEGADOT_2_LSB;
    gal_eph.BGD_E1E5a = static_cast<double>(bgd_raw) * FNAV_BGD_1_LSB;
    gal_eph.E5a_HS = 2;
    gal_eph.E5a_DVS = true;

    const std::string tx_msg = rtcm->print_MT1045(gal_eph);
    ASSERT_EQ(68U, tx_msg.size());  // 3-byte header, 62-byte payload, 3-byte CRC.
    ASSERT_TRUE(rtcm->check_CRC(tx_msg));

    const std::string message_bits = rtcm->binary_data_to_bin(tx_msg);
    ASSERT_EQ(544U, message_bits.size());
    EXPECT_EQ("11010011", message_bits.substr(0, 8));
    EXPECT_EQ(0U, rtcm->bin_to_uint(message_bits.substr(8, 6)));
    EXPECT_EQ(62U, rtcm->bin_to_uint(message_bits.substr(14, 10)));

    std::size_t field_index = 24U;
    const auto read_unsigned = [&message_bits, &field_index, &rtcm](std::size_t width) {
        const auto value = rtcm->bin_to_uint(message_bits.substr(field_index, width));
        field_index += width;
        return value;
    };
    const auto read_signed = [&message_bits, &field_index, &rtcm](std::size_t width) {
        const auto value = rtcm->bin_to_int(message_bits.substr(field_index, width));
        field_index += width;
        return value;
    };

    EXPECT_EQ(1045U, read_unsigned(12));
    EXPECT_EQ(gal_eph.PRN, read_unsigned(6));
    EXPECT_EQ(gal_eph.WN, read_unsigned(12));
    EXPECT_EQ(static_cast<uint32_t>(gal_eph.IOD_ephemeris), read_unsigned(10));
    EXPECT_EQ(static_cast<uint32_t>(gal_eph.SISA), read_unsigned(8));
    EXPECT_EQ(idot_raw, read_signed(14));
    EXPECT_EQ(5760U, read_unsigned(14));
    EXPECT_EQ(af2_raw, read_signed(6));
    EXPECT_EQ(af1_raw, read_signed(21));
    EXPECT_EQ(af0_raw, read_signed(31));
    EXPECT_EQ(crs_raw, read_signed(16));
    EXPECT_EQ(delta_n_raw, read_signed(16));
    EXPECT_EQ(m0_raw, read_signed(32));
    EXPECT_EQ(cuc_raw, read_signed(16));
    EXPECT_EQ(ecc_raw, read_unsigned(32));
    EXPECT_EQ(cus_raw, read_signed(16));
    EXPECT_EQ(sqrt_a_raw, read_unsigned(32));
    EXPECT_EQ(7200U, read_unsigned(14));
    EXPECT_EQ(cic_raw, read_signed(16));
    EXPECT_EQ(omega0_raw, read_signed(32));
    EXPECT_EQ(cis_raw, read_signed(16));
    EXPECT_EQ(i0_raw, read_signed(32));
    EXPECT_EQ(crc_raw, read_signed(16));
    EXPECT_EQ(omega_raw, read_signed(32));
    EXPECT_EQ(omega_dot_raw, read_signed(24));
    EXPECT_EQ(bgd_raw, read_signed(10));
    EXPECT_EQ(2U, read_unsigned(2));
    EXPECT_EQ(1U, read_unsigned(1));
    EXPECT_EQ(0U, read_unsigned(7));
    EXPECT_EQ(520U, field_index);

    Galileo_Ephemeris gal_eph_read;
    EXPECT_EQ(0, rtcm->read_MT1045(tx_msg, gal_eph_read));
    EXPECT_EQ(gal_eph.PRN, gal_eph_read.PRN);
    EXPECT_EQ(gal_eph.WN, gal_eph_read.WN);
    EXPECT_EQ(gal_eph.IOD_ephemeris, gal_eph_read.IOD_ephemeris);
    EXPECT_EQ(gal_eph.IOD_ephemeris, gal_eph_read.IOD_nav);
    EXPECT_EQ(gal_eph.SISA, gal_eph_read.SISA);
    EXPECT_DOUBLE_EQ(gal_eph.idot, gal_eph_read.idot);
    EXPECT_DOUBLE_EQ(gal_eph.toc, gal_eph_read.toc);
    EXPECT_DOUBLE_EQ(gal_eph.af2, gal_eph_read.af2);
    EXPECT_DOUBLE_EQ(gal_eph.af1, gal_eph_read.af1);
    EXPECT_DOUBLE_EQ(gal_eph.af0, gal_eph_read.af0);
    EXPECT_DOUBLE_EQ(gal_eph.Crs, gal_eph_read.Crs);
    EXPECT_DOUBLE_EQ(gal_eph.delta_n, gal_eph_read.delta_n);
    EXPECT_DOUBLE_EQ(gal_eph.M_0, gal_eph_read.M_0);
    EXPECT_DOUBLE_EQ(gal_eph.Cuc, gal_eph_read.Cuc);
    EXPECT_DOUBLE_EQ(gal_eph.ecc, gal_eph_read.ecc);
    EXPECT_DOUBLE_EQ(gal_eph.Cus, gal_eph_read.Cus);
    EXPECT_DOUBLE_EQ(gal_eph.sqrtA, gal_eph_read.sqrtA);
    EXPECT_DOUBLE_EQ(gal_eph.toe, gal_eph_read.toe);
    EXPECT_DOUBLE_EQ(gal_eph.Cic, gal_eph_read.Cic);
    EXPECT_DOUBLE_EQ(gal_eph.OMEGA_0, gal_eph_read.OMEGA_0);
    EXPECT_DOUBLE_EQ(gal_eph.Cis, gal_eph_read.Cis);
    EXPECT_DOUBLE_EQ(gal_eph.i_0, gal_eph_read.i_0);
    EXPECT_DOUBLE_EQ(gal_eph.Crc, gal_eph_read.Crc);
    EXPECT_DOUBLE_EQ(gal_eph.omega, gal_eph_read.omega);
    EXPECT_DOUBLE_EQ(gal_eph.OMEGAdot, gal_eph_read.OMEGAdot);
    EXPECT_DOUBLE_EQ(gal_eph.BGD_E1E5a, gal_eph_read.BGD_E1E5a);
    EXPECT_EQ(gal_eph.E5a_HS, gal_eph_read.E5a_HS);
    EXPECT_EQ(gal_eph.E5a_DVS, gal_eph_read.E5a_DVS);
    EXPECT_EQ(Galileo_Nav_Message_Type::FNAV, gal_eph_read.nav_message_type);

    // Verify interoperability with the independent RTKLIB RTCM3 decoder.
    rtcm_t rtklib_decoder{};
    ASSERT_EQ(1, init_rtcm(&rtklib_decoder));
    int decoder_status = 0;
    for (const char byte : tx_msg)
        {
            decoder_status = input_rtcm3(&rtklib_decoder, static_cast<unsigned char>(byte));
        }
    EXPECT_EQ(2, decoder_status);
    ASSERT_GT(rtklib_decoder.ephsat, 0);
    const eph_t& rtklib_ephemeris = rtklib_decoder.nav.eph[rtklib_decoder.ephsat - 1];
    EXPECT_EQ(gal_eph.IOD_ephemeris, rtklib_ephemeris.iode);
    EXPECT_EQ(static_cast<int>(gal_eph.WN + 1024U), rtklib_ephemeris.week);
    EXPECT_EQ(gal_eph.SISA, rtklib_ephemeris.sva);
    const auto expect_rtklib_near = [](double expected, double actual) {
        EXPECT_NEAR(expected, actual, std::fabs(expected) * 1e-14 + 1e-30);
    };
    expect_rtklib_near(gal_eph.idot, rtklib_ephemeris.idot);
    int decoded_week = 0;
    EXPECT_DOUBLE_EQ(gal_eph.toc, time2gpst(rtklib_ephemeris.toc, &decoded_week));
    EXPECT_EQ(static_cast<int>(gal_eph.WN + 1024U), decoded_week);
    EXPECT_DOUBLE_EQ(gal_eph.af2, rtklib_ephemeris.f2);
    EXPECT_DOUBLE_EQ(gal_eph.af1, rtklib_ephemeris.f1);
    EXPECT_DOUBLE_EQ(gal_eph.af0, rtklib_ephemeris.f0);
    EXPECT_DOUBLE_EQ(gal_eph.Crs, rtklib_ephemeris.crs);
    expect_rtklib_near(gal_eph.delta_n, rtklib_ephemeris.deln);
    expect_rtklib_near(gal_eph.M_0, rtklib_ephemeris.M0);
    EXPECT_DOUBLE_EQ(gal_eph.Cuc, rtklib_ephemeris.cuc);
    EXPECT_DOUBLE_EQ(gal_eph.ecc, rtklib_ephemeris.e);
    EXPECT_DOUBLE_EQ(gal_eph.Cus, rtklib_ephemeris.cus);
    EXPECT_DOUBLE_EQ(gal_eph.sqrtA * gal_eph.sqrtA, rtklib_ephemeris.A);
    EXPECT_DOUBLE_EQ(gal_eph.toe, rtklib_ephemeris.toes);
    EXPECT_DOUBLE_EQ(gal_eph.Cic, rtklib_ephemeris.cic);
    expect_rtklib_near(gal_eph.OMEGA_0, rtklib_ephemeris.OMG0);
    EXPECT_DOUBLE_EQ(gal_eph.Cis, rtklib_ephemeris.cis);
    expect_rtklib_near(gal_eph.i_0, rtklib_ephemeris.i0);
    EXPECT_DOUBLE_EQ(gal_eph.Crc, rtklib_ephemeris.crc);
    expect_rtklib_near(gal_eph.omega, rtklib_ephemeris.omg);
    expect_rtklib_near(gal_eph.OMEGAdot, rtklib_ephemeris.OMGd);
    EXPECT_DOUBLE_EQ(gal_eph.BGD_E1E5a, rtklib_ephemeris.tgd[0]);
    EXPECT_EQ((gal_eph.E5a_HS << 4) + (static_cast<int>(gal_eph.E5a_DVS) << 3), rtklib_ephemeris.svh);
    EXPECT_EQ(2, rtklib_ephemeris.code);
    free_rtcm(&rtklib_decoder);

    EXPECT_EQ(1, rtcm->read_MT1045(rtcm->bin_to_binary_data(rtcm->hex_to_bin("FFFFFFFFFFF")), gal_eph_read));

    gal_eph.nav_message_type = Galileo_Nav_Message_Type::INAV;
    EXPECT_TRUE(rtcm->print_MT1045(gal_eph).empty());
}


TEST(RtcmTest, MSMCell)
{
    auto rtcm = std::make_shared<Rtcm>();
    Gps_Ephemeris gps_eph = Gps_Ephemeris();
    Galileo_Ephemeris gal_eph = Galileo_Ephemeris();
    // Glonass_Gnav_Ephemeris glo_gnav_eph = Glonass_Gnav_Ephemeris();
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

    std::string gal = "E";

    std::string e1 = "1B";
    std::string e5b = "7X";
    std::string e5a = "5X";

    gnss_synchro.System = *gal.c_str();
    gnss_synchro2.System = *gal.c_str();
    gnss_synchro3.System = *gal.c_str();
    gnss_synchro4.System = *gal.c_str();
    gnss_synchro5.System = *gal.c_str();
    gnss_synchro6.System = *gal.c_str();

    std::memcpy(reinterpret_cast<void*>(gnss_synchro.Signal), e5a.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gnss_synchro2.Signal), e5b.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gnss_synchro3.Signal), e1.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gnss_synchro4.Signal), e5a.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gnss_synchro5.Signal), e1.c_str(), 3);
    std::memcpy(reinterpret_cast<void*>(gnss_synchro6.Signal), e1.c_str(), 3);

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

    gal_eph.PRN = gnss_synchro.PRN;
    // glo_gnav_eph.PRN = gnss_synchro.PRN;

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
    EXPECT_EQ(0, MSM1_bin_2.substr(size_header + size_msg_length + 169, Nsat * Nsig).compare("001010101100"));  // check cell mask

    Gnss_Synchro gnss_synchro7;
    gnss_synchro7.PRN = 10;
    gnss_synchro7.System = *gal.c_str();
    std::memcpy(reinterpret_cast<void*>(gnss_synchro7.Signal), e5b.c_str(), 3);
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
    unsigned int clock_steering_indicator = 2;
    unsigned int external_clock_indicator = 1;
    int smooth_int = 0;
    bool divergence_free = true;
    bool more_messages = false;
    double obs_time = 25.0;

    gps_eph.PRN = gnss_synchro.PRN;

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
    EXPECT_EQ(std::string("10011000"), MSM1_bin.substr(size_header + size_msg_length + 65, 8));         // check MSM common header DF411, DF412, DF417, DF418 order
    EXPECT_EQ(0, MSM1_bin.substr(size_header + size_msg_length + 169, Nsat * Nsig).compare("101101"));  // check cell mask

    double meters_to_miliseconds = SPEED_OF_LIGHT_M_S * 0.001;
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


TEST(RtcmTest, MSMSizeLimits)
{
    auto rtcm = std::make_shared<Rtcm>();
    Galileo_Ephemeris gal_eph = Galileo_Ephemeris();
    gal_eph.PRN = 1;

    const auto make_galileo_observables = [](uint32_t nsats) -> std::map<int, Gnss_Synchro> {
        std::map<int, Gnss_Synchro> observables;
        const std::string signals[3] = {"1B", "5X", "7X"};
        int key = 1;
        for (uint32_t prn = 1; prn <= nsats; prn++)
            {
                for (uint32_t signal_index = 0; signal_index < 3; signal_index++)
                    {
                        Gnss_Synchro gnss_synchro;
                        gnss_synchro.PRN = prn;
                        gnss_synchro.System = 'E';
                        std::memcpy(static_cast<void*>(gnss_synchro.Signal), signals[signal_index].c_str(), 3);
                        gnss_synchro.Pseudorange_m = 20000000.0 + 1000.0 * static_cast<double>(prn) + 10.0 * signal_index;
                        gnss_synchro.Carrier_phase_rads = 1000.0 + static_cast<double>(prn);
                        gnss_synchro.Carrier_Doppler_hz = -500.0;
                        gnss_synchro.CN0_dB_hz = 45.0;
                        gnss_synchro.Tracking_sample_counter = 100000;
                        observables.insert(std::pair<int, Gnss_Synchro>(key, gnss_synchro));
                        key++;
                    }
            }
        return observables;
    };

    const std::map<int, Gnss_Synchro> within_limit = make_galileo_observables(21);  // 21 satellites * 3 signals = 63 cell-mask bits
    const std::string within_limit_msg = rtcm->print_MSM_7({}, {}, gal_eph, {}, 25.0, within_limit, 1234, 0, 0, 0, false, false);
    EXPECT_FALSE(within_limit_msg.empty());
    EXPECT_TRUE(rtcm->check_CRC(within_limit_msg));

    const std::map<int, Gnss_Synchro> over_limit = make_galileo_observables(22);  // 22 satellites * 3 signals = 66 cell-mask bits
    const std::string over_limit_msg = rtcm->print_MSM_7({}, {}, gal_eph, {}, 25.0, over_limit, 1234, 0, 0, 0, false, false);
    EXPECT_TRUE(over_limit_msg.empty());
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
    uint64_t expected1 = 42;
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
