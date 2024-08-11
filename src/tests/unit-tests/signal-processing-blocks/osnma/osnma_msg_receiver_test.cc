/*!
 * \file osmna_msg_receiver_test.cc
 * \brief Tests for the osnma_msg_receiver class.
 * \author Carles Fernandez, 2023-2024. cfernandez(at)cttc.es
 *   Cesare Ghionoiu Martinez, 2023-2024. c.ghionoiu-martinez@tu-braunschweig.de
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "Galileo_OSNMA.h"
#include "gnss_crypto.h"
#include "osnma_helper.h"
#include "osnma_msg_receiver.h"
#include <gtest/gtest.h>
#include <bitset>
#include <chrono>
#include <fstream>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>  // for LOG
#else
#include <absl/log/log.h>
#endif

class OsnmaMsgReceiverTest : public ::testing::Test
{
protected:
    Osnma_Helper helper;
    osnma_msg_receiver_sptr osnma;
    OSNMA_msg osnma_msg{};
    std::array<int8_t, 15> nma_position_filled;
    uint32_t d_GST_SIS{};
    uint32_t TOW{};
    uint32_t WN{};
    std::tm GST_START_EPOCH = {0, 0, 0, 22, 8 - 1, 1999 - 1900, 0, 0, 0, 0, 0};  // months start with 0 and years since 1900 in std::tm
    const uint32_t LEAP_SECONDS = 0;                                             // tried with 13 + 5, which is the official count, but won't parse correctly
    void set_time(std::tm& input);

    void SetUp() override
    {
        // std::tm input_time = {0, 0, 5, 16, 8 - 1, 2023 - 1900, 0, 0, 0, 0, 0}; // conf. 1
        std::tm input_time = {0, 0, 0, 27, 7 - 1, 2023 - 1900, 0, 0, 0, 0, 0};  // conf. 2
        set_time(input_time);
        osnma = osnma_msg_receiver_make(CRTFILE_DEFAULT, MERKLEFILE_DEFAULT);
    }
};


TEST_F(OsnmaMsgReceiverTest, ComputeMerkleRoot)
{
    // input data taken from Receiver Guidelines v1.3,  A.7
    // Arrange
    std::vector<uint8_t> computed_merkle_root;
    std::vector<uint8_t> expected_merkle_root = helper.convert_from_hex_string("A10C440F3AA62453526DB4AF76DF8D9410D35D8277397D7053C700D192702B0D");
    DSM_PKR_message dsm_pkr_message;
    dsm_pkr_message.npkt = 0x01;
    dsm_pkr_message.npktid = 0x2;
    dsm_pkr_message.mid = 0x01;
    std::vector<uint8_t> base_leaf = helper.convert_from_hex_string("120303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");

    // ITN
    std::vector<uint8_t> vec = helper.convert_from_hex_string(
        "7CBE05D9970CFC9E22D0A43A340EF557624453A2E821AADEAC989C405D78BA06"
        "956380BAB0D2C939EC6208151040CCFFCF1FB7156178FD1255BA0AECAAA253F7"
        "407B6C5DD4DF059FF8789474061301E1C34881DB7A367A913A3674300E21EAB1"
        "24EF508389B7D446C3E2ECE8D459FBBD3239A794906F5B1F92469C640164FD87");
    std::copy(vec.begin(), vec.end(), dsm_pkr_message.itn.begin());
    dsm_pkr_message.npk = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");

    // Act
    computed_merkle_root = osnma->compute_merkle_root(dsm_pkr_message, base_leaf);

    // Assert
    ASSERT_EQ(computed_merkle_root, expected_merkle_root);
}


TEST_F(OsnmaMsgReceiverTest, ComputeBaseLeaf)
{
    // input data taken from Receiver Guidelines v1.3,  A.7
    // Arrange
    std::vector<uint8_t> expected_base_leaf = helper.convert_from_hex_string("120303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");
    DSM_PKR_message dsm_pkr_message;
    dsm_pkr_message.npkt = 0x01;
    dsm_pkr_message.npktid = 0x2;
    dsm_pkr_message.npk = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");

    // Act
    std::vector<uint8_t> computed_base_leaf = osnma->get_merkle_tree_leaves(dsm_pkr_message);

    // Assert
    ASSERT_EQ(computed_base_leaf, expected_base_leaf);
}


TEST_F(OsnmaMsgReceiverTest, VerifyPublicKey)
{
    // Input data taken from Receiver Guidelines v1.3,  A.7
    // Arrange
    osnma->d_crypto->set_merkle_root(helper.convert_from_hex_string("A10C440F3AA62453526DB4AF76DF8D9410D35D8277397D7053C700D192702B0D"));
    DSM_PKR_message dsm_pkr_message;
    dsm_pkr_message.npkt = 0x01;
    dsm_pkr_message.npktid = 0x2;
    dsm_pkr_message.mid = 0x01;
    std::vector<uint8_t> vec = helper.convert_from_hex_string(
        "7CBE05D9970CFC9E22D0A43A340EF557624453A2E821AADEAC989C405D78BA06"
        "956380BAB0D2C939EC6208151040CCFFCF1FB7156178FD1255BA0AECAAA253F7"
        "407B6C5DD4DF059FF8789474061301E1C34881DB7A367A913A3674300E21EAB1"
        "24EF508389B7D446C3E2ECE8D459FBBD3239A794906F5B1F92469C640164FD87");
    std::copy(vec.begin(), vec.end(), dsm_pkr_message.itn.begin());
    dsm_pkr_message.npk = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");

    // Act
    bool result = osnma->verify_dsm_pkr(dsm_pkr_message);  // TODO - refactor method so that output is more than a boolean.

    // Assert
    ASSERT_TRUE(result);
}


TEST_F(OsnmaMsgReceiverTest, BuildTagMessageM0)
{
    // input data taken from Receiver Guidelines v1.3,  A.6.5.1
    // Arrange
    std::vector<uint8_t> expected_message = {
        0x02, 0x4E, 0x05, 0x46, 0x3C, 0x01, 0x83, 0xA5, 0x91, 0x05, 0x1D, 0x69, 0x25, 0x80, 0x07, 0x6B,
        0x3E, 0xEA, 0x81, 0x41, 0xBF, 0x03, 0xAD, 0xCB, 0x5A, 0xAD, 0xB2, 0x77, 0xAF, 0x6F, 0xCF, 0x21,
        0xFB, 0x98, 0xFF, 0x7E, 0x83, 0xAF, 0xFC, 0x37, 0x02, 0x03, 0xB0, 0xD8, 0xE1, 0x0E, 0xB1, 0x4D,
        0x11, 0x18, 0xE6, 0xB0, 0xE8, 0x20, 0x01, 0xA0, 0x00, 0xE5, 0x91, 0x00, 0x06, 0xD3, 0x1F, 0x00,
        0x02, 0x68, 0x05, 0x4A, 0x02, 0xC2, 0x26, 0x07, 0xF7, 0xFC, 0x00};

    uint32_t TOW_Tag0 = 345660;
    uint32_t TOW_NavData = TOW_Tag0 - 30;
    uint32_t TOW_Key_Tag0 = TOW_Tag0 + 30;
    uint32_t WN = 1248;
    uint32_t PRNa = 2;
    uint8_t CTR = 1;

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;                                                                                        // 40 bit
    osnma->d_tesla_keys[TOW_Key_Tag0] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6, 0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDB, 0xBC, 0x73};  // K4
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_nav_data_manager->add_navigation_data(
        "000011101001011001000100000101000111010110100100100101100000000000"
        "011101101011001111101110101010000001010000011011111100000011101011"
        "011100101101011010101011011011001001110111101011110110111111001111"
        "001000011111101110011000111111110111111010000011101011111111110000"
        "110111000000100000001110110000110110001110000100001110101100010100"
        "110100010001000110001110011010110000111010000010000000000001101000"
        "000000000011100101100100010000000000000110110100110001111100000000"
        "000000100110100000000101010010100000001011000010001001100000011111"
        "110111111111000000000",
        PRNa, TOW_NavData);
    osnma->d_osnma_data.d_nma_header.nmas = 0b10;

    MACK_tag_and_info MTI;
    MTI.tag = static_cast<uint64_t>(0xE37BC4F858);
    MTI.tag_info.PRN_d = 0x02;
    MTI.tag_info.ADKD = 0x00;
    MTI.tag_info.cop = 0x0F;
    Tag t0(MTI, TOW_Tag0, WN, PRNa, CTR);

    // Act
    auto computed_message = osnma->build_message(t0);

    // Assert
    ASSERT_TRUE(computed_message == expected_message);
}


TEST_F(OsnmaMsgReceiverTest, TagVerification)
{
    // input data taken from Receiver Guidelines v1.3,  A.6.5.1
    // Arrange
    // Tag0
    uint32_t TOW_Tag0 = 345660;
    uint32_t TOW_NavData = TOW_Tag0 - 30;
    uint32_t TOW_Key_Tag0 = TOW_Tag0 + 30;
    uint32_t WN = 1248;
    uint32_t PRNa = 2;
    uint8_t CTR = 1;

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;                                                                                        // 40 bit
    osnma->d_tesla_keys[TOW_Key_Tag0] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6, 0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73};  // K4
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_nav_data_manager->add_navigation_data(
        "000011101001011001000100000101000111010110100100100101100000000000"
        "011101101011001111101110101010000001010000011011111100000011101011"
        "011100101101011010101011011011001001110111101011110110111111001111"
        "001000011111101110011000111111110111111010000011101011111111110000"
        "110111000000100000001110110000110110001110000100001110101100010100"
        "110100010001000110001110011010110000111010000010000000000001101000"
        "000000000011100101100100010000000000000110110100110001111100000000"
        "000000100110100000000101010010100000001011000010001001100000011111"
        "110111111111000000000",
        PRNa, TOW_NavData);
    osnma->d_osnma_data.d_nma_header.nmas = 0b10;

    MACK_tag_and_info MTI;
    MTI.tag = static_cast<uint64_t>(0xE37BC4F858);
    MTI.tag_info.PRN_d = 0x02;
    MTI.tag_info.ADKD = 0x00;
    MTI.tag_info.cop = 0x0F;
    Tag t0(MTI, TOW_Tag0, WN, PRNa, CTR);

    // Act
    bool result_tag0 = osnma->verify_tag(t0);

    // Assert

    // Tag3
    uint32_t TOW_Key_Tag3 = TOW_Tag0 + 30;
    WN = 1248;
    PRNa = 2;
    CTR = 3;

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;                                                                                        // 40 bit
    osnma->d_tesla_keys[TOW_Key_Tag3] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6, 0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73};  // K4
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_nav_data_manager->add_navigation_data(
        "111111111111111111111111111111110000000000000000000000010001001001001000"
        "111000001000100111100010010111111111011110111111111001001100000100000",
        PRNa, TOW_NavData);
    osnma->d_osnma_data.d_nma_header.nmas = 0b10;

    MTI.tag = static_cast<uint64_t>(0x7BB238C883);
    MTI.tag_info.PRN_d = 0x02;
    MTI.tag_info.ADKD = 0x04;
    MTI.tag_info.cop = 0x0F;
    Tag t3(MTI, TOW_Tag0, WN, PRNa, CTR);

    bool result_tag3 = osnma->verify_tag(t3);

    ASSERT_TRUE(result_tag0 && result_tag3);
}


TEST_F(OsnmaMsgReceiverTest, TeslaKeyVerification)
{
    // input data taken from Receiver Guidelines v1.3,  A.5.2
    // Arrange
    osnma->d_tesla_key_verified = false;
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = {0x5B, 0xF8, 0xC9, 0xCB, 0xFC, 0xF7, 0x04, 0x22, 0x08, 0x14, 0x75, 0xFD, 0x44, 0x5D, 0xF0, 0xFF};  // Kroot, TOW 345570 GST_0 - 30
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;                                                                                                    // TABLE 10 --> 128 bits
    osnma->d_osnma_data.d_dsm_kroot_message.alpha = 0x610BDF26D77B;
    osnma->d_GST_SIS = (1248 & 0x00000FFF) << 20 | (345630 & 0x000FFFFF);
    osnma->d_GST_0 = ((1248 & 0x00000FFF) << 20 | (345600 & 0x000FFFFF));                          // applicable time (GST_Kroot + 30)
    osnma->d_GST_Sf = osnma->d_GST_0 + 30 * std::floor((osnma->d_GST_SIS - osnma->d_GST_0) / 30);  // Eq. 3 R.G.

    osnma->d_tesla_keys.insert((std::pair<uint32_t, std::vector<uint8_t>>(345600, {0xEF, 0xF9, 0x99, 0x04, 0x0E, 0x19, 0xB5, 0x70, 0x83, 0x50, 0x60, 0xBE, 0xBD, 0x23, 0xED, 0x92})));  // K1, not needed, just for reference.
    std::vector<uint8_t> key = {0x2D, 0xC3, 0xA3, 0xCD, 0xB1, 0x17, 0xFA, 0xAD, 0xB8, 0x3B, 0x5F, 0x0B, 0x6F, 0xEA, 0x88, 0xEB};                                                        // K2
    uint32_t TOW = 345630;

    // Act
    bool result = osnma->verify_tesla_key(key, TOW);  // TODO - refactor so that output is not a boolean. Or use last_verified_tesla_key?

    // Assert
    ASSERT_TRUE(result);
}


/**
 * @brief Sets the time based on the given input.
 *
 * This function calculates the week number (WN) and time of week (TOW)
 * based on the input time and the GST_START_EPOCH. It then stores the
 * calculated values in the WN and TOW member variables. Finally, it
 * combines the WN and TOW into a single 32-bit value and stores it in
 * the d_GST_SIS member variable.
 *
 * @param input The input time as a tm struct.
 */
void OsnmaMsgReceiverTest::set_time(std::tm& input)
{
    auto epoch_time_point = std::chrono::system_clock::from_time_t(mktime(&GST_START_EPOCH));
    auto input_time_point = std::chrono::system_clock::from_time_t(mktime(&input));

    // Get the duration from epoch in seconds
    auto duration_sec = std::chrono::duration_cast<std::chrono::seconds>(input_time_point - epoch_time_point);

    // Calculate the week number (WN) and time of week (TOW)
    uint32_t sec_in_week = 7 * 24 * 60 * 60;
    uint32_t week_number = duration_sec.count() / sec_in_week;
    uint32_t time_of_week = duration_sec.count() % sec_in_week;
    this->WN = week_number;
    this->TOW = time_of_week + LEAP_SECONDS;
    // Return the week number and time of week as a pair

    // TODO: d_GST_SIS or d_receiver_time? doubt
    // I am assuming that local realisation of receiver is identical to SIS GST time coming from W5 or W0
    this->d_GST_SIS = (this->WN & 0x00000FFF) << 20 | (this->TOW & 0x000FFFFF);
}
