/*!
 * \file osmna_test_vectors.cc
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

#include "gnss_crypto.h"
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

struct TestVector
{
    int svId;
    int numNavBits;
    std::vector<uint8_t> navBits;
};


class OsnmaTestVectors : public ::testing::Test
{
protected:
    std::vector<uint8_t> parseNavBits(const std::string& hex);
    std::vector<TestVector> readTestVectorsFromFile(const std::string& filename);
    std::string bytes_to_str(const std::vector<uint8_t>& bytes);
    std::vector<uint8_t> extract_page_bytes(const TestVector& tv, int byte_index, int num_bytes);
    bool feedOsnmaWithTestVectors(osnma_msg_receiver_sptr osnma_object, std::vector<std::vector<TestVector>> testVectors, std::vector<std::tm> startTimesFiles);
    void set_time(std::tm& input);
    void SetUp() override
    {
    }

    uint32_t d_GST_SIS{};
    uint32_t TOW{};
    uint32_t WN{};
    std::tm GST_START_EPOCH = {0, 0, 0, 22, 8 - 1, 1999 - 1900, 0, 0, 0, 0, 0};  // months start with 0 and years since 1900 in std::tm
    const uint32_t LEAP_SECONDS = 0;
    const int SIZE_PAGE_BYTES{240 / 8};  // total bytes of a page
    const int SIZE_SUBFRAME_PAGES{15};   // number of pages of a subframe
    const int DURATION_SUBFRAME{30};     // duration of a subframe, in seconds// 13 + 5;

    bool d_flag_NPK{false};  // flag for NPK, new MT will be set when the new Kroot is received.
};

TEST_F(OsnmaTestVectors, NominalTestConf1)
{
    // Arrange
    std::string crtFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_1/PublicKey/OSNMA_PublicKey_20230803105952_newPKID_1.crt";
    std::string merkleFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_1/MerkleTree/OSNMA_MerkleTree_20230803105953_newPKID_1.xml";
    osnma_msg_receiver_sptr osnma = osnma_msg_receiver_make(crtFilePath, merkleFilePath);

    std::tm input_time = {0, 0, 5, 16, 8 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::vector<std::tm> input_times = {input_time};

    std::vector<TestVector> testVector = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/configuration_1/16_AUG_2023_GST_05_00_01.csv");
    if (testVector.empty())
        {
            ASSERT_TRUE(false);
        }
    std::vector<std::vector<TestVector>> testVectors = {testVector};

    // Act
    bool result = feedOsnmaWithTestVectors(osnma, testVectors, input_times);
    ASSERT_TRUE(result);

    // Assert
    LOG(INFO) << "Successful tags count= " << osnma->d_count_successful_tags;
    LOG(INFO) << "Failed tags count= " << osnma->d_count_failed_tags;
    LOG(INFO) << "Unverified tags count= " << osnma->d_tags_awaiting_verify.size();
    LOG(INFO) << "Failed Kroot count= " << osnma->d_count_failed_Kroot;
    LOG(INFO) << "Failed PK count= " << osnma->d_count_failed_pubKey;
    LOG(INFO) << "Failed MACSEQ count= " << osnma->d_count_failed_macseq;
    ASSERT_EQ(osnma->d_count_failed_tags, 0);
    ASSERT_EQ(osnma->d_count_failed_Kroot, 0);
    ASSERT_EQ(osnma->d_count_failed_pubKey, 0);
    ASSERT_EQ(osnma->d_count_failed_macseq, 0);
}

TEST_F(OsnmaTestVectors, NominalTestConf2)
{
    // Arrange
    std::string crtFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_2/PublicKey/OSNMA_PublicKey_20230720113300_newPKID_2.crt";  // conf. 2
    std::string merkleFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_2/MerkleTree/OSNMA_MerkleTree_20230720113300_newPKID_2.xml";
    osnma_msg_receiver_sptr osnma = osnma_msg_receiver_make(crtFilePath, merkleFilePath);

    std::tm input_time = {0, 0, 0, 27, 7 - 1, 2023 - 1900, 0, 0, 0, 0, 0};  // conf. 2
    std::vector<std::tm> input_times = {input_time};

    std::vector<TestVector> testVector = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/configuration_2/27_JUL_2023_GST_00_00_01.csv");
    if (testVector.empty())
        {
            ASSERT_TRUE(false);
        }
    std::vector<std::vector<TestVector>> testVectors = {testVector};

    // Act
    bool result = feedOsnmaWithTestVectors(osnma, testVectors, input_times);
    ASSERT_TRUE(result);

    // Assert
    LOG(INFO) << "Successful tags count= " << osnma->d_count_successful_tags;
    LOG(INFO) << "Failed tags count= " << osnma->d_count_failed_tags;
    LOG(INFO) << "Unverified tags count= " << osnma->d_tags_awaiting_verify.size();
    LOG(INFO) << "Failed Kroot count= " << osnma->d_count_failed_Kroot;
    LOG(INFO) << "Failed PK count= " << osnma->d_count_failed_pubKey;
    LOG(INFO) << "Failed MACSEQ count= " << osnma->d_count_failed_macseq;
    ASSERT_EQ(osnma->d_count_failed_tags, 0);
    ASSERT_EQ(osnma->d_count_failed_Kroot, 0);
    ASSERT_EQ(osnma->d_count_failed_pubKey, 0);
    ASSERT_EQ(osnma->d_count_failed_macseq, 0);
}

TEST_F(OsnmaTestVectors, PublicKeyRenewal)
{
    // Arrange
    std::string crtFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_2/PublicKey/OSNMA_PublicKey_20231007041500_PKID_7.crt";
    std::string merkleFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_2/MerkleTree/OSNMA_MerkleTree_20231007041500_PKID_7.xml";
    osnma_msg_receiver_sptr osnma = osnma_msg_receiver_make(crtFilePath, merkleFilePath);

    std::tm input_time_step1 = {0, 45, 2, 7, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::tm input_time_step2 = {0, 45, 3, 7, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::tm input_time_step3 = {0, 45, 4, 7, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::vector<std::tm> input_times = {input_time_step1, input_time_step2, input_time_step3};

    std::vector<TestVector> testVectors_step1 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/npk_step1/07_OCT_2023_GST_02_45_01.csv");
    std::vector<TestVector> testVectors_step2 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/npk_step2/07_OCT_2023_GST_03_45_01.csv");
    std::vector<TestVector> testVectors_step3 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/npk_step3/07_OCT_2023_GST_04_45_01.csv");
    if (testVectors_step1.empty() || testVectors_step2.empty() || testVectors_step3.empty())
        {
            ASSERT_TRUE(false);
        }
    std::vector<std::vector<TestVector>> testVectors = {testVectors_step1, testVectors_step2, testVectors_step3};

    // Act
    d_flag_NPK = true;
    bool result = feedOsnmaWithTestVectors(osnma, testVectors, input_times);
    ASSERT_TRUE(result);

    // Assert
    LOG(INFO) << "Successful tags count= " << osnma->d_count_successful_tags;
    LOG(INFO) << "Failed tags count= " << osnma->d_count_failed_tags;
    LOG(INFO) << "Unverified tags count= " << osnma->d_tags_awaiting_verify.size();
    LOG(INFO) << "Failed Kroot count= " << osnma->d_count_failed_Kroot;
    LOG(INFO) << "Failed PK count= " << osnma->d_count_failed_pubKey;
    LOG(INFO) << "Failed MACSEQ count= " << osnma->d_count_failed_macseq;
    ASSERT_EQ(osnma->d_count_failed_tags, 0);
    ASSERT_EQ(osnma->d_count_failed_Kroot, 0);
    ASSERT_EQ(osnma->d_count_failed_pubKey, 0);
    ASSERT_EQ(osnma->d_count_failed_macseq, 0);
}

TEST_F(OsnmaTestVectors, PublicKeyRevocation)
{
    // Arrange
    std::string crtFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_2/PublicKey/OSNMA_PublicKey_20231007081500_PKID_8.crt";
    std::string merkleFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_2/MerkleTree/OSNMA_MerkleTree_20231007081500_PKID_8.xml";
    osnma_msg_receiver_sptr osnma = osnma_msg_receiver_make(crtFilePath, merkleFilePath);

    std::tm input_time_step1 = {0, 45, 7, 7, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::tm input_time_step2 = {0, 30, 9, 7, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::tm input_time_step3 = {0, 30, 10, 7, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::vector<std::tm> input_times = {input_time_step1, input_time_step2, input_time_step3};

    std::vector<TestVector> testVectors_step1 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/pkrev_step1/07_OCT_2023_GST_07_45_01.csv");
    std::vector<TestVector> testVectors_step2 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/pkrev_step2/07_OCT_2023_GST_09_30_01.csv");
    std::vector<TestVector> testVectors_step3 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/pkrev_step3/07_OCT_2023_GST_10_30_01.csv");
    if (testVectors_step1.empty() || testVectors_step2.empty() || testVectors_step3.empty())
        {
            ASSERT_TRUE(false);
        }
    std::vector<std::vector<TestVector>> testVectors = {testVectors_step1, testVectors_step2, testVectors_step3};

    // Act
    bool result = feedOsnmaWithTestVectors(osnma, testVectors, input_times);
    ASSERT_TRUE(result);

    // Assert
    LOG(INFO) << "Successful tags count= " << osnma->d_count_successful_tags;
    LOG(INFO) << "Failed tags count= " << osnma->d_count_failed_tags;
    LOG(INFO) << "Unverified tags count= " << osnma->d_tags_awaiting_verify.size();
    LOG(INFO) << "Failed Kroot count= " << osnma->d_count_failed_Kroot;
    LOG(INFO) << "Failed PK count= " << osnma->d_count_failed_pubKey;
    LOG(INFO) << "Failed MACSEQ count= " << osnma->d_count_failed_macseq;
    ASSERT_EQ(osnma->d_count_failed_tags, 0);
    ASSERT_EQ(osnma->d_count_failed_Kroot, 0);
    ASSERT_EQ(osnma->d_count_failed_pubKey, 0);
    ASSERT_EQ(osnma->d_count_failed_macseq, 0);
}

TEST_F(OsnmaTestVectors, ChainRenewal)
{
    // Arrange
    std::string crtFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_2/PublicKey/OSNMA_PublicKey_20231007041500_PKID_7.crt";
    std::string merkleFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_2/MerkleTree/OSNMA_MerkleTree_20231007041500_PKID_7.xml";
    osnma_msg_receiver_sptr osnma = osnma_msg_receiver_make(crtFilePath, merkleFilePath);

    std::tm input_time_step1 = {0, 45, 16, 6, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::tm input_time_step2 = {0, 30, 18, 6, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::vector<std::tm> input_times = {input_time_step1, input_time_step2};

    std::vector<TestVector> testVectors_step1 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/eoc_step1/06_OCT_2023_GST_16_45_01.csv");
    std::vector<TestVector> testVectors_step2 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/eoc_step2/06_OCT_2023_GST_18_30_01.csv");
    if (testVectors_step1.empty() || testVectors_step2.empty())
        {
            ASSERT_TRUE(false);
        }
    std::vector<std::vector<TestVector>> testVectors = {testVectors_step1, testVectors_step2};

    // Act
    bool result = feedOsnmaWithTestVectors(osnma, testVectors, input_times);
    ASSERT_TRUE(result);

    // Assert
    LOG(INFO) << "Successful tags count= " << osnma->d_count_successful_tags;
    LOG(INFO) << "Failed tags count= " << osnma->d_count_failed_tags;
    LOG(INFO) << "Unverified tags count= " << osnma->d_tags_awaiting_verify.size();
    LOG(INFO) << "Failed Kroot count= " << osnma->d_count_failed_Kroot;
    LOG(INFO) << "Failed PK count= " << osnma->d_count_failed_pubKey;
    LOG(INFO) << "Failed MACSEQ count= " << osnma->d_count_failed_macseq;
    ASSERT_EQ(osnma->d_count_failed_tags, 0);
    ASSERT_EQ(osnma->d_count_failed_Kroot, 0);
    ASSERT_EQ(osnma->d_count_failed_pubKey, 0);
    ASSERT_EQ(osnma->d_count_failed_macseq, 0);
}

TEST_F(OsnmaTestVectors, ChainRevocation)
{
    // Arrange
    std::string crtFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_2/PublicKey/OSNMA_PublicKey_20231007041500_PKID_7.crt";
    std::string merkleFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_2/MerkleTree/OSNMA_MerkleTree_20231007041500_PKID_7.xml";
    osnma_msg_receiver_sptr osnma = osnma_msg_receiver_make(crtFilePath, merkleFilePath);

    std::tm input_time_step1 = {0, 45, 21, 6, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::tm input_time_step2 = {0, 30, 23, 6, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::tm input_time_step3 = {0, 30, 00, 7, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};

    std::vector<std::tm> input_times = {input_time_step1, input_time_step2, input_time_step3};

    std::vector<TestVector> testVectors_step1 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/crev_step1/06_OCT_2023_GST_21_45_01.csv");
    std::vector<TestVector> testVectors_step2 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/crev_step2/06_OCT_2023_GST_23_30_01.csv");
    std::vector<TestVector> testVectors_step3 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/crev_step3/07_OCT_2023_GST_00_30_01.csv");
    if (testVectors_step1.empty() || testVectors_step2.empty() || testVectors_step3.empty())
        {
            ASSERT_TRUE(false);
        }
    std::vector<std::vector<TestVector>> testVectors = {testVectors_step1, testVectors_step2, testVectors_step3};

    // Act
    bool result = feedOsnmaWithTestVectors(osnma, testVectors, input_times);
    ASSERT_TRUE(result);

    // Assert
    LOG(INFO) << "Successful tags count= " << osnma->d_count_successful_tags;
    LOG(INFO) << "Failed tags count= " << osnma->d_count_failed_tags;
    LOG(INFO) << "Unverified tags count= " << osnma->d_tags_awaiting_verify.size();
    LOG(INFO) << "Failed Kroot count= " << osnma->d_count_failed_Kroot;
    LOG(INFO) << "Failed PK count= " << osnma->d_count_failed_pubKey;
    LOG(INFO) << "Failed MACSEQ count= " << osnma->d_count_failed_macseq;
    ASSERT_EQ(osnma->d_count_failed_tags, 0);
    ASSERT_EQ(osnma->d_count_failed_Kroot, 0);
    ASSERT_EQ(osnma->d_count_failed_pubKey, 0);
    ASSERT_EQ(osnma->d_count_failed_macseq, 0);
}

TEST_F(OsnmaTestVectors, AlertMessage)
{
    // Arrange
    std::string crtFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_3/PublicKey/OSNMA_PublicKey_20231007201500_PKID_1.crt";
    std::string merkleFilePath = std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_3/MerkleTree/OSNMA_MerkleTree_20231007201500_PKID_1.xml";
    osnma_msg_receiver_sptr osnma = osnma_msg_receiver_make(crtFilePath, merkleFilePath);

    std::tm input_time_step1 = {0, 45, 18, 7, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::tm input_time_step2 = {0, 45, 19, 7, 10 - 1, 2023 - 1900, 0, 0, 0, 0, 0};
    std::vector<std::tm> input_times = {input_time_step1, input_time_step2};

    std::vector<TestVector> testVectors_step1 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/oam_step1/07_OCT_2023_GST_18_45_01.csv");
    std::vector<TestVector> testVectors_step2 = readTestVectorsFromFile(std::string(BASE_OSNMA_TEST_VECTORS) + "osnma_test_vectors/oam_step2/07_OCT_2023_GST_19_45_01.csv");
    if (testVectors_step1.empty() || testVectors_step2.empty())
        {
            ASSERT_TRUE(false);
        }
    std::vector<std::vector<TestVector>> testVectors = {testVectors_step1, testVectors_step2};

    // Act
    bool result = feedOsnmaWithTestVectors(osnma, testVectors, input_times);
    ASSERT_TRUE(result);

    // Assert
    LOG(INFO) << "Successful tags count= " << osnma->d_count_successful_tags;
    LOG(INFO) << "Failed tags count= " << osnma->d_count_failed_tags;
    LOG(INFO) << "Unverified tags count= " << osnma->d_tags_awaiting_verify.size();
    LOG(INFO) << "Failed Kroot count= " << osnma->d_count_failed_Kroot;
    LOG(INFO) << "Failed PK count= " << osnma->d_count_failed_pubKey;
    LOG(INFO) << "Failed MACSEQ count= " << osnma->d_count_failed_macseq;
    ASSERT_EQ(osnma->d_count_failed_tags, 0);
    ASSERT_EQ(osnma->d_count_failed_Kroot, 0);
    ASSERT_EQ(osnma->d_count_failed_pubKey, 0);
    ASSERT_EQ(osnma->d_count_failed_macseq, 0);
}

// Auxiliary functions for the OsnmaTestVectorsSimulation test fixture.
// Essentially, they perform same work as the telemetry decoder block, but adapted to the osnma-test-vector files.
bool OsnmaTestVectors::feedOsnmaWithTestVectors(osnma_msg_receiver_sptr osnma_object, std::vector<std::vector<TestVector>> testVectors, std::vector<std::tm> startTimesFiles)
{
    bool end_of_hex_stream;
    int offset_byte{0};
    int byte_index{0};  // index containing the last byte position of the hex stream that was retrieved. Takes advantage that all TVs have same size

    const int DUMMY_PAGE{63};
    bool flag_dummy_page{false};
    // Act
    // loop over all bytes of data. Note: all TestVectors have same amount of data.
    // if needed, add global flags so that particular logic may be done at certain points in between files
    for (size_t test_step = 0; test_step < testVectors.size(); test_step++)
        {
            // set variables for each file
            end_of_hex_stream = false;
            offset_byte = 0;
            byte_index = 0;
            set_time(startTimesFiles[test_step]);
            std::cout << "OsnmaTestVectorsSimulation:"
                      << " d_GST_SIS= " << d_GST_SIS
                      << ", TOW=" << TOW
                      << ", WN=" << WN << std::endl;

            if (test_step == 1 && d_flag_NPK == true)
                {
                    // step 2: this simulates the osnma connecting to the GSC server and downloading the Merkle tree of the next public key
                    osnma_object->read_merkle_xml(
                        std::string(BASE_OSNMA_TEST_VECTORS) + "cryptographic_material/Merkle_tree_2/MerkleTree/OSNMA_MerkleTree_20231007081500_PKID_8.xml");
                }

            while (!end_of_hex_stream)
                {
                    // loop over all SVs, extract a subframe
                    for (const TestVector& tv : testVectors[test_step])
                        {  // loop over all SVs, extract a subframe
                            std::cout << "OsnmaTestVectorsSimulation: SVID (PRN_a) " << tv.svId << std::endl;
                            auto osnmaMsg_sptr = std::make_shared<OSNMA_msg>();
                            std::array<uint8_t, 15> hkroot{};
                            std::array<uint32_t, 15> mack{};
                            byte_index = offset_byte;                             // reset byte_index to the offset position for the next test vector. Offset is updated at the end of each Subframe (every 30 s or 450 Bytes)
                            std::map<uint8_t, std::bitset<128>> words_for_OSNMA;  // structure containing <WORD_NUMBER> and <EXTRACTED_BITS>

                            for (int idx = 0; idx < SIZE_SUBFRAME_PAGES; ++idx)  // extract all pages of a subframe
                                {
                                    // extract bytes of complete page (odd+even) -- extract SIZE_PAGE from tv.navBits, starting from byte_index
                                    std::vector<uint8_t> page_bytes = extract_page_bytes(tv, byte_index, SIZE_PAGE_BYTES);
                                    if (page_bytes.empty())
                                        {
                                            std::cout << "OsnmaTestVectorsSimulation: end of TestVectors \n"
                                                      << "byte_index=" << byte_index << " expected= " << 432000 / 8 << std::endl;
                                            end_of_hex_stream = true;
                                            break;
                                        }
                                    // convert them to bitset representation using bytes_to_string
                                    std::string page_bits = bytes_to_str(page_bytes);
                                    // Extract the 40 OSNMA bits starting from the 18th bit
                                    std::string even_page = page_bits.substr(0, page_bits.size() / 2);
                                    std::string odd_page = page_bits.substr(page_bits.size() / 2);
                                    if (even_page.size() < 120 || odd_page.size() < 120)
                                        {
                                            std::cout << "OsnmaTestVectorsSimulation: error parsing pages" << std::endl;
                                        }
                                    bool even_odd_OK = even_page[0] == '0' && odd_page[0] == '1';
                                    bool page_type_OK = even_page[1] == '0' && odd_page[1] == '0';
                                    bool tail_bits_OK = even_page.substr(even_page.size() - 6) == "000000" && odd_page.substr(odd_page.size() - 6) == "000000";
                                    if (!even_odd_OK || !page_type_OK || !tail_bits_OK)
                                        std::cerr << "OsnmaTestVectorsSimulation: error parsing pages." << std::endl;

                                    std::bitset<112> data_k(even_page.substr(2, 112));
                                    std::bitset<16> data_j(odd_page.substr(2, 16));
                                    std::bitset<112> shifted_data_k = data_k;
                                    uint8_t word_type = static_cast<uint8_t>((shifted_data_k >>= 106).to_ulong());  // word type is the first 6 bits of the word
                                    // std::cout << "OsnmaTestVectorsSimulation: received Word " << static_cast<int>(word_type) << std::endl;
                                    if ((word_type >= 1 && word_type <= 5) || word_type == 6 || word_type == 10)
                                        {
                                            // store raw word
                                            std::bitset<128> data_combined(data_k.to_string() + data_j.to_string());
                                            words_for_OSNMA[word_type] = data_combined;
                                        }
                                    if (word_type == DUMMY_PAGE)
                                        flag_dummy_page = true;

                                    // place it into osnma object.
                                    std::bitset<40> osnmaBits(odd_page.substr(18, 40));

                                    // Extract bits for hkroot and mack
                                    std::bitset<8> hkrootBits(osnmaBits.to_string().substr(0, 8));
                                    std::bitset<32> mackBits(osnmaBits.to_string().substr(8, 32));
                                    hkroot[idx] = static_cast<uint8_t>(hkrootBits.to_ulong());
                                    mack[idx] = static_cast<uint32_t>(mackBits.to_ulong());

                                    byte_index += SIZE_PAGE_BYTES;
                                }

                            // std::cout << "----------" << std::endl;
                            if (end_of_hex_stream)
                                break;
                            if (flag_dummy_page)
                                {
                                    flag_dummy_page = false;
                                    continue;  // skip this SV
                                }

                            // Fill osnma object
                            osnmaMsg_sptr->hkroot = hkroot;
                            osnmaMsg_sptr->mack = mack;

                            osnmaMsg_sptr->TOW_sf0 = d_GST_SIS & 0x000FFFFF;
                            osnmaMsg_sptr->WN_sf0 = (d_GST_SIS & 0xFFF00000) >> 20;
                            osnmaMsg_sptr->PRN = tv.svId;  // PRNa

                            // TODO - refactor this logic, currently it is split
                            // check if words_for_OSNMA 1--> 5 words_for_OSNMA are received => fill EphClockStatus data vector
                            bool ephClockStatusWordsReceived = true;
                            for (int i = 1; i <= 5; ++i)
                                {
                                    if (words_for_OSNMA.find(i) == words_for_OSNMA.end())
                                        {
                                            ephClockStatusWordsReceived = false;
                                            std::cerr << "OsnmaTestVectorsSimulation: error parsing words_for_OSNMA 1->5. "
                                                         "Word "
                                                      << i << " should be received for each subframe but was not." << std::endl;
                                        }
                                }
                            // extract bits as needed by osnma block
                            if (ephClockStatusWordsReceived)
                                {
                                    // Define the starting position and length of bits to extract for each word
                                    std::map<uint8_t, std::pair<uint8_t, uint8_t>> extractionParams = {
                                        {1, {6, 120}},
                                        {2, {6, 120}},
                                        {3, {6, 122}},
                                        {4, {6, 120}},
                                        {5, {6, 67}},
                                    };

                                    // Fill NavData bits -- Iterate over the extraction parameters
                                    std::string nav_data_ADKD_0_12 = "";
                                    for (const auto& param : extractionParams)
                                        {
                                            uint8_t wordKey = param.first;
                                            uint8_t start = param.second.first;
                                            uint8_t length = param.second.second;

                                            // Extract the required bits and fill osnma block
                                            nav_data_ADKD_0_12 += words_for_OSNMA[wordKey].to_string().substr(start, length);
                                        }
                                    // send to osnma block
                                    bool check_size_is_ok = nav_data_ADKD_0_12.size() == 549;
                                    if (check_size_is_ok)
                                        {
                                            std::cout << "Galileo OSNMA: sending ADKD=0/12 navData, PRN_d (" << tv.svId << ") "
                                                      << "TOW_sf=" << osnmaMsg_sptr->TOW_sf0 << std::endl;
                                            const auto tmp_obj_osnma = std::make_shared<std::tuple<uint32_t, std::string, uint32_t>>(  // < PRNd , navDataBits, TOW_Sosf>
                                                tv.svId,
                                                nav_data_ADKD_0_12,
                                                osnmaMsg_sptr->TOW_sf0);
                                            // LOG(INFO) << "|---> Galileo OSNMA :: Telemetry Decoder NavData (PRN_d=" << static_cast<int>(tv.svId) << ", TOW=" << static_cast<int>(osnmaMsg_sptr->TOW_sf0) << "): 0b" << nav_data_ADKD_0_12;
                                            osnma_object->msg_handler_osnma(pmt::make_any(tmp_obj_osnma));
                                        }
                                }

                            // check w6 && w10 is received => fill TimingData data vector
                            bool timingWordsReceived = words_for_OSNMA.find(6) != words_for_OSNMA.end() &&
                                                       words_for_OSNMA.find(10) != words_for_OSNMA.end();
                            // extract bits as needed by osnma block
                            if (timingWordsReceived)
                                {
                                    // Define the starting position and length of bits to extract for each word
                                    std::map<uint8_t, std::pair<uint8_t, uint8_t>> extractionParams = {
                                        {6, {6, 99}},
                                        {10, {86, 42}}};

                                    std::string nav_data_ADKD_4 = "";
                                    // Fill NavData bits -- Iterate over the extraction parameters
                                    for (const auto& param : extractionParams)
                                        {
                                            uint8_t wordKey = param.first;
                                            uint8_t start = param.second.first;
                                            uint8_t length = param.second.second;

                                            // Extract the required bits and fill osnma block
                                            nav_data_ADKD_4 += words_for_OSNMA[wordKey].to_string().substr(start, length);
                                        }
                                    // send to osnma block
                                    bool check_size_is_ok = nav_data_ADKD_4.size() == 141;
                                    if (check_size_is_ok)
                                        {
                                            std::cout << "Galileo OSNMA: sending ADKD=04 navData, PRN_d (" << tv.svId << ") "
                                                      << "TOW_sf=" << osnmaMsg_sptr->TOW_sf0 << std::endl;
                                            const auto tmp_obj_osnma = std::make_shared<std::tuple<uint32_t, std::string, uint32_t>>(  // < PRNd , navDataBits, TOW_Sosf>
                                                tv.svId,
                                                nav_data_ADKD_4,
                                                osnmaMsg_sptr->TOW_sf0);
                                            // LOG(INFO) << "|---> Galileo OSNMA :: Telemetry Decoder NavData (PRN_d=" << static_cast<int>(tv.svId) << ", TOW=" << static_cast<int>(osnmaMsg_sptr->TOW_sf0) << "): 0b" << nav_data_ADKD_4;
                                            osnma_object->msg_handler_osnma(pmt::make_any(tmp_obj_osnma));
                                        }
                                }

                            // Call the handler, as if it came from telemetry decoder block
                            auto temp_obj = pmt::make_any(osnmaMsg_sptr);

                            osnma_object->msg_handler_osnma(temp_obj);  // osnma entry point
                        }
                    if (!end_of_hex_stream)
                        {
                            offset_byte = byte_index;  // update offset for the next subframe
                            d_GST_SIS += DURATION_SUBFRAME;
                            TOW = d_GST_SIS & 0x000FFFFF;
                            WN = (d_GST_SIS & 0xFFF00000) >> 20;
                            std::cout << "OsnmaTestVectorsSimulation:"
                                      << " d_GST_SIS= " << d_GST_SIS
                                      << ", TOW=" << TOW
                                      << ", WN=" << WN << std::endl;
                        }
                }
            if (end_of_hex_stream)
                continue;
        }
    return true;
}

std::vector<TestVector> OsnmaTestVectors::readTestVectorsFromFile(const std::string& filename)
{
    std::ifstream file(filename);
    std::vector<TestVector> testVectors;
    if (!file.is_open())
        {
            std::cerr << "Error reading the file \"" << filename << "\" \n";
            return testVectors;
        }

    std::string line;
    std::getline(file, line);
    if (line != "SVID,NumNavBits,NavBitsHEX\r")
        {
            std::cerr << "Error parsing first line"
                      << "\n";
        }

    while (std::getline(file, line))
        {
            std::stringstream ss(line);
            TestVector tv;

            std::string val;

            std::getline(ss, val, ',');
            tv.svId = std::stoi(val);

            std::getline(ss, val, ',');
            tv.numNavBits = std::stoi(val);

            std::getline(ss, val, ',');
            tv.navBits = OsnmaTestVectors::parseNavBits(val);

            testVectors.push_back(tv);
        }

    return testVectors;
}

std::vector<uint8_t> OsnmaTestVectors::parseNavBits(const std::string& hexadecimal)
{
    std::vector<uint8_t> bytes;

    for (unsigned int i = 0; i < hexadecimal.length() - 1; i += 2)
        {
            std::string byteString = hexadecimal.substr(i, 2);
            uint8_t byte = static_cast<uint8_t>(strtol(byteString.c_str(), nullptr, 16));
            bytes.push_back(byte);
        }
    return bytes;
}

std::string OsnmaTestVectors::bytes_to_str(const std::vector<uint8_t>& bytes)
{
    std::string bit_string;
    bit_string.reserve(bytes.size() * 8);
    for (const auto& byte : bytes)
        {
            std::bitset<8> bits(byte);
            bit_string += bits.to_string();
        }
    return bit_string;
}

/**
 * @brief Extracts a range of bytes from a TestVector's navBits vector.
 *
 * This function extracts a extracts the bytes of complete page (odd+even)
 * from the navBits vector of a TestVector object.
 *
 *
 * @param tv The TestVector object from which to extract bytes.
 * @param byte_index The index of the first byte to extract.
 * @param num_bytes The number of bytes to extract.
 * @return A vector containing the extracted bytes, or an empty vector if extraction is not possible.
 */
std::vector<uint8_t> OsnmaTestVectors::extract_page_bytes(const TestVector& tv, int byte_index, int num_bytes)
{
    // Ensure we don't go beyond the end of tv.navBits
    int num_bytes_to_extract = std::min(num_bytes, static_cast<int>(tv.navBits.size() - byte_index));

    // If byte_index is beyond the end of tv.navBits, return an empty vector
    if (num_bytes_to_extract <= 0)
        {
            return std::vector<uint8_t>();
        }

    // Use std::next to get an iterator to the range to extract
    std::vector<uint8_t> extracted_bytes(tv.navBits.begin() + byte_index, tv.navBits.begin() + byte_index + num_bytes_to_extract);

    return extracted_bytes;
}

/**
 * @brief Sets the time based on the given input.
 *
 * This function calculates the week number (WN) and time of week (TOW)
 * based on the input time and the GST_START_EPOCH. It then stores the
 * calculated values in the WN and TOW member variables. Finally, it
 * combines the WN and TOW into a single 32-bit value and stores it in
 * the d_GST_SIS member variable.
 * \post WN, TOW and GST_SIS are set up based on the input time.
 *
 * @param input The input time as a tm struct.
 */
void OsnmaTestVectors::set_time(std::tm& input)
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
