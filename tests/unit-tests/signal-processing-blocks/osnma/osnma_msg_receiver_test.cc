/*!
 * \file osmna_msg_receiver_test.cc
 * \brief Tests for the osnma_msg_receiver class.
 * \author Carles Fernandez, 2023-2026. cfernandez(at)cttc.es
 *   Cesare Ghionoiu Martinez, 2023-2024. c.ghionoiu-martinez@tu-braunschweig.de
 *
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

#include "Galileo_OSNMA.h"
#include "gnss_crypto.h"
#include "gnss_sdr_filesystem.h"
#include "osnma_helper.h"
#include "osnma_msg_receiver.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <bitset>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>  // for LOG
#else
#include <absl/log/log.h>
#endif

namespace
{
void remove_file_if_present(const std::string& path)
{
    errorlib::error_code ec;
    fs::remove(fs::path(path), ec);
}
}  // namespace

class ScopedDefaultFileBackup
{
public:
    explicit ScopedDefaultFileBackup(std::string path) : d_path(std::move(path))
    {
        std::ifstream file(d_path, std::ios::binary);
        if (file)
            {
                d_had_file = true;
                d_contents.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
            }
    }

    ~ScopedDefaultFileBackup()
    {
        if (d_had_file)
            {
                std::ofstream file(d_path, std::ios::binary | std::ios::trunc);
                file.write(d_contents.data(), d_contents.size());
            }
        else
            {
                remove_file_if_present(d_path);
            }
    }

private:
    std::string d_path;
    std::string d_contents;
    bool d_had_file{false};
};


class OsnmaMsgReceiverTest : public ::testing::Test
{
protected:
    ScopedDefaultFileBackup d_default_pem_backup{PEMFILE_DEFAULT};
    ScopedDefaultFileBackup d_default_pem_metadata_backup{PEMFILE_DEFAULT + ".meta"};
    ScopedDefaultFileBackup d_default_kroot_backup{KROOTFILE_DEFAULT};
    ScopedDefaultFileBackup d_default_kroot_metadata_backup{KROOTFILE_DEFAULT + ".meta"};
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
        remove_file_if_present(PEMFILE_DEFAULT);
        remove_file_if_present(PEMFILE_DEFAULT + ".meta");
        remove_file_if_present(KROOTFILE_DEFAULT);
        remove_file_if_present(KROOTFILE_DEFAULT + ".meta");
        // std::tm input_time = {0, 0, 5, 16, 8 - 1, 2023 - 1900, 0, 0, 0, 0, 0}; // conf. 1
        std::tm input_time = {0, 0, 0, 27, 7 - 1, 2023 - 1900, 0, 0, 0, 0, 0};  // conf. 2
        set_time(input_time);
        osnma = osnma_msg_receiver_make(CRTFILE_DEFAULT, MERKLEFILE_DEFAULT);
    }
};


static std::vector<uint8_t> make_valid_dsm_pkr_p256(Osnma_Helper& helper)
{
    std::vector<uint8_t> dsm_pkr(169, 0);
    dsm_pkr[0] = 0x71;    // NB_DP=7, MID=1
    dsm_pkr[129] = 0x12;  // NPKT=ECDSA P-256, NPKID=2

    std::vector<uint8_t> itn = helper.convert_from_hex_string(
        "7CBE05D9970CFC9E22D0A43A340EF557624453A2E821AADEAC989C405D78BA06"
        "956380BAB0D2C939EC6208151040CCFFCF1FB7156178FD1255BA0AECAAA253F7"
        "407B6C5DD4DF059FF8789474061301E1C34881DB7A367A913A3674300E21EAB1"
        "24EF508389B7D446C3E2ECE8D459FBBD3239A794906F5B1F92469C640164FD87");
    std::copy(itn.cbegin(), itn.cend(), dsm_pkr.begin() + 1);

    std::vector<uint8_t> npk = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");
    std::copy(npk.cbegin(), npk.cend(), dsm_pkr.begin() + 130);

    return dsm_pkr;
}


static std::string bytes_to_upper_hex(const std::vector<uint8_t>& bytes)
{
    std::ostringstream output;
    output << std::hex << std::uppercase << std::setfill('0');
    for (const auto byte : bytes)
        {
            output << std::setw(2) << static_cast<uint32_t>(byte);
        }
    return output.str();
}


static std::string canonical_public_key_fingerprint(uint8_t pkid, uint8_t npkt, const std::vector<uint8_t>& public_key)
{
    if (pkid > 15 || npkt > 15 || public_key.empty())
        {
            return {};
        }
    std::vector<uint8_t> input;
    input.reserve(1 + public_key.size());
    input.push_back(static_cast<uint8_t>(((npkt & 0x0F) << 4) | (pkid & 0x0F)));
    input.insert(input.end(), public_key.cbegin(), public_key.cend());
    Gnss_Crypto crypto;
    return bytes_to_upper_hex(crypto.compute_SHA_256(input));
}


static bool test_file_is_readable(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    return file.good();
}


static std::vector<uint8_t> make_dsm_kroot_with_valid_padding_and_bad_signature(
    uint8_t pkid,
    uint8_t nma_header,
    uint8_t hf = 0)
{
    constexpr uint8_t nb_dk = 1;
    constexpr uint8_t ks = 0;
    constexpr uint8_t ts = 5;
    constexpr uint8_t maclt = 28;
    constexpr size_t l_lk_bytes = 12;
    constexpr size_t l_ds_bytes = 64;
    constexpr size_t fields_length_bytes = 13 + l_lk_bytes + l_ds_bytes;
    constexpr size_t l_dk_bytes = 91;

    std::vector<uint8_t> dsm_kroot(l_dk_bytes, 0);
    dsm_kroot[0] = static_cast<uint8_t>((nb_dk << 4) | pkid);
    dsm_kroot[1] = static_cast<uint8_t>(hf << 2);
    dsm_kroot[2] = static_cast<uint8_t>((ks << 4) | ts);
    dsm_kroot[3] = maclt;
    dsm_kroot[4] = 0x04;  // WN_k = 1258
    dsm_kroot[5] = 0xEA;
    dsm_kroot[6] = 1;  // TOWH_k
    for (size_t i = 0; i < l_lk_bytes; ++i)
        {
            dsm_kroot[13 + i] = static_cast<uint8_t>(0xA0 + i);
        }

    std::vector<uint8_t> msg;
    msg.reserve(fields_length_bytes);
    msg.push_back(nma_header);
    msg.insert(msg.end(), dsm_kroot.cbegin() + 1, dsm_kroot.cbegin() + 13 + l_lk_bytes);
    msg.insert(msg.end(), dsm_kroot.cbegin() + 13 + l_lk_bytes, dsm_kroot.cbegin() + fields_length_bytes);
    Gnss_Crypto crypto;
    const auto hash = crypto.compute_SHA_256(msg);
    dsm_kroot[fields_length_bytes] = hash[0];
    dsm_kroot[fields_length_bytes + 1] = hash[1];
    return dsm_kroot;
}


static void write_mack_bits(std::array<uint8_t, 60>& bytes, size_t bit_offset, size_t bit_length, uint64_t value)
{
    for (size_t bit = 0; bit < bit_length; ++bit)
        {
            const size_t shift = bit_length - bit - 1;
            const uint8_t bit_value = static_cast<uint8_t>((value >> shift) & 0x01);
            const size_t absolute_bit = bit_offset + bit;
            const size_t byte_index = absolute_bit / 8;
            const size_t bit_index = 7 - (absolute_bit % 8);
            if (bit_value != 0)
                {
                    bytes[byte_index] |= static_cast<uint8_t>(1U << bit_index);
                }
            else
                {
                    bytes[byte_index] &= static_cast<uint8_t>(~(1U << bit_index));
                }
        }
}


static std::shared_ptr<OSNMA_msg> make_test_mack_message(uint32_t wn, uint32_t tow, uint32_t prn, const std::vector<uint8_t>& key)
{
    auto msg = std::make_shared<OSNMA_msg>();
    msg->WN_sf0 = wn;
    msg->TOW_sf0 = tow;
    msg->PRN = prn;
    msg->page_valid.fill(1);

    std::array<uint8_t, 60> bytes{};
    constexpr size_t lt_bits = 40;
    constexpr size_t tag_and_info_bits = lt_bits + 16;
    constexpr uint16_t nt = 6;
    write_mack_bits(bytes, 0, lt_bits, 0x0102030405ULL);
    write_mack_bits(bytes, lt_bits, 12, 0x0ABC);
    write_mack_bits(bytes, lt_bits + 12, 4, 1);

    for (uint16_t k = 0; k < nt - 1; ++k)
        {
            const size_t tag_bit_offset = static_cast<size_t>(k + 1U) * tag_and_info_bits;
            write_mack_bits(bytes, tag_bit_offset, lt_bits, 0x1000000000ULL + k);
            write_mack_bits(bytes, tag_bit_offset + lt_bits, 8, prn + k);
            write_mack_bits(bytes, tag_bit_offset + lt_bits + 8, 4, k == 1 ? 4 : 0);
            write_mack_bits(bytes, tag_bit_offset + lt_bits + 12, 4, 1);
        }

    const size_t key_bit_offset = nt * tag_and_info_bits;
    for (size_t i = 0; i < key.size(); ++i)
        {
            write_mack_bits(bytes, key_bit_offset + i * 8U, 8, key[i]);
        }

    for (size_t page = 0; page < msg->mack.size(); ++page)
        {
            const size_t byte_index = page * 4U;
            msg->mack[page] = (static_cast<uint32_t>(bytes[byte_index]) << 24) |
                              (static_cast<uint32_t>(bytes[byte_index + 1]) << 16) |
                              (static_cast<uint32_t>(bytes[byte_index + 2]) << 8) |
                              static_cast<uint32_t>(bytes[byte_index + 3]);
        }
    return msg;
}


TEST(OsnmaNavDataManagerTest, VerifiedDataEmittedOnce)
{
    OSNMA_NavDataManager manager;
    const std::string nav_bits(549, '0');
    manager.add_navigation_data(nav_bits, 2, 1258, 1000);

    MACK_tag_and_info mti;
    mti.tag_info.PRN_d = 2;
    mti.tag_info.ADKD = 0;
    mti.tag_info.cop = 1;
    Tag tag(mti, 1030, 1258, 2, 2, 0b10);
    tag.status = Tag::SUCCESS;
    tag.nav_data = nav_bits;

    std::multimap<uint32_t, Tag> tags;
    tags.insert({tag.TOW, tag});
    manager.update_nav_data(tags, 40);

    EXPECT_EQ(manager.get_verified_data().size(), 1);
    EXPECT_TRUE(manager.get_verified_data().empty());
}


TEST(OsnmaNavDataManagerTest, NavDataAvailableNearWeekStartDoesNotUnderflow)
{
    OSNMA_NavDataManager manager;
    const std::string nav_bits(549, '1');
    manager.add_navigation_data(nav_bits, 2, 1258, 0);

    MACK_tag_and_info mti;
    mti.tag_info.PRN_d = 2;
    mti.tag_info.ADKD = 0;
    mti.tag_info.cop = 1;
    Tag tag(mti, 15, 1258, 2, 1, 0b10);

    EXPECT_TRUE(manager.have_nav_data(tag));
    EXPECT_EQ(manager.get_navigation_data(tag), nav_bits);
}


TEST(OsnmaNavDataManagerTest, NavDataDoesNotAliasAcrossWeeks)
{
    OSNMA_NavDataManager manager;
    const std::string nav_bits(549, '1');
    manager.add_navigation_data(nav_bits, 2, 1258, 0);

    MACK_tag_and_info mti;
    mti.tag_info.PRN_d = 2;
    mti.tag_info.ADKD = 0;
    mti.tag_info.cop = 1;
    Tag next_week_tag(mti, 30, 1259, 2, 1, 0b10);

    EXPECT_FALSE(manager.have_nav_data(next_week_tag));

    manager.add_navigation_data(nav_bits, 2, 1259, 0);
    EXPECT_TRUE(manager.have_nav_data(next_week_tag));
    EXPECT_EQ(manager.get_navigation_data(next_week_tag), nav_bits);
}


TEST(OsnmaNavDataManagerTest, PruneOldNavigationDataDropsExpiredEntries)
{
    OSNMA_NavDataManager manager;
    const std::string nav_bits(549, '1');
    manager.add_navigation_data(nav_bits, 2, 1258, 1000);
    manager.prune_old_navigation_data(1258, 16001);

    MACK_tag_and_info mti;
    mti.tag_info.PRN_d = 2;
    mti.tag_info.ADKD = 0;
    mti.tag_info.cop = 1;
    Tag stale_tag(mti, 1030, 1258, 2, 1, 0b10);

    EXPECT_FALSE(manager.have_nav_data(stale_tag));
}


TEST(OsnmaNavDataManagerTest, TagAccumulationResetsAfterFailedTag)
{
    OSNMA_NavDataManager manager;
    const uint32_t wn = 1258;
    const uint32_t prnd = 2;
    const std::string nav_bits(549, '1');
    manager.add_navigation_data(nav_bits, prnd, wn, 1000);
    manager.add_navigation_data(nav_bits, prnd, wn, 1120);

    auto make_tag = [&](uint32_t tow, Tag::e_verification_status status) {
        MACK_tag_and_info mti{};
        mti.tag_info.PRN_d = prnd;
        mti.tag_info.ADKD = 0;
        mti.tag_info.cop = 1;
        Tag tag(mti, tow, wn, prnd, 1, 0b10);
        tag.status = status;
        tag.nav_data = nav_bits;
        return tag;
    };

    std::multimap<uint32_t, Tag> tags;
    tags.insert({1030, make_tag(1030, Tag::SUCCESS)});
    tags.insert({1060, make_tag(1060, Tag::FAIL)});
    tags.insert({1090, make_tag(1090, Tag::SUCCESS)});
    manager.update_nav_data(tags, 20);

    EXPECT_TRUE(manager.get_verified_data().empty());

    std::multimap<uint32_t, Tag> next_tags;
    next_tags.insert({1120, make_tag(1120, Tag::SUCCESS)});
    manager.update_nav_data(next_tags, 20);

    EXPECT_EQ(manager.get_verified_data().size(), 1);
}


TEST(OsnmaNavDataManagerTest, TagAccumulationRejectsIncoherentGstSf)
{
    OSNMA_NavDataManager manager;
    const uint32_t wn = 1258;
    const uint32_t prnd = 2;
    const std::string nav_bits(549, '1');
    manager.add_navigation_data(nav_bits, prnd, wn, 1000);
    manager.add_navigation_data(nav_bits, prnd, wn, 1105);

    auto make_tag = [&](uint32_t tow) {
        MACK_tag_and_info mti{};
        mti.tag_info.PRN_d = prnd;
        mti.tag_info.ADKD = 0;
        mti.tag_info.cop = 1;
        Tag tag(mti, tow, wn, prnd, 1, 0b10);
        tag.status = Tag::SUCCESS;
        tag.nav_data = nav_bits;
        return tag;
    };

    std::multimap<uint32_t, Tag> tags;
    tags.insert({1030, make_tag(1030)});
    tags.insert({1075, make_tag(1075)});
    manager.update_nav_data(tags, 20);

    EXPECT_TRUE(manager.get_verified_data().empty());

    std::multimap<uint32_t, Tag> next_tags;
    next_tags.insert({1105, make_tag(1105)});
    manager.update_nav_data(next_tags, 20);

    EXPECT_EQ(manager.get_verified_data().size(), 1);
}


TEST(OsnmaNavDataManagerTest, TagAccumulationRequiresSameAdkd)
{
    OSNMA_NavDataManager manager;
    const uint32_t wn = 1258;
    const uint32_t prnd = 2;
    const std::string nav_bits(549, '1');
    manager.add_navigation_data(nav_bits, prnd, wn, 1000);
    manager.add_navigation_data(nav_bits, prnd, wn, 1090);

    auto make_tag = [&](uint32_t tow, uint8_t adkd) {
        MACK_tag_and_info mti{};
        mti.tag_info.PRN_d = prnd;
        mti.tag_info.ADKD = adkd;
        mti.tag_info.cop = 1;
        Tag tag(mti, tow, wn, prnd, 1, 0b10);
        tag.status = Tag::SUCCESS;
        tag.nav_data = nav_bits;
        return tag;
    };

    std::multimap<uint32_t, Tag> tags;
    tags.insert({1030, make_tag(1030, 0)});
    tags.insert({1060, make_tag(1060, 12)});
    manager.update_nav_data(tags, 20);

    EXPECT_TRUE(manager.get_verified_data().empty());

    std::multimap<uint32_t, Tag> next_tags;
    next_tags.insert({1090, make_tag(1090, 12)});
    manager.update_nav_data(next_tags, 20);

    EXPECT_EQ(manager.get_verified_data().size(), 1);
}


TEST(OsnmaNavDataManagerTest, TagAccumulationRespectsCopWindow)
{
    OSNMA_NavDataManager manager;
    const uint32_t wn = 1258;
    const uint32_t prnd = 2;
    const std::string nav_bits(549, '1');
    manager.add_navigation_data(nav_bits, prnd, wn, 1000);
    manager.add_navigation_data(nav_bits, prnd, wn, 1120);

    auto make_tag = [&](uint32_t tow) {
        MACK_tag_and_info mti{};
        mti.tag_info.PRN_d = prnd;
        mti.tag_info.ADKD = 0;
        mti.tag_info.cop = 1;
        Tag tag(mti, tow, wn, prnd, 1, 0b10);
        tag.status = Tag::SUCCESS;
        tag.nav_data = nav_bits;
        return tag;
    };

    std::multimap<uint32_t, Tag> tags;
    tags.insert({1030, make_tag(1030)});
    tags.insert({1090, make_tag(1090)});
    manager.update_nav_data(tags, 20);

    EXPECT_TRUE(manager.get_verified_data().empty());

    std::multimap<uint32_t, Tag> next_tags;
    next_tags.insert({1120, make_tag(1120)});
    manager.update_nav_data(next_tags, 20);

    EXPECT_EQ(manager.get_verified_data().size(), 1);
}


TEST(OsnmaNavDataManagerTest, VerifiedTagCreditsOnlyNavDataInCopWindow)
{
    OSNMA_NavDataManager manager;
    const uint32_t prnd = 2;
    const std::string nav_bits(549, '1');
    manager.add_navigation_data(nav_bits, prnd, 1258, 1000);
    manager.add_navigation_data(nav_bits, prnd, 1259, 1000);

    MACK_tag_and_info mti{};
    mti.tag_info.PRN_d = prnd;
    mti.tag_info.ADKD = 0;
    mti.tag_info.cop = 1;
    Tag tag(mti, 1030, 1259, prnd, 1, 0b10);
    tag.status = Tag::SUCCESS;
    tag.nav_data = nav_bits;

    std::multimap<uint32_t, Tag> tags;
    tags.insert({tag.TOW, tag});
    manager.update_nav_data(tags, 40);

    auto verified = manager.get_verified_data();
    ASSERT_EQ(verified.size(), 1);
    EXPECT_EQ(verified.front().get_wn_sf0(), 1259);
    EXPECT_EQ(verified.front().get_tow_sf0(), 1000);
}


TEST(OsnmaTimeTest, GalileoWeekTowWithOffsetWrapsAtWeekStart)
{
    const auto adkd0_time = osnma::galileo_week_tow_with_offset(1258, 20, -25);
    EXPECT_EQ(adkd0_time.first, 1257);
    EXPECT_EQ(adkd0_time.second, 604795);

    const auto adkd4_time = osnma::galileo_week_tow_with_offset(1258, 4, -5);
    EXPECT_EQ(adkd4_time.first, 1257);
    EXPECT_EQ(adkd4_time.second, 604799);

    const auto normal_time = osnma::galileo_week_tow_with_offset(1258, 30, -25);
    EXPECT_EQ(normal_time.first, 1258);
    EXPECT_EQ(normal_time.second, 5);
}


TEST(OsnmaNavDataManagerTest, ResetTagAccumulationsDropsPartialAuthentication)
{
    OSNMA_NavDataManager manager;
    const uint32_t wn = 1258;
    const uint32_t prnd = 2;
    const std::string nav_bits(549, '1');
    manager.add_navigation_data(nav_bits, prnd, wn, 1000);
    manager.add_navigation_data(nav_bits, prnd, wn, 1090);

    auto make_tag = [&](uint32_t tow) {
        MACK_tag_and_info mti{};
        mti.tag_info.PRN_d = prnd;
        mti.tag_info.ADKD = 0;
        mti.tag_info.cop = 1;
        Tag tag(mti, tow, wn, prnd, 1, 0b10);
        tag.status = Tag::SUCCESS;
        tag.nav_data = nav_bits;
        return tag;
    };

    std::multimap<uint32_t, Tag> first_tag;
    first_tag.insert({1030, make_tag(1030)});
    manager.update_nav_data(first_tag, 20);

    manager.reset_tag_accumulations();

    std::multimap<uint32_t, Tag> second_tag;
    second_tag.insert({1060, make_tag(1060)});
    manager.update_nav_data(second_tag, 20);

    EXPECT_TRUE(manager.get_verified_data().empty());

    std::multimap<uint32_t, Tag> third_tag;
    third_tag.insert({1090, make_tag(1090)});
    manager.update_nav_data(third_tag, 20);

    EXPECT_EQ(manager.get_verified_data().size(), 1);
}


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


TEST_F(OsnmaMsgReceiverTest, ComputeMerkleRootUsesConfiguredHashFunction)
{
    DSM_PKR_message dsm_pkr_message;
    dsm_pkr_message.npkt = 0x01;
    dsm_pkr_message.npktid = 0x2;
    dsm_pkr_message.mid = 0x01;
    const std::vector<uint8_t> base_leaf = helper.convert_from_hex_string("120303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");

    std::vector<uint8_t> vec = helper.convert_from_hex_string(
        "7CBE05D9970CFC9E22D0A43A340EF557624453A2E821AADEAC989C405D78BA06"
        "956380BAB0D2C939EC6208151040CCFFCF1FB7156178FD1255BA0AECAAA253F7"
        "407B6C5DD4DF059FF8789474061301E1C34881DB7A367A913A3674300E21EAB1"
        "24EF508389B7D446C3E2ECE8D459FBBD3239A794906F5B1F92469C640164FD87");
    std::copy(vec.begin(), vec.end(), dsm_pkr_message.itn.begin());

    osnma->d_crypto->set_merkle_tree_hash_function("SHA3-256");
    std::vector<uint8_t> expected_merkle_root = osnma->d_crypto->compute_SHA3_256(base_leaf);
    for (size_t i = 0; i < 4; i++)
        {
            std::vector<uint8_t> node;
            node.reserve(64);
            const auto* itn_start = dsm_pkr_message.itn.begin() + (32 * i);
            const auto* itn_end = itn_start + 32;
            if (((dsm_pkr_message.mid >> i) & 1) != 0)
                {
                    node.insert(node.end(), itn_start, itn_end);
                    node.insert(node.end(), expected_merkle_root.begin(), expected_merkle_root.end());
                }
            else
                {
                    node.insert(node.end(), expected_merkle_root.begin(), expected_merkle_root.end());
                    node.insert(node.end(), itn_start, itn_end);
                }
            expected_merkle_root = osnma->d_crypto->compute_SHA3_256(node);
        }

    EXPECT_EQ(osnma->compute_merkle_root(dsm_pkr_message, base_leaf), expected_merkle_root);
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


TEST_F(OsnmaMsgReceiverTest, CanonicalPublicKeyFingerprintIncludesPkidNpktAndPoint)
{
    const std::vector<uint8_t> public_key = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");
    const std::string fingerprint = osnma->public_key_fingerprint_sha256(2, 1, public_key);

    EXPECT_EQ(fingerprint, canonical_public_key_fingerprint(2, 1, public_key));
    EXPECT_FALSE(fingerprint.empty());
    EXPECT_NE(fingerprint, osnma->public_key_fingerprint_sha256(3, 1, public_key));
    EXPECT_NE(fingerprint, osnma->public_key_fingerprint_sha256(2, 3, public_key));

    std::vector<uint8_t> different_public_key = public_key;
    different_public_key[1] ^= 0x01;
    EXPECT_NE(fingerprint, osnma->public_key_fingerprint_sha256(2, 1, different_public_key));
    EXPECT_TRUE(osnma->public_key_fingerprint_sha256(16, 1, public_key).empty());
    EXPECT_TRUE(osnma->public_key_fingerprint_sha256(2, 16, public_key).empty());
    EXPECT_TRUE(osnma->public_key_fingerprint_sha256(2, 1, std::vector<uint8_t>()).empty());
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

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;                                                                                                                // 40 bit
    osnma->d_tesla_keys[helper.compute_gst(WN, TOW_Key_Tag0)] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6, 0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDB, 0xBC, 0x73};  // K4
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
        PRNa, WN, TOW_NavData);
    osnma->d_osnma_data.d_nma_header.nmas = 0b10;

    MACK_tag_and_info MTI;
    MTI.tag = static_cast<uint64_t>(0xE37BC4F858);
    MTI.tag_info.PRN_d = 0x02;
    MTI.tag_info.ADKD = 0x00;
    MTI.tag_info.cop = 0x0F;
    Tag t0(MTI, TOW_Tag0, WN, PRNa, CTR, osnma->d_osnma_data.d_nma_header.nmas);
    osnma->d_osnma_data.d_nma_header.nmas = 0b01;

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

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;                                                                                                                // 40 bit
    osnma->d_tesla_keys[helper.compute_gst(WN, TOW_Key_Tag0)] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6, 0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73};  // K4
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
        PRNa, WN, TOW_NavData);
    osnma->d_osnma_data.d_nma_header.nmas = 0b10;

    MACK_tag_and_info MTI;
    MTI.tag = static_cast<uint64_t>(0xE37BC4F858);
    MTI.tag_info.PRN_d = 0x02;
    MTI.tag_info.ADKD = 0x00;
    MTI.tag_info.cop = 0x0F;
    Tag t0(MTI, TOW_Tag0, WN, PRNa, CTR, osnma->d_osnma_data.d_nma_header.nmas);

    // Act
    bool result_tag0 = osnma->verify_tag(t0);

    // Assert

    // Tag3
    uint32_t TOW_Key_Tag3 = TOW_Tag0 + 30;
    WN = 1248;
    PRNa = 2;
    CTR = 3;

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;                                                                                                                // 40 bit
    osnma->d_tesla_keys[helper.compute_gst(WN, TOW_Key_Tag3)] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6, 0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73};  // K4
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_nav_data_manager->add_navigation_data(
        "111111111111111111111111111111110000000000000000000000010001001001001000"
        "111000001000100111100010010111111111011110111111111001001100000100000",
        PRNa, WN, TOW_NavData);
    osnma->d_osnma_data.d_nma_header.nmas = 0b10;

    MTI.tag = static_cast<uint64_t>(0x7BB238C883);
    MTI.tag_info.PRN_d = 0x02;
    MTI.tag_info.ADKD = 0x04;
    MTI.tag_info.cop = 0x0F;
    Tag t3(MTI, TOW_Tag0, WN, PRNa, CTR, osnma->d_osnma_data.d_nma_header.nmas);

    bool result_tag3 = osnma->verify_tag(t3);

    ASSERT_TRUE(result_tag0 && result_tag3);
}


TEST_F(OsnmaMsgReceiverTest, NavDataArrivalRetriesPendingTagsImmediately)
{
    const uint32_t TOW_Tag0 = 345660;
    const uint32_t TOW_NavData = TOW_Tag0 - 30;
    const uint32_t TOW_Key_Tag0 = TOW_Tag0 + 30;
    const uint32_t WN = 1248;
    const uint32_t PRNa = 2;
    const uint8_t CTR = 1;
    const std::string nav_bits =
        "000011101001011001000100000101000111010110100100100101100000000000"
        "011101101011001111101110101010000001010000011011111100000011101011"
        "011100101101011010101011011011001001110111101011110110111111001111"
        "001000011111101110011000111111110111111010000011101011111111110000"
        "110111000000100000001110110000110110001110000100001110101100010100"
        "110100010001000110001110011010110000111010000010000000000001101000"
        "000000000011100101100100010000000000000110110100110001111100000000"
        "000000100110100000000101010010100000001011000010001001100000011111"
        "110111111111000000000";

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_tesla_keys[helper.compute_gst(WN, TOW_Key_Tag0)] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6,
        0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73};

    MACK_tag_and_info MTI;
    MTI.tag = static_cast<uint64_t>(0xE37BC4F858);
    MTI.tag_info.PRN_d = PRNa;
    MTI.tag_info.ADKD = 0;
    MTI.tag_info.cop = 0x0F;
    Tag tag(MTI, TOW_Tag0, WN, PRNa, CTR, 0b10);
    osnma->d_tags_awaiting_verify.insert({TOW_Tag0, tag});

    const auto inav_data = std::make_shared<std::tuple<uint32_t, std::string, uint32_t, uint32_t>>(PRNa, nav_bits, WN, TOW_NavData);
    osnma->msg_handler_osnma(pmt::make_any(inav_data));

    EXPECT_EQ(osnma->d_count_successful_tags, 1);
    EXPECT_EQ(osnma->d_count_failed_tags, 0);
    EXPECT_TRUE(osnma->d_tags_awaiting_verify.empty());
}


TEST_F(OsnmaMsgReceiverTest, TimeConstraintAllowsOnlySlowMac)
{
    const uint32_t TOW_Tag0 = 345660;
    const uint32_t TOW_NavData = TOW_Tag0 - 30;
    const uint32_t TOW_Key_Tag0 = TOW_Tag0 + 30;
    const uint32_t WN = 1248;
    const uint32_t PRNa = 2;
    const uint8_t CTR = 1;

    osnma->d_kroot_verified = true;
    osnma->d_tesla_key_verified = true;
    osnma->d_tags_to_verify = {0, 4, 12};
    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    const std::vector<uint8_t> key = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6,
        0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73};
    osnma->d_tesla_keys[helper.compute_gst(WN, TOW_Tag0)] = key;
    osnma->d_tesla_keys[helper.compute_gst(WN, TOW_Key_Tag0)] = key;
    osnma->d_osnma_data.d_mack_message.WN = WN;
    osnma->d_osnma_data.d_mack_message.TOW = TOW_Tag0;
    osnma->d_osnma_data.d_mack_message.PRNa = PRNa;
    osnma->d_osnma_data.d_mack_message.nmas = 0b10;

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
        PRNa, WN, TOW_NavData);

    MACK_tag_and_info MTI;
    MTI.tag = static_cast<uint64_t>(0xE37BC4F858);
    MTI.tag_info.PRN_d = 0x02;
    MTI.tag_info.ADKD = 0x00;
    MTI.tag_info.cop = 0x0F;
    Tag t0(MTI, TOW_Tag0, WN, PRNa, CTR, 0b10, std::vector<uint8_t>{12});
    osnma->d_tags_awaiting_verify.insert({TOW_Tag0, t0});

    osnma->process_mack_message();

    ASSERT_EQ(osnma->d_count_successful_tags, 0);
    ASSERT_EQ(osnma->d_tags_awaiting_verify.size(), 1);
    EXPECT_EQ(osnma->d_tags_awaiting_verify.begin()->second.status, Tag::UNVERIFIED);
}


TEST_F(OsnmaMsgReceiverTest, TimeConstraintDecisionIsStoredWithTag)
{
    MACK_message mack;
    mack.WN = 1258;
    mack.TOW = 1000;
    mack.PRNa = 2;
    mack.nmas = 0b10;
    mack.allowed_adkds = {12};

    Tag tag0(mack);
    EXPECT_EQ(tag0.allowed_adkds, (std::vector<uint8_t>{12}));

    MACK_tag_and_info mti;
    mti.tag_info.PRN_d = 2;
    mti.tag_info.ADKD = 0;
    mti.tag_info.cop = 1;
    Tag tag(mti, 1000, 1258, 2, 2, 0b10, mack.allowed_adkds);
    EXPECT_EQ(tag.allowed_adkds, (std::vector<uint8_t>{12}));
}


TEST_F(OsnmaMsgReceiverTest, TeslaKeyRetainsTimeConstraintDecision)
{
    constexpr uint32_t WN = 1258;
    constexpr uint32_t fast_tag_tow = 1000;
    constexpr uint32_t slow_tag_tow = 700;
    constexpr uint32_t key_tow = 1030;

    osnma->d_tesla_keys[helper.compute_gst(WN, key_tow)] =
        osnma_msg_receiver::VerifiedTeslaKey(std::vector<uint8_t>{0xAA}, std::vector<uint8_t>{12});

    MACK_tag_and_info fast_mti;
    fast_mti.tag_info.PRN_d = 2;
    fast_mti.tag_info.ADKD = 0;
    fast_mti.tag_info.cop = 1;
    Tag fast_tag(fast_mti, fast_tag_tow, WN, 2, 1, 0b10);

    MACK_tag_and_info slow_mti;
    slow_mti.tag_info.PRN_d = 2;
    slow_mti.tag_info.ADKD = 12;
    slow_mti.tag_info.cop = 1;
    Tag slow_tag(slow_mti, slow_tag_tow, WN, 2, 1, 0b10);

    EXPECT_FALSE(osnma->tag_has_key_available(fast_tag));
    EXPECT_TRUE(osnma->tag_has_key_available(slow_tag));
}


TEST_F(OsnmaMsgReceiverTest, TeslaChainResetCanPreserveOrClearDeferredMackQueue)
{
    osnma->d_tesla_keys[helper.compute_gst(1258, 1030)] = {0xAA};
    osnma->d_partial_tesla_keys[helper.compute_gst(1258, 1030)] = osnma_msg_receiver::PartialTeslaKey();
    osnma->d_tesla_key_verified = true;
    osnma->d_last_verified_key_GST = helper.compute_gst(1258, 1030);

    MACK_message pending_mack;
    pending_mack.WN = 1258;
    pending_mack.TOW = 1000;
    osnma->d_macks_awaiting_MACSEQ_verification.push_back(pending_mack);

    MACK_tag_and_info pending_mti;
    pending_mti.tag_info.PRN_d = 2;
    pending_mti.tag_info.ADKD = 0;
    Tag pending_tag(pending_mti, 1000, 1258, 2, 1, 0b10);
    osnma->d_tags_awaiting_verify.insert({pending_tag.TOW, pending_tag});

    osnma_msg_receiver::DeferredMackBlock deferred_block;
    deferred_block.PRN = 2;
    deferred_block.WN_sf0 = 1258;
    deferred_block.TOW_sf0 = 1000;
    osnma->d_mack_blocks_awaiting_kroot.push_back(deferred_block);

    osnma->reset_tesla_chain_state(true);

    EXPECT_TRUE(osnma->d_tesla_keys.empty());
    EXPECT_TRUE(osnma->d_partial_tesla_keys.empty());
    EXPECT_TRUE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    EXPECT_TRUE(osnma->d_tags_awaiting_verify.empty());
    ASSERT_EQ(osnma->d_mack_blocks_awaiting_kroot.size(), 1);
    EXPECT_FALSE(osnma->d_tesla_key_verified);
    EXPECT_EQ(osnma->d_last_verified_key_GST, 0);

    osnma->reset_tesla_chain_state();
    EXPECT_TRUE(osnma->d_mack_blocks_awaiting_kroot.empty());
}


TEST_F(OsnmaMsgReceiverTest, PruneOldTeslaKeysDropsExpiredDisclosureKeys)
{
    const uint32_t stale_key_gst = helper.compute_gst(1258, 1000);
    const uint32_t recent_key_gst = helper.compute_gst(1258, 1280);
    const uint32_t latest_key_gst = helper.compute_gst(1258, 1400);
    osnma->d_tesla_keys[stale_key_gst] = {0x10};
    osnma->d_tesla_keys[recent_key_gst] = {0x20};
    osnma->d_tesla_keys[latest_key_gst] = {0x30};
    osnma->d_last_verified_key_GST = latest_key_gst;

    osnma->prune_old_tesla_keys(latest_key_gst);

    EXPECT_EQ(osnma->d_tesla_keys.find(stale_key_gst), osnma->d_tesla_keys.end());
    EXPECT_NE(osnma->d_tesla_keys.find(recent_key_gst), osnma->d_tesla_keys.end());
    EXPECT_NE(osnma->d_tesla_keys.find(latest_key_gst), osnma->d_tesla_keys.end());
}


TEST_F(OsnmaMsgReceiverTest, TimeConstraintUsesReceiverGuidelineBounds)
{
    auto make_msg = [](uint32_t tow) {
        auto msg = std::make_shared<OSNMA_msg>();
        msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
        msg->hkroot[1] = 0;
        msg->WN_sf0 = 1258;
        msg->TOW_sf0 = tow;
        msg->PRN = 2;
        return msg;
    };

    osnma->d_receiver_time_override = true;

    osnma->d_GST_Rx = helper.compute_gst(1258, 1032);
    osnma->msg_handler_osnma(pmt::make_any(make_msg(1000)));
    EXPECT_EQ(osnma->d_tags_to_verify, (std::vector<uint8_t>{0, 4, 12}));

    osnma->d_GST_Rx = helper.compute_gst(1258, 1044);
    osnma->msg_handler_osnma(pmt::make_any(make_msg(1000)));
    EXPECT_EQ(osnma->d_tags_to_verify, (std::vector<uint8_t>{0, 4, 12}));

    osnma->d_GST_Rx = helper.compute_gst(1258, 1045);
    osnma->msg_handler_osnma(pmt::make_any(make_msg(1000)));
    EXPECT_EQ(osnma->d_tags_to_verify, (std::vector<uint8_t>{12}));

    osnma->d_GST_Rx = helper.compute_gst(1258, 1194);
    osnma->msg_handler_osnma(pmt::make_any(make_msg(1000)));
    EXPECT_EQ(osnma->d_tags_to_verify, (std::vector<uint8_t>{12}));

    osnma->d_GST_Rx = helper.compute_gst(1258, 1195);
    osnma->msg_handler_osnma(pmt::make_any(make_msg(1000)));
    EXPECT_TRUE(osnma->d_tags_to_verify.empty());

    osnma->d_GST_Rx = helper.compute_gst(1258, 1016);
    osnma->msg_handler_osnma(pmt::make_any(make_msg(1000)));
    EXPECT_EQ(osnma->d_tags_to_verify, (std::vector<uint8_t>{0, 4, 12}));

    osnma->d_GST_Rx = helper.compute_gst(1258, 1015);
    osnma->msg_handler_osnma(pmt::make_any(make_msg(1000)));
    EXPECT_EQ(osnma->d_tags_to_verify, (std::vector<uint8_t>{12}));

    osnma->d_GST_Rx = helper.compute_gst(1258, 866);
    osnma->msg_handler_osnma(pmt::make_any(make_msg(1000)));
    EXPECT_EQ(osnma->d_tags_to_verify, (std::vector<uint8_t>{12}));

    osnma->d_GST_Rx = helper.compute_gst(1258, 865);
    osnma->msg_handler_osnma(pmt::make_any(make_msg(1000)));
    EXPECT_TRUE(osnma->d_tags_to_verify.empty());
}


TEST_F(OsnmaMsgReceiverTest, ReplayModeSkipsReceiverTimeConstraint)
{
    auto replay_osnma = osnma_msg_receiver_make(CRTFILE_DEFAULT, MERKLEFILE_DEFAULT, false, true);
    auto msg = std::make_shared<OSNMA_msg>();
    msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
    msg->hkroot[1] = 0;
    msg->WN_sf0 = 1258;
    msg->TOW_sf0 = 1000;
    msg->PRN = 2;

    replay_osnma->d_receiver_time_override = true;
    replay_osnma->d_GST_Rx = helper.compute_gst(1258, 1195);
    replay_osnma->msg_handler_osnma(pmt::make_any(msg));

    EXPECT_TRUE(replay_osnma->d_replay_mode);
    EXPECT_FALSE(replay_osnma->d_strict_mode);
    EXPECT_FALSE(replay_osnma->d_time_constraint_verified);
    EXPECT_EQ(replay_osnma->d_tags_to_verify, (std::vector<uint8_t>{0, 4, 12}));
}


TEST_F(OsnmaMsgReceiverTest, UnverifiedTransitionHeadersDoNotStartTransitions)
{
    auto process_header = [this](uint8_t nmas, uint8_t cpks, uint32_t tow) {
        auto msg = std::make_shared<OSNMA_msg>();
        msg->hkroot[0] = static_cast<uint8_t>((nmas << 6) | (cpks << 1));
        msg->hkroot[1] = 0;
        msg->WN_sf0 = 1258;
        msg->TOW_sf0 = tow;
        osnma->process_osnma_message(msg);
    };

    process_header(2, 4, 1000);  // OP/NPK
    process_header(3, 5, 1030);  // DU/PKREV
    process_header(3, 7, 1060);  // DU/AM
    process_header(2, 2, 1090);  // OP/EOC
    process_header(3, 3, 1120);  // DU/CREV
    process_header(2, 6, 1150);  // OP/NMT

    EXPECT_FALSE(osnma->d_flag_PK_renewal);
    EXPECT_FALSE(osnma->d_flag_PK_revocation);
    EXPECT_FALSE(osnma->d_flag_NPK_set);
    EXPECT_FALSE(osnma->d_flag_alert_message);
    EXPECT_FALSE(osnma->d_flag_alert_message_verified);
    EXPECT_FALSE(osnma->d_flag_chain_renewal);
    EXPECT_FALSE(osnma->d_flag_chain_revocation);
    EXPECT_FALSE(osnma->d_flag_merkle_tree_renewal);
    EXPECT_EQ(osnma->d_GST_PKR_PKREV_start, 0);
    EXPECT_EQ(osnma->d_GST_PKR_AM_start, 0);
    EXPECT_EQ(osnma->d_GST_chain_renewal_start, 0);
    EXPECT_EQ(osnma->d_GST_chain_revocation_start, 0);
    EXPECT_EQ(osnma->d_GST_merkle_tree_renewal_start, 0);
}


TEST_F(OsnmaMsgReceiverTest, VerifiedAlertMessageClearsMerkleMaterial)
{
    const std::vector<uint8_t> active_merkle_root(32, 0xA5);
    const std::vector<uint8_t> candidate_merkle_root(32, 0x5A);
    osnma->set_merkle_root(active_merkle_root);

    Osnma_Merkle_Tree_Material candidate_merkle_tree;
    candidate_merkle_tree.valid = true;
    candidate_merkle_tree.source = "test";
    candidate_merkle_tree.hash_function = "SHA-256";
    candidate_merkle_tree.root = candidate_merkle_root;
    osnma->d_material_manager->set_candidate_merkle_tree(candidate_merkle_tree);

    ASSERT_EQ(osnma->d_crypto->get_merkle_root(), active_merkle_root);
    ASSERT_TRUE(osnma->d_material_manager->active_merkle_tree().valid);
    ASSERT_TRUE(osnma->d_material_manager->candidate_merkle_tree().valid);

    osnma->handle_verified_alert_message();

    EXPECT_TRUE(osnma->d_crypto->get_merkle_root().empty());
    EXPECT_FALSE(osnma->d_material_manager->active_merkle_tree().valid);
    EXPECT_TRUE(osnma->d_material_manager->active_merkle_tree().root.empty());
    EXPECT_FALSE(osnma->d_material_manager->candidate_merkle_tree().valid);
}


TEST_F(OsnmaMsgReceiverTest, UnverifiedDontUseHeaderDoesNotSkipMackProcessing)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 30;
    auto msg = std::make_shared<OSNMA_msg>();
    msg->PRN = 2;
    msg->WN_sf0 = wn;
    msg->TOW_sf0 = tow;

    osnma->d_osnma_data.d_nma_header.nmas = 3;  // Don't Use, not yet authenticated by this MACK.
    osnma->d_osnma_data.d_dsm_kroot_message.wn_k = wn;
    osnma->d_osnma_data.d_dsm_kroot_message.towh_k = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;
    osnma->d_osnma_data.d_dsm_kroot_message.hf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 34;
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = std::vector<uint8_t>(16, 0);
    osnma->d_osnma_data.d_dsm_kroot_message.verified = true;
    osnma->d_kroot_verified = true;
    osnma->d_GST_SIS = helper.compute_gst(wn, tow);
    osnma->d_last_verified_kroot_GST = osnma->d_GST_SIS;
    osnma->d_GST_0 = helper.compute_gst(wn, 0);
    osnma->d_GST_Sf = helper.compute_gst(wn, tow);

    osnma->read_and_process_mack_block(msg);

    ASSERT_EQ(osnma->d_macks_awaiting_MACSEQ_verification.size(), 1);
    EXPECT_EQ(osnma->d_macks_awaiting_MACSEQ_verification.front().nmas, 3);
}


TEST_F(OsnmaMsgReceiverTest, VerifiedDontUseTagStopsNavigationAuthentication)
{
    const uint32_t wn = 1258;
    const uint32_t tag_tow = 1000;
    const uint32_t key_tow = tag_tow + 30;
    const uint32_t prna = 2;
    const std::vector<uint8_t> key = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6,
        0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73};

    osnma->d_kroot_verified = true;
    osnma->d_tesla_key_verified = true;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(wn, tag_tow);
    osnma->d_last_verified_key_GST = helper.compute_gst(wn, key_tow);
    osnma->d_GST_SIS = helper.compute_gst(wn, key_tow);
    osnma->d_GST_Sf = helper.compute_gst(wn, key_tow);
    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_tesla_keys[helper.compute_gst(wn, key_tow)] = key;

    const std::string nav_bits(549, '1');
    osnma->d_nav_data_manager->add_navigation_data(nav_bits, prna, wn, tag_tow - 30);

    MACK_tag_and_info mti{};
    mti.tag_info.PRN_d = prna;
    mti.tag_info.ADKD = 0;
    mti.tag_info.cop = 1;
    Tag tag(mti, tag_tow, wn, prna, 1, 0b11);
    const auto message = osnma->build_message(tag);
    const auto mac = osnma->d_crypto->compute_HMAC_SHA_256(key, message);
    tag.received_tag = (static_cast<uint64_t>(mac[0]) << 32) |
                       (static_cast<uint64_t>(mac[1]) << 24) |
                       (static_cast<uint64_t>(mac[2]) << 16) |
                       (static_cast<uint64_t>(mac[3]) << 8) |
                       static_cast<uint64_t>(mac[4]);
    osnma->d_tags_awaiting_verify.insert({tag.TOW, tag});

    osnma->d_osnma_data.d_mack_message.WN = wn;
    osnma->d_osnma_data.d_mack_message.TOW = key_tow;
    osnma->d_osnma_data.d_mack_message.PRNa = prna;
    osnma->d_osnma_data.d_mack_message.nmas = 0b11;

    osnma->process_mack_message();

    EXPECT_EQ(osnma->d_count_successful_tags, 1);
    EXPECT_EQ(osnma->d_count_failed_tags, 0);
    EXPECT_FALSE(osnma->d_kroot_verified);
    EXPECT_EQ(osnma->d_last_verified_kroot_GST, 0);
    EXPECT_FALSE(osnma->d_tesla_key_verified);
    EXPECT_EQ(osnma->d_last_verified_key_GST, 0);
    EXPECT_TRUE(osnma->d_tesla_keys.empty());
    EXPECT_TRUE(osnma->d_tags_awaiting_verify.empty());
    EXPECT_TRUE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    EXPECT_TRUE(osnma->d_nav_data_manager->get_verified_data().empty());
}


TEST_F(OsnmaMsgReceiverTest, AuthenticatedDontUseClearsDsmAndNavAccumulation)
{
    const uint32_t wn = 1258;
    const uint32_t prnd = 2;
    const std::string nav_bits(549, '1');

    osnma->d_dsm_message[4][0] = 0xAA;
    osnma->d_dsm_id_received[4][0] = 1;
    osnma->d_number_of_blocks[4] = 8;
    osnma->d_dsm_nma_header[4] = 0xFF;
    osnma->d_dsm_first_gst[4] = helper.compute_gst(wn, 1000);
    osnma->d_dsm_first_gst_valid[4] = true;

    osnma->d_kroot_verified = true;
    osnma->d_tesla_key_verified = true;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(wn, 1030);
    osnma->d_last_verified_key_GST = helper.compute_gst(wn, 1030);
    osnma->d_tesla_keys[helper.compute_gst(wn, 1030)] = {0x01, 0x02, 0x03};

    osnma->d_nav_data_manager->add_navigation_data(nav_bits, prnd, wn, 1000);
    auto make_tag = [&](uint32_t tow) {
        MACK_tag_and_info mti{};
        mti.tag_info.PRN_d = prnd;
        mti.tag_info.ADKD = 0;
        mti.tag_info.cop = 1;
        Tag tag(mti, tow, wn, prnd, 1, 0b10);
        tag.status = Tag::SUCCESS;
        tag.nav_data = nav_bits;
        return tag;
    };

    std::multimap<uint32_t, Tag> first_tag;
    first_tag.insert({1030, make_tag(1030)});
    osnma->d_nav_data_manager->update_nav_data(first_tag, 20);

    osnma->handle_authenticated_dont_use_status();

    EXPECT_EQ(osnma->d_dsm_message[4][0], 0);
    EXPECT_EQ(osnma->d_dsm_id_received[4][0], 0);
    EXPECT_EQ(osnma->d_number_of_blocks[4], 0);
    EXPECT_EQ(osnma->d_dsm_nma_header[4], 0);
    EXPECT_EQ(osnma->d_dsm_first_gst[4], 0);
    EXPECT_FALSE(osnma->d_dsm_first_gst_valid[4]);
    EXPECT_FALSE(osnma->d_kroot_verified);
    EXPECT_FALSE(osnma->d_tesla_key_verified);
    EXPECT_TRUE(osnma->d_tesla_keys.empty());
    EXPECT_TRUE(osnma->d_tags_awaiting_verify.empty());
    EXPECT_TRUE(osnma->d_macks_awaiting_MACSEQ_verification.empty());

    std::multimap<uint32_t, Tag> second_tag;
    second_tag.insert({1060, make_tag(1060)});
    osnma->d_nav_data_manager->update_nav_data(second_tag, 20);

    EXPECT_TRUE(osnma->d_nav_data_manager->get_verified_data().empty());
}


TEST_F(OsnmaMsgReceiverTest, StaleKrootStatusSkipsMackProcessing)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 4601;
    const uint32_t prn = 2;
    const std::vector<uint8_t> key = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
        0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F};
    auto msg = make_test_mack_message(wn, tow, prn, key);

    osnma->d_osnma_data.d_nma_header.nmas = 2;
    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;
    osnma->d_osnma_data.d_dsm_kroot_message.hf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 34;
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = std::vector<uint8_t>(16, 0);
    osnma->d_osnma_data.d_dsm_kroot_message.verified = true;
    osnma->d_kroot_verified = true;
    osnma->d_tesla_key_verified = true;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(wn, 1000);
    osnma->d_GST_SIS = helper.compute_gst(wn, tow);
    osnma->d_GST_0 = helper.compute_gst(wn, 970);
    osnma->d_GST_Sf = helper.compute_gst(wn, tow);

    osnma->read_and_process_mack_block(msg);

    EXPECT_TRUE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    EXPECT_FALSE(osnma->d_kroot_verified);
    EXPECT_FALSE(osnma->d_tesla_key_verified);
    EXPECT_EQ(osnma->d_last_verified_kroot_GST, 0);
}


TEST_F(OsnmaMsgReceiverTest, PartialMackKeepsAvailableFixedTags)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 1000;
    const uint32_t prn = 2;
    const std::vector<uint8_t> key = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
    auto msg = make_test_mack_message(wn, tow, prn, key);
    msg->page_valid[4] = 0;  // Break only the second Tag&Info field for TS=40.

    osnma->d_osnma_data.d_nma_header.nmas = 0b10;
    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;
    osnma->d_osnma_data.d_dsm_kroot_message.hf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 34;
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = std::vector<uint8_t>(16, 0);
    osnma->d_kroot_verified = true;
    osnma->d_tesla_key_verified = true;
    osnma->d_GST_SIS = helper.compute_gst(wn, tow);
    osnma->d_last_verified_kroot_GST = osnma->d_GST_SIS;
    osnma->d_GST_0 = helper.compute_gst(wn, tow - 30);
    osnma->d_GST_Sf = helper.compute_gst(wn, tow);
    osnma->d_tesla_keys[helper.compute_gst(wn, tow)] = key;

    osnma->read_and_process_mack_block(msg);

    ASSERT_FALSE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    const auto& parsed_mack = osnma->d_macks_awaiting_MACSEQ_verification.back();
    ASSERT_EQ(parsed_mack.tag_and_info.size(), 5);
    EXPECT_TRUE(parsed_mack.tag_and_info[0].valid);
    EXPECT_FALSE(parsed_mack.tag_and_info[1].valid);
    EXPECT_TRUE(parsed_mack.tag_and_info[2].valid);
    EXPECT_EQ(parsed_mack.key, key);
}


TEST_F(OsnmaMsgReceiverTest, MackWithMissingTag0StillParsesMackBody)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 1000;
    const uint32_t prn = 2;
    const std::vector<uint8_t> key = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
    auto msg = make_test_mack_message(wn, tow, prn, key);
    msg->page_valid[0] = 0;

    osnma->d_osnma_data.d_nma_header.nmas = 0b10;
    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;
    osnma->d_osnma_data.d_dsm_kroot_message.hf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 34;
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = std::vector<uint8_t>(16, 0);
    osnma->d_kroot_verified = true;
    osnma->d_tesla_key_verified = true;
    osnma->d_GST_SIS = helper.compute_gst(wn, tow);
    osnma->d_last_verified_kroot_GST = osnma->d_GST_SIS;
    osnma->d_GST_0 = helper.compute_gst(wn, tow - 30);
    osnma->d_GST_Sf = helper.compute_gst(wn, tow);
    osnma->d_tesla_keys[helper.compute_gst(wn, tow)] = key;

    osnma->read_and_process_mack_block(msg);

    ASSERT_FALSE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    const auto& parsed_mack = osnma->d_macks_awaiting_MACSEQ_verification.back();
    EXPECT_FALSE(parsed_mack.header.tag0_valid);
    EXPECT_TRUE(parsed_mack.header.macseq_valid);
    ASSERT_EQ(parsed_mack.tag_and_info.size(), 5);
    EXPECT_TRUE(parsed_mack.tag_and_info[0].valid);
    EXPECT_EQ(parsed_mack.key, key);
}


TEST_F(OsnmaMsgReceiverTest, AllZeroPageValidityMaskMeansNoMackPagesAvailable)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 1000;
    const uint32_t prn = 2;
    const std::vector<uint8_t> key = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
    auto msg = make_test_mack_message(wn, tow, prn, key);
    msg->page_valid.fill(0);
    msg->page_validity_available = true;

    osnma->d_osnma_data.d_nma_header.nmas = 0b10;
    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;
    osnma->d_osnma_data.d_dsm_kroot_message.hf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 34;
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = std::vector<uint8_t>(16, 0);
    osnma->d_kroot_verified = true;
    osnma->d_tesla_key_verified = true;
    osnma->d_GST_SIS = helper.compute_gst(wn, tow);
    osnma->d_last_verified_kroot_GST = osnma->d_GST_SIS;
    osnma->d_GST_0 = helper.compute_gst(wn, tow - 30);
    osnma->d_GST_Sf = helper.compute_gst(wn, tow);

    osnma->read_and_process_mack_block(msg);

    ASSERT_FALSE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    const auto& parsed_mack = osnma->d_macks_awaiting_MACSEQ_verification.back();
    EXPECT_FALSE(parsed_mack.header.tag0_valid);
    EXPECT_FALSE(parsed_mack.header.macseq_valid);
    EXPECT_TRUE(parsed_mack.key.empty());
    ASSERT_EQ(parsed_mack.tag_and_info.size(), 5);
    EXPECT_TRUE(std::all_of(parsed_mack.tag_and_info.cbegin(), parsed_mack.tag_and_info.cend(),
        [](const MACK_tag_and_info& tag_and_info) { return !tag_and_info.valid; }));
}


TEST_F(OsnmaMsgReceiverTest, PartialTeslaKeyReconstructedAcrossSatellites)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 1000;
    const std::vector<uint8_t> key = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
        0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F};
    auto first_msg = make_test_mack_message(wn, tow, 2, key);
    auto second_msg = make_test_mack_message(wn, tow, 9, key);
    first_msg->page_valid[12] = 0;
    second_msg->page_valid[11] = 0;

    osnma->d_osnma_data.d_nma_header.nmas = 0b10;
    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;
    osnma->d_osnma_data.d_dsm_kroot_message.hf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 34;
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = std::vector<uint8_t>(16, 0);
    osnma->d_kroot_verified = true;
    osnma->d_tesla_key_verified = true;
    osnma->d_GST_SIS = helper.compute_gst(wn, tow);
    osnma->d_last_verified_kroot_GST = osnma->d_GST_SIS;
    osnma->d_GST_0 = helper.compute_gst(wn, tow - 30);
    osnma->d_GST_Sf = helper.compute_gst(wn, tow);

    osnma->read_and_process_mack_block(first_msg);

    ASSERT_FALSE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    EXPECT_TRUE(osnma->d_macks_awaiting_MACSEQ_verification.back().key.empty());
    EXPECT_EQ(osnma->d_partial_tesla_keys.size(), 1);

    osnma->read_and_process_mack_block(second_msg);

    ASSERT_GE(osnma->d_macks_awaiting_MACSEQ_verification.size(), 2);
    EXPECT_EQ(osnma->d_macks_awaiting_MACSEQ_verification.back().key, key);
    EXPECT_TRUE(osnma->d_partial_tesla_keys.empty());
}


TEST_F(OsnmaMsgReceiverTest, TagVerification20Bit)
{
    // Same Receiver Guidelines v1.3 A.6.5.1 data as TagVerification, with TS set to 20-bit tags.
    const uint32_t TOW_Tag0 = 345660;
    const uint32_t TOW_NavData = TOW_Tag0 - 30;
    const uint32_t TOW_Key_Tag0 = TOW_Tag0 + 30;
    const uint32_t WN = 1248;
    const uint32_t PRNa = 2;
    const uint8_t CTR = 1;

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 5;                                                                                                                // 20 bit
    osnma->d_tesla_keys[helper.compute_gst(WN, TOW_Key_Tag0)] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6, 0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73};  // K4
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
        PRNa, WN, TOW_NavData);
    osnma->d_osnma_data.d_nma_header.nmas = 0b10;

    MACK_tag_and_info MTI;
    MTI.tag = static_cast<uint64_t>(0xE37BC);
    MTI.tag_info.PRN_d = 0x02;
    MTI.tag_info.ADKD = 0x00;
    MTI.tag_info.cop = 0x0F;
    Tag t0(MTI, TOW_Tag0, WN, PRNa, CTR, osnma->d_osnma_data.d_nma_header.nmas);

    ASSERT_TRUE(osnma->verify_tag(t0));
    ASSERT_EQ(t0.computed_tag, MTI.tag);
}


TEST_F(OsnmaMsgReceiverTest, DummyCopZeroTagUsesZeroNavigationData)
{
    const uint32_t tow = 1000;
    const uint32_t wn = 1258;
    const uint32_t prna = 2;
    const uint8_t ctr = 1;
    const std::vector<uint8_t> key = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6,
        0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73};

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_tesla_keys[helper.compute_gst(wn, tow + 30)] = key;

    MACK_tag_and_info tag0_mti{};
    tag0_mti.tag_info.PRN_d = prna;
    tag0_mti.tag_info.ADKD = 0;
    tag0_mti.tag_info.cop = 0;
    Tag tag0(tag0_mti, tow, wn, prna, ctr, 0b10);
    std::vector<uint8_t> expected_adkd0_message;
    expected_adkd0_message.push_back(static_cast<uint8_t>(prna));
    const auto gst_bytes = helper.gst_to_uint8(helper.compute_gst(wn, tow));
    expected_adkd0_message.insert(expected_adkd0_message.end(), gst_bytes.cbegin(), gst_bytes.cend());
    expected_adkd0_message.push_back(ctr);
    expected_adkd0_message.push_back(0b10 << 6);
    expected_adkd0_message.resize((8 + 32 + 8 + 2 + 549 + 7) / 8, 0);

    EXPECT_EQ(osnma->build_message(tag0), expected_adkd0_message);

    const auto mac = osnma->d_crypto->compute_HMAC_SHA_256(key, expected_adkd0_message);
    tag0.received_tag = (static_cast<uint64_t>(mac[0]) << 32) |
                        (static_cast<uint64_t>(mac[1]) << 24) |
                        (static_cast<uint64_t>(mac[2]) << 16) |
                        (static_cast<uint64_t>(mac[3]) << 8) |
                        static_cast<uint64_t>(mac[4]);
    EXPECT_TRUE(osnma->verify_tag(tag0));

    MACK_tag_and_info utc_mti{};
    utc_mti.tag_info.PRN_d = 3;
    utc_mti.tag_info.ADKD = 4;
    utc_mti.tag_info.cop = 0;
    Tag utc_tag(utc_mti, tow, wn, prna, 2, 0b10);
    std::vector<uint8_t> expected_adkd4_message;
    expected_adkd4_message.push_back(3);
    expected_adkd4_message.push_back(static_cast<uint8_t>(prna));
    expected_adkd4_message.insert(expected_adkd4_message.end(), gst_bytes.cbegin(), gst_bytes.cend());
    expected_adkd4_message.push_back(2);
    expected_adkd4_message.push_back(0b10 << 6);
    expected_adkd4_message.resize((8 + 8 + 32 + 8 + 2 + 141 + 7) / 8, 0);

    EXPECT_EQ(osnma->build_message(utc_tag), expected_adkd4_message);
}


TEST_F(OsnmaMsgReceiverTest, UnverifiedRevocationHeadersDoNotClearState)
{
    osnma->d_dsm_message[3][0] = 0xAA;
    osnma->d_dsm_id_received[3][2] = 1;
    osnma->d_number_of_blocks[3] = 7;
    osnma->d_dsm_nma_header[3] = 0xBB;
    osnma->d_public_key_verified = true;
    osnma->d_kroot_verified = true;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(1258, 1000);
    osnma->d_tesla_keys[helper.compute_gst(1258, 900)] = {0xAA};
    osnma->d_last_verified_key_GST = helper.compute_gst(1258, 900);
    osnma->d_tesla_key_verified = true;
    MACK_message pending_mack;
    pending_mack.WN = 1258;
    pending_mack.TOW = 900;
    osnma->d_macks_awaiting_MACSEQ_verification.push_back(pending_mack);
    MACK_tag_and_info pending_mti;
    pending_mti.tag_info.PRN_d = 2;
    pending_mti.tag_info.ADKD = 0;
    pending_mti.tag_info.cop = 1;
    Tag pending_tag(pending_mti, 900, 1258, 2, 1, 0b10);
    osnma->d_tags_awaiting_verify.insert({pending_tag.TOW, pending_tag});

    auto osnma_msg = std::make_shared<OSNMA_msg>();
    osnma_msg->hkroot[0] = static_cast<uint8_t>((3 << 6) | (3 << 1));  // NMAS=DU, CPKS=CREV
    osnma_msg->hkroot[1] = static_cast<uint8_t>((4 << 4) | 0);         // DSM_ID=4, BID=0
    osnma_msg->WN_sf0 = 1258;
    osnma_msg->TOW_sf0 = 1000;

    osnma->process_osnma_message(osnma_msg);

    EXPECT_EQ(osnma->d_dsm_message[3][0], 0xAA);
    EXPECT_EQ(osnma->d_dsm_id_received[3][2], 1);
    EXPECT_EQ(osnma->d_number_of_blocks[3], 7);
    EXPECT_EQ(osnma->d_dsm_nma_header[3], 0xBB);
    EXPECT_FALSE(osnma->d_tesla_keys.empty());
    EXPECT_FALSE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    EXPECT_FALSE(osnma->d_tags_awaiting_verify.empty());
    EXPECT_TRUE(osnma->d_tesla_key_verified);
    EXPECT_TRUE(osnma->d_public_key_verified);
    EXPECT_TRUE(osnma->d_kroot_verified);
    EXPECT_NE(osnma->d_last_verified_key_GST, 0);
    EXPECT_FALSE(osnma->d_flag_chain_revocation);
    EXPECT_FALSE(osnma->d_flag_PK_revocation);

    auto pkrev_msg = std::make_shared<OSNMA_msg>();
    pkrev_msg->hkroot[0] = static_cast<uint8_t>((3 << 6) | (5 << 1));  // NMAS=DU, CPKS=PKREV
    pkrev_msg->hkroot[1] = static_cast<uint8_t>((5 << 4) | 0);         // DSM_ID=5, BID=0
    pkrev_msg->WN_sf0 = 1258;
    pkrev_msg->TOW_sf0 = 1030;

    osnma->process_osnma_message(pkrev_msg);

    EXPECT_EQ(osnma->d_dsm_message[3][0], 0xAA);
    EXPECT_EQ(osnma->d_dsm_id_received[3][2], 1);
    EXPECT_EQ(osnma->d_number_of_blocks[3], 7);
    EXPECT_EQ(osnma->d_dsm_nma_header[3], 0xBB);
    EXPECT_FALSE(osnma->d_tesla_keys.empty());
    EXPECT_FALSE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    EXPECT_FALSE(osnma->d_tags_awaiting_verify.empty());
    EXPECT_TRUE(osnma->d_tesla_key_verified);
    EXPECT_TRUE(osnma->d_public_key_verified);
    EXPECT_TRUE(osnma->d_kroot_verified);
    EXPECT_NE(osnma->d_last_verified_key_GST, 0);
    EXPECT_FALSE(osnma->d_flag_chain_revocation);
    EXPECT_FALSE(osnma->d_flag_PK_revocation);
}


TEST_F(OsnmaMsgReceiverTest, AuthenticatedCrevPreservesVerifiedFutureKroot)
{
    DSM_KROOT_message active_kroot;
    active_kroot.pkid = 7;
    active_kroot.cidkr = 0;
    active_kroot.kroot = {0x11};
    active_kroot.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_message = active_kroot;

    DSM_KROOT_message future_kroot;
    future_kroot.pkid = 7;
    future_kroot.cidkr = 1;
    future_kroot.kroot = {0x22, 0x33};
    future_kroot.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_new_message = future_kroot;

    osnma->d_kroot_verified = true;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(1258, 1000);
    osnma->d_tesla_key_verified = true;
    osnma->d_tesla_keys[helper.compute_gst(1258, 970)] = {0xAA};
    osnma->d_GST_SIS = helper.compute_gst(1258, 1000);

    osnma->handle_authenticated_revocation(true, false, true, false);

    EXPECT_FALSE(osnma->d_kroot_verified);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.kroot.size(), 0);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_new_message.kroot, future_kroot.kroot);
    EXPECT_TRUE(osnma->d_osnma_data.d_dsm_kroot_new_message.verified);
    EXPECT_EQ(osnma->d_last_verified_kroot_GST, helper.compute_gst(1258, 1000));
    EXPECT_TRUE(osnma->d_flag_chain_revocation);
    EXPECT_TRUE(osnma->d_tesla_keys.empty());
    EXPECT_FALSE(osnma->d_tesla_key_verified);
}


TEST_F(OsnmaMsgReceiverTest, AuthenticatedPkrevPreservesVerifiedFutureKrootAndCurrentPublicKey)
{
    DSM_KROOT_message future_kroot;
    future_kroot.pkid = 8;
    future_kroot.cidkr = 2;
    future_kroot.kroot = {0x44, 0x55};
    future_kroot.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_new_message = future_kroot;

    Osnma_Public_Key_Material active_public_key;
    active_public_key.valid = true;
    active_public_key.pkid_valid = true;
    active_public_key.pkid = 8;
    active_public_key.key_type = "ECDSA P-256";
    active_public_key.compressed_key = {0x02, 0xAA};
    osnma->d_material_manager->set_active_public_key(active_public_key);
    osnma->d_public_key_verified = true;
    osnma->d_active_public_key_id = 8;
    osnma->d_active_public_key_id_valid = true;
    osnma->d_flag_PK_renewal = true;
    osnma->d_flag_NPK_set = true;
    osnma->d_new_public_key_id = 8;
    osnma->d_new_public_key = {0x02, 0xAA};
    osnma->d_last_verified_kroot_GST = helper.compute_gst(1258, 1000);
    osnma->d_GST_SIS = helper.compute_gst(1258, 1000);

    osnma->handle_authenticated_revocation(false, true, true, true);

    EXPECT_FALSE(osnma->d_kroot_verified);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_new_message.kroot, future_kroot.kroot);
    EXPECT_TRUE(osnma->d_osnma_data.d_dsm_kroot_new_message.verified);
    EXPECT_EQ(osnma->d_last_verified_kroot_GST, helper.compute_gst(1258, 1000));
    EXPECT_TRUE(osnma->d_public_key_verified);
    EXPECT_TRUE(osnma->d_active_public_key_id_valid);
    EXPECT_EQ(osnma->d_active_public_key_id, 8);
    EXPECT_EQ(osnma->d_material_manager->active_public_key().compressed_key, active_public_key.compressed_key);
    EXPECT_TRUE(osnma->d_flag_PK_revocation);
    EXPECT_FALSE(osnma->d_flag_PK_renewal);
    EXPECT_FALSE(osnma->d_flag_NPK_set);
    EXPECT_TRUE(osnma->d_new_public_key.empty());
}


TEST_F(OsnmaMsgReceiverTest, AuthenticatedRevocationPromotesPreservedFutureKrootAtApplicability)
{
    DSM_KROOT_message future_kroot;
    future_kroot.pkid = 8;
    future_kroot.cidkr = 2;
    future_kroot.wn_k = 1258;
    future_kroot.towh_k = 1;
    future_kroot.kroot = {0x44, 0x55};
    future_kroot.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_new_message = future_kroot;

    osnma->d_flag_chain_revocation = true;
    osnma->d_flag_PK_revocation = true;
    osnma->d_GST_chain_revocation_start = helper.compute_gst(1258, 1000);
    osnma->d_GST_PKR_PKREV_start = helper.compute_gst(1258, 1000);
    osnma->d_kroot_verified = false;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(1258, 1000);
    osnma->d_tesla_key_verified = true;
    osnma->d_last_verified_key_GST = helper.compute_gst(1258, 970);
    osnma->d_tesla_keys[helper.compute_gst(1258, 970)] = {0xAA};

    osnma->promote_verified_future_kroot_if_due(helper.compute_gst(1258, 3600));

    EXPECT_FALSE(osnma->d_flag_chain_revocation);
    EXPECT_FALSE(osnma->d_flag_PK_revocation);
    EXPECT_EQ(osnma->d_GST_chain_revocation_start, 0);
    EXPECT_EQ(osnma->d_GST_PKR_PKREV_start, 0);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.pkid, future_kroot.pkid);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.cidkr, future_kroot.cidkr);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.kroot, future_kroot.kroot);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_new_message.kroot.size(), 0);
    EXPECT_TRUE(osnma->d_kroot_verified);
    EXPECT_FALSE(osnma->d_tesla_key_verified);
    EXPECT_TRUE(osnma->d_tesla_keys.empty());
    EXPECT_EQ(osnma->d_last_verified_key_GST, 0);
}


TEST_F(OsnmaMsgReceiverTest, DsmBlockZeroResetsStaleAccumulator)
{
    osnma->d_dsm_message[4][0] = 0x10;
    osnma->d_dsm_message[4][SIZE_DSM_BLOCKS_BYTES] = 0xAA;
    osnma->d_dsm_id_received[4][0] = 1;
    osnma->d_dsm_id_received[4][1] = 1;
    osnma->d_number_of_blocks[4] = 8;
    osnma->d_dsm_nma_header[4] = 0xBB;
    osnma->d_dsm_block_nma_header[4][0] = 0xBB;
    osnma->d_dsm_block_nma_header[4][1] = 0xBB;

    auto osnma_msg = std::make_shared<OSNMA_msg>();
    osnma_msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
    osnma_msg->hkroot[1] = static_cast<uint8_t>((4 << 4) | 0);         // DSM_ID=4, BID=0
    osnma_msg->hkroot[2] = static_cast<uint8_t>(2 << 4);               // NB_DK=2

    osnma->read_dsm_header(osnma_msg->hkroot[1]);
    osnma->read_dsm_block(osnma_msg);

    EXPECT_EQ(osnma->d_dsm_message[4][SIZE_DSM_BLOCKS_BYTES], 0);
    EXPECT_EQ(osnma->d_dsm_id_received[4][1], 0);
    EXPECT_EQ(osnma->d_number_of_blocks[4], 8);
    EXPECT_EQ(osnma->d_dsm_nma_header[4], osnma_msg->hkroot[0]);
    EXPECT_EQ(osnma->d_dsm_id_received[4][0], 1);
    EXPECT_EQ(osnma->d_dsm_block_nma_header[4][0], osnma_msg->hkroot[0]);
    EXPECT_EQ(osnma->d_dsm_block_nma_header[4][1], 0);
}


TEST_F(OsnmaMsgReceiverTest, DsmBlockZeroKeepsCompatibleUnanchoredBlocks)
{
    constexpr uint8_t nma_header = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
    osnma->d_dsm_message[4][SIZE_DSM_BLOCKS_BYTES] = 0xAA;
    osnma->d_dsm_id_received[4][1] = 1;
    osnma->d_dsm_block_nma_header[4][1] = nma_header;

    auto osnma_msg = std::make_shared<OSNMA_msg>();
    osnma_msg->hkroot[0] = nma_header;
    osnma_msg->hkroot[1] = static_cast<uint8_t>((4 << 4) | 0);  // DSM_ID=4, BID=0
    osnma_msg->hkroot[2] = static_cast<uint8_t>(2 << 4);        // NB_DK=2

    osnma->read_dsm_header(osnma_msg->hkroot[1]);
    osnma->read_dsm_block(osnma_msg);

    EXPECT_EQ(osnma->d_dsm_message[4][SIZE_DSM_BLOCKS_BYTES], 0xAA);
    EXPECT_EQ(osnma->d_dsm_id_received[4][1], 1);
    EXPECT_EQ(osnma->d_number_of_blocks[4], 8);
    EXPECT_EQ(osnma->d_dsm_nma_header[4], osnma_msg->hkroot[0]);
    EXPECT_EQ(osnma->d_dsm_id_received[4][0], 1);
    EXPECT_EQ(osnma->d_dsm_block_nma_header[4][0], osnma_msg->hkroot[0]);
    EXPECT_EQ(osnma->d_dsm_block_nma_header[4][1], osnma_msg->hkroot[0]);
}


TEST_F(OsnmaMsgReceiverTest, DsmBlockZeroDropsIncompatibleUnanchoredBlocks)
{
    constexpr uint8_t nma_header = static_cast<uint8_t>((2 << 6) | (1 << 1));        // NMAS=OP, CPKS=Nominal
    constexpr uint8_t other_nma_header = static_cast<uint8_t>((2 << 6) | (1 << 4));  // CID changed
    osnma->d_dsm_message[4][SIZE_DSM_BLOCKS_BYTES] = 0xAA;
    osnma->d_dsm_message[4][2 * SIZE_DSM_BLOCKS_BYTES] = 0xBB;
    osnma->d_dsm_id_received[4][1] = 1;
    osnma->d_dsm_id_received[4][2] = 1;
    osnma->d_dsm_block_nma_header[4][1] = other_nma_header;
    osnma->d_dsm_block_nma_header[4][2] = nma_header;

    auto osnma_msg = std::make_shared<OSNMA_msg>();
    osnma_msg->hkroot[0] = nma_header;
    osnma_msg->hkroot[1] = static_cast<uint8_t>((4 << 4) | 0);  // DSM_ID=4, BID=0
    osnma_msg->hkroot[2] = static_cast<uint8_t>(2 << 4);        // NB_DK=2

    osnma->read_dsm_header(osnma_msg->hkroot[1]);
    osnma->read_dsm_block(osnma_msg);

    EXPECT_EQ(osnma->d_dsm_message[4][SIZE_DSM_BLOCKS_BYTES], 0);
    EXPECT_EQ(osnma->d_dsm_id_received[4][1], 0);
    EXPECT_EQ(osnma->d_dsm_block_nma_header[4][1], 0);
    EXPECT_EQ(osnma->d_dsm_message[4][2 * SIZE_DSM_BLOCKS_BYTES], 0xBB);
    EXPECT_EQ(osnma->d_dsm_id_received[4][2], 1);
    EXPECT_EQ(osnma->d_dsm_block_nma_header[4][2], nma_header);
    EXPECT_EQ(osnma->d_number_of_blocks[4], 8);
    EXPECT_EQ(osnma->d_dsm_nma_header[4], osnma_msg->hkroot[0]);
    EXPECT_EQ(osnma->d_dsm_id_received[4][0], 1);
}


TEST_F(OsnmaMsgReceiverTest, AnchoredDsmBlockOutsideAnnouncedLengthIsIgnored)
{
    constexpr uint8_t dsm_id = 4;
    constexpr uint8_t nma_header = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal

    auto block_zero = std::make_shared<OSNMA_msg>();
    block_zero->hkroot[0] = nma_header;
    block_zero->hkroot[1] = static_cast<uint8_t>((dsm_id << 4) | 0);
    block_zero->hkroot[2] = static_cast<uint8_t>(2 << 4);  // NB_DK=2 -> 8 blocks
    block_zero->WN_sf0 = 1258;
    block_zero->TOW_sf0 = 1000;

    osnma->read_dsm_header(block_zero->hkroot[1]);
    osnma->read_dsm_block(block_zero);
    ASSERT_EQ(osnma->d_number_of_blocks[dsm_id], 8);

    auto out_of_range_block = std::make_shared<OSNMA_msg>();
    out_of_range_block->hkroot[0] = nma_header;
    out_of_range_block->hkroot[1] = static_cast<uint8_t>((dsm_id << 4) | 8);
    out_of_range_block->hkroot[2] = 0xAA;
    out_of_range_block->WN_sf0 = 1258;
    out_of_range_block->TOW_sf0 = 1030;

    osnma->read_dsm_header(out_of_range_block->hkroot[1]);
    osnma->read_dsm_block(out_of_range_block);

    EXPECT_EQ(osnma->d_dsm_id_received[dsm_id][8], 0);
    EXPECT_EQ(osnma->d_dsm_message[dsm_id][8 * SIZE_DSM_BLOCKS_BYTES], 0);
    EXPECT_EQ(osnma->d_number_of_blocks[dsm_id], 8);
    EXPECT_EQ(osnma->d_dsm_id_received[dsm_id][0], 1);
}


TEST_F(OsnmaMsgReceiverTest, DsmNonzeroBlockWithDifferentNmaHeaderResetsAnchoredAccumulator)
{
    constexpr uint8_t dsm_id = 4;

    auto block_zero = std::make_shared<OSNMA_msg>();
    block_zero->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CID=0, CPKS=Nominal
    block_zero->hkroot[1] = static_cast<uint8_t>((dsm_id << 4) | 0);
    block_zero->hkroot[2] = static_cast<uint8_t>(2 << 4);  // NB_DK=2
    block_zero->WN_sf0 = 1258;
    block_zero->TOW_sf0 = 1000;

    osnma->read_dsm_header(block_zero->hkroot[1]);
    osnma->read_dsm_block(block_zero);

    ASSERT_EQ(osnma->d_dsm_id_received[dsm_id][0], 1);
    ASSERT_EQ(osnma->d_number_of_blocks[dsm_id], 8);

    auto mismatched_block = std::make_shared<OSNMA_msg>();
    mismatched_block->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 4) | (1 << 1));  // CID changed to 1
    mismatched_block->hkroot[1] = static_cast<uint8_t>((dsm_id << 4) | 1);
    mismatched_block->hkroot[2] = 0xAA;
    mismatched_block->WN_sf0 = 1258;
    mismatched_block->TOW_sf0 = 1030;

    osnma->read_dsm_header(mismatched_block->hkroot[1]);
    osnma->read_dsm_block(mismatched_block);

    EXPECT_EQ(osnma->d_dsm_id_received[dsm_id][0], 0);
    EXPECT_EQ(osnma->d_dsm_message[dsm_id][0], 0);
    EXPECT_EQ(osnma->d_number_of_blocks[dsm_id], 0);
    EXPECT_EQ(osnma->d_dsm_nma_header[dsm_id], 0);
    EXPECT_EQ(osnma->d_dsm_id_received[dsm_id][1], 1);
    EXPECT_EQ(osnma->d_dsm_message[dsm_id][SIZE_DSM_BLOCKS_BYTES], 0xAA);
    EXPECT_EQ(osnma->d_dsm_block_nma_header[dsm_id][1], mismatched_block->hkroot[0]);
}


TEST_F(OsnmaMsgReceiverTest, RepeatedDsmBlockZeroKeepsPartialAccumulator)
{
    constexpr uint8_t dsm_pkr_id = 12;

    auto osnma_msg = std::make_shared<OSNMA_msg>();
    osnma_msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
    osnma_msg->hkroot[1] = static_cast<uint8_t>((dsm_pkr_id << 4) | 0);
    osnma_msg->hkroot[2] = static_cast<uint8_t>(7 << 4);  // NB_DP=7
    for (size_t i = 3; i < osnma_msg->hkroot.size(); ++i)
        {
            osnma_msg->hkroot[i] = static_cast<uint8_t>(i);
        }

    osnma->read_dsm_header(osnma_msg->hkroot[1]);
    osnma->read_dsm_block(osnma_msg);

    osnma->d_dsm_message[dsm_pkr_id][SIZE_DSM_BLOCKS_BYTES] = 0xAA;
    osnma->d_dsm_id_received[dsm_pkr_id][1] = 1;
    osnma->d_dsm_block_nma_header[dsm_pkr_id][1] = osnma_msg->hkroot[0];

    osnma->read_dsm_header(osnma_msg->hkroot[1]);
    osnma->read_dsm_block(osnma_msg);

    EXPECT_EQ(osnma->d_dsm_message[dsm_pkr_id][SIZE_DSM_BLOCKS_BYTES], 0xAA);
    EXPECT_EQ(osnma->d_dsm_id_received[dsm_pkr_id][1], 1);
    EXPECT_EQ(osnma->d_number_of_blocks[dsm_pkr_id], 13);
    EXPECT_EQ(osnma->d_dsm_nma_header[dsm_pkr_id], osnma_msg->hkroot[0]);
    EXPECT_EQ(osnma->d_dsm_id_received[dsm_pkr_id][0], 1);
}


TEST_F(OsnmaMsgReceiverTest, DsmKrootAccumulatorExpiresAfterOneHour)
{
    auto make_msg = [](uint8_t dsm_id, uint8_t bid, uint32_t tow, uint8_t payload) {
        auto msg = std::make_shared<OSNMA_msg>();
        msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));
        msg->hkroot[1] = static_cast<uint8_t>((dsm_id << 4) | bid);
        msg->hkroot[2] = payload;
        msg->WN_sf0 = 1258;
        msg->TOW_sf0 = tow;
        return msg;
    };

    constexpr uint8_t dsm_kroot_id = 4;
    auto first_msg = make_msg(dsm_kroot_id, 1, 1000, 0xAA);
    osnma->read_dsm_header(first_msg->hkroot[1]);
    osnma->read_dsm_block(first_msg);

    EXPECT_TRUE(osnma->d_dsm_first_gst_valid[dsm_kroot_id]);
    EXPECT_EQ(osnma->d_dsm_first_gst[dsm_kroot_id], helper.compute_gst(1258, 1000));
    EXPECT_EQ(osnma->d_dsm_id_received[dsm_kroot_id][1], 1);

    auto expired_msg = make_msg(dsm_kroot_id, 2, 4600, 0xBB);
    osnma->read_dsm_header(expired_msg->hkroot[1]);
    osnma->read_dsm_block(expired_msg);

    EXPECT_EQ(osnma->d_dsm_id_received[dsm_kroot_id][1], 0);
    EXPECT_EQ(osnma->d_dsm_message[dsm_kroot_id][SIZE_DSM_BLOCKS_BYTES], 0);
    EXPECT_EQ(osnma->d_dsm_id_received[dsm_kroot_id][2], 1);
    EXPECT_EQ(osnma->d_dsm_message[dsm_kroot_id][2 * SIZE_DSM_BLOCKS_BYTES], 0xBB);
    EXPECT_EQ(osnma->d_dsm_first_gst[dsm_kroot_id], helper.compute_gst(1258, 4600));
}


TEST_F(OsnmaMsgReceiverTest, DsmPkrAccumulatorExpiresAfterThirteenHours)
{
    auto make_msg = [](uint8_t dsm_id, uint8_t bid, uint32_t tow, uint8_t payload) {
        auto msg = std::make_shared<OSNMA_msg>();
        msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));
        msg->hkroot[1] = static_cast<uint8_t>((dsm_id << 4) | bid);
        msg->hkroot[2] = payload;
        msg->WN_sf0 = 1258;
        msg->TOW_sf0 = tow;
        return msg;
    };

    constexpr uint8_t dsm_pkr_id = 12;
    auto first_msg = make_msg(dsm_pkr_id, 1, 1000, 0xAA);
    osnma->read_dsm_header(first_msg->hkroot[1]);
    osnma->read_dsm_block(first_msg);

    auto valid_msg = make_msg(dsm_pkr_id, 2, 47799, 0xBB);
    osnma->read_dsm_header(valid_msg->hkroot[1]);
    osnma->read_dsm_block(valid_msg);

    EXPECT_EQ(osnma->d_dsm_id_received[dsm_pkr_id][1], 1);
    EXPECT_EQ(osnma->d_dsm_message[dsm_pkr_id][SIZE_DSM_BLOCKS_BYTES], 0xAA);
    EXPECT_EQ(osnma->d_dsm_id_received[dsm_pkr_id][2], 1);
    EXPECT_EQ(osnma->d_dsm_message[dsm_pkr_id][2 * SIZE_DSM_BLOCKS_BYTES], 0xBB);

    auto expired_msg = make_msg(dsm_pkr_id, 3, 47800, 0xCC);
    osnma->read_dsm_header(expired_msg->hkroot[1]);
    osnma->read_dsm_block(expired_msg);

    EXPECT_EQ(osnma->d_dsm_id_received[dsm_pkr_id][1], 0);
    EXPECT_EQ(osnma->d_dsm_id_received[dsm_pkr_id][2], 0);
    EXPECT_EQ(osnma->d_dsm_message[dsm_pkr_id][SIZE_DSM_BLOCKS_BYTES], 0);
    EXPECT_EQ(osnma->d_dsm_message[dsm_pkr_id][2 * SIZE_DSM_BLOCKS_BYTES], 0);
    EXPECT_EQ(osnma->d_dsm_id_received[dsm_pkr_id][3], 1);
    EXPECT_EQ(osnma->d_dsm_message[dsm_pkr_id][3 * SIZE_DSM_BLOCKS_BYTES], 0xCC);
    EXPECT_EQ(osnma->d_dsm_first_gst[dsm_pkr_id], helper.compute_gst(1258, 47800));
}


TEST_F(OsnmaMsgReceiverTest, UnverifiedNominalHeaderDoesNotResetTransitionLatches)
{
    auto nominal_msg = std::make_shared<OSNMA_msg>();
    nominal_msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
    nominal_msg->hkroot[1] = 0;
    nominal_msg->WN_sf0 = 1258;
    nominal_msg->TOW_sf0 = 2000;

    osnma->d_flag_PK_renewal = true;
    osnma->d_flag_NPK_set = true;
    osnma->d_new_public_key_id = 9;
    osnma->d_new_public_key = {0x01, 0x02};
    osnma->d_GST_PKR_PKREV_start = helper.compute_gst(1258, 1000);
    osnma->process_osnma_message(nominal_msg);
    EXPECT_TRUE(osnma->d_flag_PK_renewal);
    EXPECT_TRUE(osnma->d_flag_NPK_set);
    EXPECT_EQ(osnma->d_new_public_key, (std::vector<uint8_t>{0x01, 0x02}));
    EXPECT_EQ(osnma->d_new_public_key_id, 9);
    EXPECT_EQ(osnma->d_GST_PKR_PKREV_start, helper.compute_gst(1258, 1000));
    osnma->d_flag_PK_renewal = false;
    osnma->d_flag_NPK_set = false;

    osnma->d_flag_PK_revocation = true;
    osnma->d_GST_PKR_PKREV_start = helper.compute_gst(1258, 1500);
    osnma->process_osnma_message(nominal_msg);
    EXPECT_TRUE(osnma->d_flag_PK_revocation);
    EXPECT_EQ(osnma->d_GST_PKR_PKREV_start, helper.compute_gst(1258, 1500));
    osnma->d_flag_PK_revocation = false;
    osnma->d_GST_PKR_PKREV_start = 0;

    osnma->d_flag_chain_renewal = true;
    osnma->d_GST_chain_renewal_start = helper.compute_gst(1258, 1600);
    osnma->process_osnma_message(nominal_msg);
    EXPECT_TRUE(osnma->d_flag_chain_renewal);
    EXPECT_EQ(osnma->d_GST_chain_renewal_start, helper.compute_gst(1258, 1600));

    osnma->d_flag_chain_revocation = true;
    osnma->d_GST_chain_revocation_start = helper.compute_gst(1258, 1700);
    osnma->process_osnma_message(nominal_msg);
    EXPECT_TRUE(osnma->d_flag_chain_revocation);
    EXPECT_EQ(osnma->d_GST_chain_revocation_start, helper.compute_gst(1258, 1700));

    osnma->d_flag_merkle_tree_renewal = true;
    osnma->d_GST_merkle_tree_renewal_start = helper.compute_gst(1258, 1800);
    osnma->process_osnma_message(nominal_msg);
    EXPECT_TRUE(osnma->d_flag_merkle_tree_renewal);
    EXPECT_EQ(osnma->d_GST_merkle_tree_renewal_start, helper.compute_gst(1258, 1800));
}


TEST_F(OsnmaMsgReceiverTest, UnverifiedNominalHeaderKeepsPublicKeyRenewalPending)
{
    auto nominal_msg = std::make_shared<OSNMA_msg>();
    nominal_msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
    nominal_msg->hkroot[1] = 0;
    nominal_msg->WN_sf0 = 1258;
    nominal_msg->TOW_sf0 = 2000;

    osnma->d_flag_PK_renewal = true;
    osnma->d_new_public_key_id = 9;
    osnma->d_new_public_key = {0x01, 0x02};
    osnma->d_GST_PKR_PKREV_start = helper.compute_gst(1258, 1000);

    osnma->process_osnma_message(nominal_msg);

    EXPECT_TRUE(osnma->d_flag_PK_renewal);
    EXPECT_FALSE(osnma->d_flag_NPK_set);
    EXPECT_EQ(osnma->d_GST_PKR_PKREV_start, helper.compute_gst(1258, 1000));
    EXPECT_EQ(osnma->d_new_public_key_id, 9);
    EXPECT_EQ(osnma->d_new_public_key, (std::vector<uint8_t>{0x01, 0x02}));
}


TEST_F(OsnmaMsgReceiverTest, DsmMessageUsesSavedNmaHeaderForChainRouting)
{
    osnma->d_public_key_verified = true;
    osnma->d_flag_chain_renewal = true;
    osnma->d_osnma_data.d_dsm_header.dsm_id = 4;
    osnma->d_osnma_data.d_nma_header.cid = 1;
    osnma->d_crypto->set_public_key_type("ECDSA P-256");

    std::vector<uint8_t> dsm_kroot(8 * SIZE_DSM_BLOCKS_BYTES, 0);
    dsm_kroot[0] = static_cast<uint8_t>((2 << 4) | 1);  // NB_DK=2, PKID=1
    dsm_kroot[1] = static_cast<uint8_t>(1 << 6);        // CIDKR=1
    dsm_kroot[2] = static_cast<uint8_t>((4 << 4) | 5);  // KS=128-bit key, TS=20-bit tag

    const auto saved_nma_header = static_cast<uint8_t>((2 << 6) | (0 << 4) | (2 << 1));  // saved CID=0, CPKS=EOC
    osnma->process_dsm_message(dsm_kroot, saved_nma_header);

    EXPECT_EQ(osnma->d_count_failed_Kroot, 1);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_new_message.nb_dk, 0);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.nb_dk, 0);
}


TEST_F(OsnmaMsgReceiverTest, FailedKrootDoesNotOverwriteActiveState)
{
    osnma->d_public_key_verified = true;
    osnma->d_kroot_verified = true;
    osnma->d_active_public_key_id = 1;
    osnma->d_active_public_key_id_valid = true;
    osnma->d_osnma_data.d_dsm_header.dsm_id = 4;
    osnma->d_crypto->set_public_key_type("ECDSA P-256");
    osnma->d_count_failed_Kroot = 0;

    DSM_KROOT_message trusted_kroot;
    trusted_kroot.nb_dk = 7;
    trusted_kroot.pkid = 3;
    trusted_kroot.cidkr = 1;
    trusted_kroot.ks = 4;
    trusted_kroot.wn_k = 1258;
    trusted_kroot.towh_k = 155;
    trusted_kroot.alpha = 0x010203040506;
    trusted_kroot.kroot = {0xAA, 0xBB, 0xCC};
    trusted_kroot.ds = {0xDD};
    trusted_kroot.p_dk = {0xEE};
    trusted_kroot.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_message = trusted_kroot;

    std::vector<uint8_t> dsm_kroot(8 * SIZE_DSM_BLOCKS_BYTES, 0);
    dsm_kroot[0] = static_cast<uint8_t>((2 << 4) | 1);  // NB_DK=2, PKID=1
    dsm_kroot[2] = static_cast<uint8_t>((4 << 4) | 5);  // KS=128-bit key, TS=20-bit tag

    osnma->process_dsm_message(dsm_kroot, static_cast<uint8_t>((2 << 6) | (1 << 1)));

    EXPECT_EQ(osnma->d_count_failed_Kroot, 1);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.nb_dk, trusted_kroot.nb_dk);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.pkid, trusted_kroot.pkid);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.kroot, trusted_kroot.kroot);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.ds, trusted_kroot.ds);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.p_dk, trusted_kroot.p_dk);
    EXPECT_TRUE(osnma->d_osnma_data.d_dsm_kroot_message.verified);
    EXPECT_TRUE(osnma->d_kroot_verified);
}


TEST_F(OsnmaMsgReceiverTest, KrootWithMismatchedActivePkidIsRejected)
{
    Osnma_Public_Key_Material active_public_key;
    active_public_key.valid = true;
    active_public_key.pkid_valid = true;
    active_public_key.pkid = 2;
    active_public_key.key_type = "ECDSA P-256";
    active_public_key.compressed_key = {0x02, 0xAA};
    osnma->d_material_manager->set_active_public_key(active_public_key);
    osnma->d_public_key_verified = true;
    osnma->d_active_public_key_id = 2;
    osnma->d_active_public_key_id_valid = true;
    osnma->d_kroot_verified = true;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(1258, 1000);
    osnma->d_osnma_data.d_dsm_kroot_message.pkid = 2;
    osnma->d_osnma_data.d_dsm_kroot_message.verified = true;
    osnma->d_tesla_key_verified = true;
    osnma->d_tesla_keys[helper.compute_gst(1258, 1030)] = {0x11, 0x22};
    osnma->d_osnma_data.d_dsm_header.dsm_id = 4;
    osnma->d_crypto->set_public_key_type("ECDSA P-256");
    osnma->d_count_failed_Kroot = 0;

    std::vector<uint8_t> dsm_kroot(8 * SIZE_DSM_BLOCKS_BYTES, 0);
    dsm_kroot[0] = static_cast<uint8_t>((2 << 4) | 1);  // NB_DK=2, PKID=1
    dsm_kroot[2] = static_cast<uint8_t>((4 << 4) | 5);  // KS=128-bit key, TS=20-bit tag

    osnma->process_dsm_message(dsm_kroot, static_cast<uint8_t>((2 << 6) | (1 << 1)));

    EXPECT_EQ(osnma->d_count_failed_Kroot, 1);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.pkid, 2);
    EXPECT_TRUE(osnma->d_osnma_data.d_dsm_kroot_message.verified);
    EXPECT_TRUE(osnma->d_kroot_verified);
    EXPECT_TRUE(osnma->d_public_key_verified);
    EXPECT_TRUE(osnma->d_active_public_key_id_valid);
    EXPECT_EQ(osnma->d_active_public_key_id, 2);
    EXPECT_TRUE(osnma->d_material_manager->active_public_key().valid);
    EXPECT_EQ(osnma->d_last_verified_kroot_GST, helper.compute_gst(1258, 1000));
    EXPECT_TRUE(osnma->d_tesla_key_verified);
    EXPECT_FALSE(osnma->d_tesla_keys.empty());
}


TEST_F(OsnmaMsgReceiverTest, FailedKrootWithMatchingPkidKeepsActivePublicKey)
{
    remove_file_if_present(PEMFILE_DEFAULT);
    remove_file_if_present(PEMFILE_DEFAULT + ".meta");

    const std::vector<uint8_t> public_key = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");
    osnma->d_crypto->set_public_key(public_key);
    ASSERT_TRUE(osnma->d_crypto->store_public_key(PEMFILE_DEFAULT));
    {
        std::ofstream metadata(PEMFILE_DEFAULT + ".meta", std::ios::binary | std::ios::trunc);
        metadata << "version=1\n"
                 << "pkid=1\n"
                 << "npkt=1\n"
                 << "pk_type=ECDSA P-256\n"
                 << "source=DSM-PKR\n"
                 << "public_key_sha256=" << canonical_public_key_fingerprint(1, 1, public_key) << "\n";
    }

    Osnma_Public_Key_Material active_public_key;
    active_public_key.valid = true;
    active_public_key.pkid_valid = true;
    active_public_key.pkid = 1;
    active_public_key.key_type = "ECDSA P-256";
    active_public_key.compressed_key = public_key;
    active_public_key.pem_path = PEMFILE_DEFAULT;
    osnma->d_material_manager->set_active_public_key(active_public_key);
    osnma->d_public_key_verified = true;
    osnma->d_active_public_key_id = 1;
    osnma->d_active_public_key_id_valid = true;
    osnma->d_kroot_verified = true;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(1258, 1000);
    osnma->d_tesla_key_verified = true;
    osnma->d_tesla_keys[helper.compute_gst(1258, 1030)] = {0x11, 0x22};
    osnma->d_osnma_data.d_dsm_header.dsm_id = 4;
    osnma->d_crypto->set_public_key_type("ECDSA P-256");
    osnma->d_count_failed_Kroot = 0;

    const uint8_t nma_header = static_cast<uint8_t>((2 << 6) | (1 << 1));
    const std::vector<uint8_t> dsm_kroot = make_dsm_kroot_with_valid_padding_and_bad_signature(1, nma_header);
    osnma->process_dsm_message(dsm_kroot, nma_header);

    EXPECT_EQ(osnma->d_count_failed_Kroot, 1);
    EXPECT_TRUE(osnma->d_public_key_verified);
    EXPECT_TRUE(osnma->d_active_public_key_id_valid);
    EXPECT_EQ(osnma->d_active_public_key_id, 1);
    EXPECT_TRUE(osnma->d_material_manager->active_public_key().valid);
    EXPECT_EQ(osnma->d_material_manager->active_public_key().compressed_key, public_key);
    EXPECT_TRUE(osnma->d_crypto->have_public_key());
    EXPECT_TRUE(osnma->d_kroot_verified);
    EXPECT_EQ(osnma->d_last_verified_kroot_GST, helper.compute_gst(1258, 1000));
    EXPECT_TRUE(osnma->d_tesla_key_verified);
    EXPECT_FALSE(osnma->d_tesla_keys.empty());
    EXPECT_TRUE(test_file_is_readable(PEMFILE_DEFAULT));
    EXPECT_TRUE(test_file_is_readable(PEMFILE_DEFAULT + ".meta"));
}


TEST_F(OsnmaMsgReceiverTest, DsmKrootPdkUsesSha256WhenHfSha3)
{
    const std::vector<uint8_t> public_key = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");
    osnma->d_crypto->set_public_key(public_key);

    Osnma_Public_Key_Material active_public_key;
    active_public_key.valid = true;
    active_public_key.pkid_valid = true;
    active_public_key.pkid = 1;
    active_public_key.key_type = "ECDSA P-256";
    active_public_key.compressed_key = public_key;
    osnma->d_material_manager->set_active_public_key(active_public_key);
    osnma->d_public_key_verified = true;
    osnma->d_active_public_key_id = 1;
    osnma->d_active_public_key_id_valid = true;
    osnma->d_kroot_verified = true;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(1258, 1000);
    osnma->d_tesla_key_verified = true;
    osnma->d_tesla_keys[helper.compute_gst(1258, 1030)] = {0x11, 0x22};
    osnma->d_osnma_data.d_dsm_header.dsm_id = 4;
    osnma->d_crypto->set_public_key_type("ECDSA P-256");
    osnma->d_count_failed_Kroot = 0;

    const uint8_t nma_header = static_cast<uint8_t>((2 << 6) | (1 << 1));
    const std::vector<uint8_t> dsm_kroot = make_dsm_kroot_with_valid_padding_and_bad_signature(1, nma_header, 2);
    osnma->process_dsm_message(dsm_kroot, nma_header);

    EXPECT_EQ(osnma->d_count_failed_Kroot, 1);
    EXPECT_TRUE(osnma->d_public_key_verified);
    EXPECT_TRUE(osnma->d_active_public_key_id_valid);
    EXPECT_EQ(osnma->d_active_public_key_id, 1);
    EXPECT_TRUE(osnma->d_material_manager->active_public_key().valid);
    EXPECT_EQ(osnma->d_material_manager->active_public_key().compressed_key, public_key);
    EXPECT_TRUE(osnma->d_crypto->have_public_key());
    EXPECT_TRUE(osnma->d_kroot_verified);
    EXPECT_EQ(osnma->d_last_verified_kroot_GST, helper.compute_gst(1258, 1000));
    EXPECT_TRUE(osnma->d_tesla_key_verified);
    EXPECT_FALSE(osnma->d_tesla_keys.empty());
}


TEST_F(OsnmaMsgReceiverTest, CachedKrootWithoutMetadataIsNotPromotedAtStartup)
{
    const uint8_t nma_header = static_cast<uint8_t>((2 << 6) | (1 << 1));
    const std::vector<uint8_t> dsm_kroot = make_dsm_kroot_with_valid_padding_and_bad_signature(1, nma_header);
    {
        std::ofstream file(KROOTFILE_DEFAULT, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(file.good());
        file.write(reinterpret_cast<const char*>(&nma_header), 1);
        file.write(reinterpret_cast<const char*>(dsm_kroot.data()), dsm_kroot.size());
    }
    remove_file_if_present(KROOTFILE_DEFAULT + ".meta");

    EXPECT_FALSE(osnma->load_pending_dsm_kroot_cache());
    EXPECT_FALSE(osnma->d_pending_dsm_kroot_cache.valid);
    EXPECT_FALSE(osnma->d_kroot_verified);
}


TEST_F(OsnmaMsgReceiverTest, CachedKrootMetadataStoresFreshnessAndApplicability)
{
    const std::vector<uint8_t> public_key = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");
    osnma->d_crypto->set_public_key(public_key);
    ASSERT_TRUE(osnma->d_crypto->store_public_key(PEMFILE_DEFAULT));
    osnma->d_crypto->set_public_key_type("ECDSA P-256");

    Osnma_Public_Key_Material active_public_key;
    active_public_key.valid = true;
    active_public_key.pkid_valid = true;
    active_public_key.pkid = 2;
    active_public_key.npkt_valid = true;
    active_public_key.npkt = 1;
    active_public_key.key_type = "ECDSA P-256";
    active_public_key.pem_path = PEMFILE_DEFAULT;
    active_public_key.fingerprint_sha256 = canonical_public_key_fingerprint(2, 1, public_key);
    osnma->d_material_manager->set_active_public_key(active_public_key);
    osnma->d_public_key_verified = true;
    osnma->d_active_public_key_id = 2;
    osnma->d_active_public_key_id_valid = true;
    osnma->d_GST_SIS = helper.compute_gst(1258, 1000);

    DSM_KROOT_message kroot;
    kroot.pkid = 2;
    kroot.cidkr = 1;
    kroot.hf = 0;
    kroot.mf = 0;
    kroot.ks = 4;
    kroot.ts = 9;
    kroot.maclt = 34;
    kroot.wn_k = 1258;
    kroot.towh_k = 1;
    kroot.kroot = std::vector<uint8_t>(16, 0xA5);
    kroot.verified = true;

    const uint8_t nma_header = static_cast<uint8_t>((2 << 6) | (1 << 4) | (1 << 1));
    const std::vector<uint8_t> dsm_kroot = {0x42, 0x11, 0x49, 0x22, 0x04, 0xEA, 0x01, 0x01};
    ASSERT_TRUE(osnma->store_dsm_kroot(dsm_kroot, nma_header, kroot));

    ASSERT_TRUE(test_file_is_readable(KROOTFILE_DEFAULT + ".meta"));
    osnma->d_pending_dsm_kroot_cache = osnma_msg_receiver::CachedDsmKroot();
    ASSERT_TRUE(osnma->load_pending_dsm_kroot_cache());

    EXPECT_TRUE(osnma->d_pending_dsm_kroot_cache.metadata_valid);
    EXPECT_EQ(osnma->d_pending_dsm_kroot_cache.pkid, 2);
    EXPECT_EQ(osnma->d_pending_dsm_kroot_cache.cidkr, 1);
    EXPECT_EQ(osnma->d_pending_dsm_kroot_cache.gst0, helper.compute_gst(1258, 3600));
    EXPECT_EQ(osnma->d_pending_dsm_kroot_cache.kroot_gst, helper.compute_gst(1258, 3570));
    EXPECT_EQ(osnma->d_pending_dsm_kroot_cache.signature_verified_at_gst, helper.compute_gst(1258, 1000));
    EXPECT_EQ(osnma->d_pending_dsm_kroot_cache.public_key_fingerprint_sha256, canonical_public_key_fingerprint(2, 1, public_key));
    EXPECT_TRUE(osnma->d_pending_dsm_kroot_cache.public_key_npkt_valid);
    EXPECT_EQ(osnma->d_pending_dsm_kroot_cache.public_key_npkt, 1);
}


TEST_F(OsnmaMsgReceiverTest, StaleCachedKrootMetadataIsRejected)
{
    const std::vector<uint8_t> public_key = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");
    osnma->d_crypto->set_public_key(public_key);
    ASSERT_TRUE(osnma->d_crypto->store_public_key(PEMFILE_DEFAULT));
    osnma->d_crypto->set_public_key_type("ECDSA P-256");

    Osnma_Public_Key_Material active_public_key;
    active_public_key.valid = true;
    active_public_key.pkid_valid = true;
    active_public_key.pkid = 2;
    active_public_key.npkt_valid = true;
    active_public_key.npkt = 1;
    active_public_key.key_type = "ECDSA P-256";
    active_public_key.pem_path = PEMFILE_DEFAULT;
    active_public_key.fingerprint_sha256 = canonical_public_key_fingerprint(2, 1, public_key);
    osnma->d_material_manager->set_active_public_key(active_public_key);
    osnma->d_public_key_verified = true;
    osnma->d_active_public_key_id = 2;
    osnma->d_active_public_key_id_valid = true;
    osnma->d_GST_SIS = helper.compute_gst(1258, 1000);

    DSM_KROOT_message kroot;
    kroot.pkid = 2;
    kroot.cidkr = 1;
    kroot.hf = 0;
    kroot.mf = 0;
    kroot.ks = 4;
    kroot.ts = 9;
    kroot.maclt = 34;
    kroot.wn_k = 1258;
    kroot.towh_k = 0;
    kroot.kroot = std::vector<uint8_t>(16, 0x5A);
    kroot.verified = true;

    const uint8_t nma_header = static_cast<uint8_t>((2 << 6) | (1 << 4) | (1 << 1));
    const std::vector<uint8_t> dsm_kroot = {0x42, 0x11, 0x49, 0x22, 0x04, 0xEA, 0x00, 0x01};
    ASSERT_TRUE(osnma->store_dsm_kroot(dsm_kroot, nma_header, kroot));
    ASSERT_TRUE(osnma->load_pending_dsm_kroot_cache());

    osnma->evaluate_pending_dsm_kroot_cache(helper.compute_gst(1258, 4600));

    EXPECT_FALSE(osnma->d_pending_dsm_kroot_cache.valid);
    EXPECT_FALSE(osnma->d_kroot_verified);
    EXPECT_FALSE(osnma->d_kroot_loaded_from_cache);
}


TEST_F(OsnmaMsgReceiverTest, ChainRenewalDoesNotPromoteUnverifiedKroot)
{
    auto nominal_msg = std::make_shared<OSNMA_msg>();
    nominal_msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
    nominal_msg->hkroot[1] = 0;
    nominal_msg->WN_sf0 = 1258;
    nominal_msg->TOW_sf0 = 2000;

    DSM_KROOT_message active_kroot;
    active_kroot.nb_dk = 3;
    active_kroot.pkid = 1;
    active_kroot.kroot = {0x11, 0x22};
    active_kroot.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_message = active_kroot;

    DSM_KROOT_message pending_kroot;
    pending_kroot.nb_dk = 4;
    pending_kroot.pkid = 2;
    pending_kroot.wn_k = 1258;
    pending_kroot.towh_k = 1;
    pending_kroot.kroot = {0x33, 0x44};
    pending_kroot.verified = false;
    osnma->d_osnma_data.d_dsm_kroot_new_message = pending_kroot;

    osnma->d_flag_chain_renewal = true;
    osnma->d_GST_chain_renewal_start = helper.compute_gst(1258, 1000);
    osnma->d_kroot_verified = true;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(1258, 2000);
    osnma->d_tesla_key_verified = true;
    osnma->d_tesla_keys[helper.compute_gst(1258, 1100)] = {0xAA};
    osnma->d_last_verified_key_GST = helper.compute_gst(1258, 1100);
    MACK_message pending_mack;
    pending_mack.WN = 1258;
    pending_mack.TOW = 1100;
    osnma->d_macks_awaiting_MACSEQ_verification.push_back(pending_mack);
    MACK_tag_and_info pending_mti;
    pending_mti.tag_info.PRN_d = 2;
    pending_mti.tag_info.ADKD = 0;
    pending_mti.tag_info.cop = 1;
    Tag pending_tag(pending_mti, 1100, 1258, 2, 1, 0b10);
    osnma->d_tags_awaiting_verify.insert({pending_tag.TOW, pending_tag});

    osnma->process_osnma_message(nominal_msg);

    EXPECT_TRUE(osnma->d_flag_chain_renewal);
    EXPECT_EQ(osnma->d_GST_chain_renewal_start, helper.compute_gst(1258, 1000));
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.nb_dk, active_kroot.nb_dk);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.kroot, active_kroot.kroot);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_new_message.nb_dk, pending_kroot.nb_dk);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_new_message.kroot, pending_kroot.kroot);
    EXPECT_TRUE(osnma->d_kroot_verified);
    EXPECT_TRUE(osnma->d_tesla_key_verified);
    EXPECT_FALSE(osnma->d_tesla_keys.empty());
    EXPECT_FALSE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    EXPECT_FALSE(osnma->d_tags_awaiting_verify.empty());
    EXPECT_NE(osnma->d_last_verified_key_GST, 0);
}


TEST_F(OsnmaMsgReceiverTest, ChainRenewalPromotesVerifiedFutureKrootAtApplicability)
{
    auto nominal_msg = std::make_shared<OSNMA_msg>();
    nominal_msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
    nominal_msg->hkroot[1] = 0;
    nominal_msg->WN_sf0 = 1258;
    nominal_msg->TOW_sf0 = 3600;

    DSM_KROOT_message active_kroot;
    active_kroot.nb_dk = 3;
    active_kroot.pkid = 1;
    active_kroot.cidkr = 1;
    active_kroot.kroot = {0x11, 0x22};
    active_kroot.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_message = active_kroot;

    DSM_KROOT_message pending_kroot;
    pending_kroot.nb_dk = 4;
    pending_kroot.pkid = 1;
    pending_kroot.cidkr = 2;
    pending_kroot.wn_k = 1258;
    pending_kroot.towh_k = 1;
    pending_kroot.kroot = {0x33, 0x44};
    pending_kroot.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_new_message = pending_kroot;

    osnma->d_flag_chain_renewal = true;
    osnma->d_GST_chain_renewal_start = helper.compute_gst(1258, 1000);
    osnma->d_kroot_verified = true;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(1258, 2000);
    osnma->d_tesla_key_verified = true;
    osnma->d_tesla_keys[helper.compute_gst(1258, 1100)] = {0xAA};
    osnma->d_last_verified_key_GST = helper.compute_gst(1258, 1100);
    MACK_message pending_mack;
    pending_mack.WN = 1258;
    pending_mack.TOW = 1100;
    osnma->d_macks_awaiting_MACSEQ_verification.push_back(pending_mack);
    MACK_tag_and_info pending_mti;
    pending_mti.tag_info.PRN_d = 2;
    pending_mti.tag_info.ADKD = 0;
    pending_mti.tag_info.cop = 1;
    Tag pending_tag(pending_mti, 1100, 1258, 2, 1, 0b10);
    osnma->d_tags_awaiting_verify.insert({pending_tag.TOW, pending_tag});

    osnma->process_osnma_message(nominal_msg);

    EXPECT_FALSE(osnma->d_flag_chain_renewal);
    EXPECT_EQ(osnma->d_GST_chain_renewal_start, 0);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.nb_dk, pending_kroot.nb_dk);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.pkid, pending_kroot.pkid);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.cidkr, pending_kroot.cidkr);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.kroot, pending_kroot.kroot);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_new_message.nb_dk, 0);
    EXPECT_TRUE(osnma->d_kroot_verified);
    EXPECT_FALSE(osnma->d_tesla_key_verified);
    EXPECT_TRUE(osnma->d_tesla_keys.empty());
    EXPECT_TRUE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    EXPECT_TRUE(osnma->d_tags_awaiting_verify.empty());
    EXPECT_EQ(osnma->d_last_verified_key_GST, 0);
}


TEST_F(OsnmaMsgReceiverTest, UnverifiedNominalHeaderDoesNotPromoteVerifiedKroot)
{
    auto nominal_msg = std::make_shared<OSNMA_msg>();
    nominal_msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
    nominal_msg->hkroot[1] = 0;
    nominal_msg->WN_sf0 = 1258;
    nominal_msg->TOW_sf0 = 2000;

    DSM_KROOT_message active_kroot;
    active_kroot.nb_dk = 3;
    active_kroot.pkid = 1;
    active_kroot.kroot = {0x11, 0x22};
    active_kroot.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_message = active_kroot;

    DSM_KROOT_message pending_kroot;
    pending_kroot.nb_dk = 4;
    pending_kroot.pkid = 2;
    pending_kroot.wn_k = 1258;
    pending_kroot.towh_k = 1;
    pending_kroot.kroot = {0x33, 0x44};
    pending_kroot.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_new_message = pending_kroot;

    osnma->d_flag_chain_renewal = true;
    osnma->d_GST_chain_renewal_start = helper.compute_gst(1258, 1000);
    osnma->d_kroot_verified = false;
    osnma->d_tesla_key_verified = true;
    osnma->d_tesla_keys[helper.compute_gst(1258, 1100)] = {0xAA};
    osnma->d_last_verified_key_GST = helper.compute_gst(1258, 1100);
    MACK_message pending_mack;
    pending_mack.WN = 1258;
    pending_mack.TOW = 1100;
    osnma->d_macks_awaiting_MACSEQ_verification.push_back(pending_mack);
    MACK_tag_and_info pending_mti;
    pending_mti.tag_info.PRN_d = 2;
    pending_mti.tag_info.ADKD = 0;
    pending_mti.tag_info.cop = 1;
    Tag pending_tag(pending_mti, 1100, 1258, 2, 1, 0b10);
    osnma->d_tags_awaiting_verify.insert({pending_tag.TOW, pending_tag});

    osnma->process_osnma_message(nominal_msg);

    EXPECT_TRUE(osnma->d_flag_chain_renewal);
    EXPECT_EQ(osnma->d_GST_chain_renewal_start, helper.compute_gst(1258, 1000));
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.nb_dk, active_kroot.nb_dk);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.pkid, active_kroot.pkid);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.kroot, active_kroot.kroot);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_new_message.nb_dk, pending_kroot.nb_dk);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_new_message.kroot, pending_kroot.kroot);
    EXPECT_FALSE(osnma->d_kroot_verified);
    EXPECT_TRUE(osnma->d_tesla_key_verified);
    EXPECT_FALSE(osnma->d_tesla_keys.empty());
    EXPECT_FALSE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    EXPECT_FALSE(osnma->d_tags_awaiting_verify.empty());
    EXPECT_NE(osnma->d_last_verified_key_GST, 0);
}


TEST_F(OsnmaMsgReceiverTest, MalformedKrootRejectedBeforeLengthCopies)
{
    osnma->d_public_key_verified = true;
    osnma->d_active_public_key_id = 1;
    osnma->d_active_public_key_id_valid = true;
    osnma->d_osnma_data.d_dsm_header.dsm_id = 4;
    osnma->d_crypto->set_public_key_type("ECDSA P-521");
    osnma->d_count_failed_Kroot = 0;
    const std::vector<uint8_t> previous_kroot{0xAA};
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = previous_kroot;

    std::vector<uint8_t> malformed_kroot(7 * SIZE_DSM_BLOCKS_BYTES, 0);
    malformed_kroot[0] = static_cast<uint8_t>((1 << 4) | 1);  // NB_DK=1
    malformed_kroot[2] = static_cast<uint8_t>((8 << 4) | 5);  // KS=256-bit key, TS=20-bit tag

    osnma->process_dsm_message(malformed_kroot, static_cast<uint8_t>((2 << 6) | (1 << 1)));

    EXPECT_EQ(osnma->d_count_failed_Kroot, 1);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_kroot_message.kroot, previous_kroot);
    EXPECT_FALSE(osnma->d_kroot_verified);
}


TEST_F(OsnmaMsgReceiverTest, GstDeltaSecondsIsSigned)
{
    const auto receiver_time = helper.compute_gst(1258, 1000);
    const auto sis_time = helper.compute_gst(1258, 1030);

    EXPECT_EQ(osnma->gst_delta_seconds(receiver_time, sis_time), -30);
    EXPECT_EQ(osnma->gst_delta_seconds(sis_time, receiver_time), 30);
}


TEST_F(OsnmaMsgReceiverTest, KrootApplicabilitySkipsMackBeforeGst0)
{
    auto osnma_msg = std::make_shared<OSNMA_msg>();
    osnma_msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
    osnma_msg->hkroot[1] = 0;
    osnma_msg->WN_sf0 = 1258;
    osnma_msg->TOW_sf0 = 557970;

    osnma->d_kroot_verified = true;
    osnma->d_osnma_data.d_dsm_kroot_message.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_message.wn_k = 1258;
    osnma->d_osnma_data.d_dsm_kroot_message.towh_k = 155;  // GST0 TOW=558000
    osnma->d_GST_SIS = helper.compute_gst(1258, 557970);
    osnma->d_last_verified_kroot_GST = osnma->d_GST_SIS;
    osnma->d_GST_Sf = helper.compute_gst(1258, 123450);

    osnma->process_osnma_message(osnma_msg);

    EXPECT_EQ(osnma->d_GST_0, helper.compute_gst(1258, 558000));
    EXPECT_EQ(osnma->d_GST_Sf, helper.compute_gst(1258, 123450));
}


TEST_F(OsnmaMsgReceiverTest, KrootApplicabilityAcceptsTowhkZero)
{
    auto osnma_msg = std::make_shared<OSNMA_msg>();
    osnma_msg->hkroot[0] = static_cast<uint8_t>((2 << 6) | (1 << 1));  // NMAS=OP, CPKS=Nominal
    osnma_msg->page_valid[0] = 1;                                      // NMA header only, enough for this timing check.
    osnma_msg->PRN = 2;
    osnma_msg->WN_sf0 = 1258;
    osnma_msg->TOW_sf0 = 30;

    osnma->d_kroot_verified = true;
    osnma->d_last_verified_kroot_GST = helper.compute_gst(1258, 30);
    osnma->d_osnma_data.d_dsm_kroot_message.verified = true;
    osnma->d_osnma_data.d_dsm_kroot_message.wn_k = 1258;
    osnma->d_osnma_data.d_dsm_kroot_message.towh_k = 0;  // GST0 TOW=0 is valid.
    osnma->d_GST_SIS = helper.compute_gst(1258, 30);
    osnma->d_GST_Sf = helper.compute_gst(1258, 123450);

    osnma->process_osnma_message(osnma_msg);

    EXPECT_EQ(osnma->d_GST_0, helper.compute_gst(1258, 0));
    EXPECT_EQ(osnma->d_GST_Sf, helper.compute_gst(1258, 30));
    EXPECT_TRUE(osnma->d_kroot_verified);
}


TEST_F(OsnmaMsgReceiverTest, UnsupportedKrootParametersAreRejected)
{
    DSM_KROOT_message kroot;
    kroot.hf = 0;
    kroot.mf = 0;
    kroot.ks = 4;
    kroot.ts = 9;
    kroot.maclt = 34;

    EXPECT_TRUE(osnma->kroot_parameters_are_supported(kroot));

    kroot.hf = 1;
    EXPECT_FALSE(osnma->kroot_parameters_are_supported(kroot));
    kroot.hf = 0;

    kroot.mf = 2;
    EXPECT_FALSE(osnma->kroot_parameters_are_supported(kroot));
    kroot.mf = 0;

    kroot.ts = 10;
    EXPECT_FALSE(osnma->kroot_parameters_are_supported(kroot));
    kroot.ts = 9;

    kroot.maclt = 0;
    EXPECT_FALSE(osnma->kroot_parameters_are_supported(kroot));
    kroot.maclt = 34;

    kroot.ks = 9;
    EXPECT_FALSE(osnma->kroot_parameters_are_supported(kroot));
}


TEST_F(OsnmaMsgReceiverTest, UnsupportedKrootParametersSkipMackProcessing)
{
    auto msg = std::make_shared<OSNMA_msg>();
    msg->PRN = 2;
    msg->WN_sf0 = 1258;
    msg->TOW_sf0 = 1000;

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 10;
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;
    osnma->d_osnma_data.d_dsm_kroot_message.hf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 34;
    osnma->d_kroot_verified = true;
    osnma->d_tesla_key_verified = true;
    osnma->d_GST_SIS = helper.compute_gst(1258, 1000);
    osnma->d_last_verified_kroot_GST = osnma->d_GST_SIS;
    osnma->d_GST_0 = helper.compute_gst(1258, 970);
    osnma->d_GST_Sf = helper.compute_gst(1258, 1000);
    osnma->d_osnma_data.d_mack_message.tag_and_info.resize(1);
    osnma->d_osnma_data.d_mack_message.key = {0xAA};

    osnma->read_and_process_mack_block(msg);

    EXPECT_TRUE(osnma->d_macks_awaiting_MACSEQ_verification.empty());
    EXPECT_TRUE(osnma->d_osnma_data.d_mack_message.tag_and_info.empty());
    EXPECT_TRUE(osnma->d_osnma_data.d_mack_message.key.empty());
}


TEST_F(OsnmaMsgReceiverTest, MackBlockWaitsForVerifiedKroot)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 1000;
    const uint32_t prn = 2;
    const std::vector<uint8_t> key = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F};
    auto msg = make_test_mack_message(wn, tow, prn, key);

    osnma->d_osnma_data.d_nma_header.nmas = 0b10;
    osnma->d_tags_to_verify = {12};

    osnma->read_and_process_mack_block(msg);

    ASSERT_EQ(osnma->d_mack_blocks_awaiting_kroot.size(), 1);
    EXPECT_EQ(osnma->d_mack_blocks_awaiting_kroot.front().allowed_adkds, (std::vector<uint8_t>{12}));
    EXPECT_TRUE(osnma->d_macks_awaiting_MACSEQ_verification.empty());

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9;
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;
    osnma->d_osnma_data.d_dsm_kroot_message.hf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 34;
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = std::vector<uint8_t>(16, 0);
    osnma->d_osnma_data.d_dsm_kroot_message.wn_k = wn;
    osnma->d_osnma_data.d_dsm_kroot_message.towh_k = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.verified = true;
    osnma->d_kroot_verified = true;
    osnma->d_tesla_key_verified = true;
    osnma->d_GST_SIS = helper.compute_gst(wn, tow);
    osnma->d_last_verified_kroot_GST = osnma->d_GST_SIS;
    osnma->d_tesla_keys[helper.compute_gst(wn, tow)] = key;

    osnma->process_deferred_mack_blocks();

    EXPECT_TRUE(osnma->d_mack_blocks_awaiting_kroot.empty());
    ASSERT_EQ(osnma->d_macks_awaiting_MACSEQ_verification.size(), 1);
    EXPECT_EQ(osnma->d_macks_awaiting_MACSEQ_verification.front().allowed_adkds, (std::vector<uint8_t>{12}));
    EXPECT_EQ(osnma->d_macks_awaiting_MACSEQ_verification.front().nmas, 0b10);
    EXPECT_EQ(osnma->d_macks_awaiting_MACSEQ_verification.front().key, key);
}


TEST_F(OsnmaMsgReceiverTest, FailedDsmPkrKeepsCurrentPublicKey)
{
    osnma->d_public_key_verified = true;
    osnma->d_count_failed_pubKey = 0;
    osnma->d_osnma_data.d_dsm_header.dsm_id = 12;
    osnma->set_merkle_root(std::vector<uint8_t>(32, 0xFF));

    std::vector<uint8_t> dsm_pkr(169, 0);
    dsm_pkr[0] = 0x71;    // NB_DP=7, MID=1
    dsm_pkr[129] = 0x12;  // NPKT=ECDSA P-256, NPKID=2

    osnma->process_dsm_message(dsm_pkr, static_cast<uint8_t>((2 << 6) | (1 << 1)));

    EXPECT_TRUE(osnma->d_public_key_verified);
    EXPECT_EQ(osnma->d_count_failed_pubKey, 1);
}


TEST_F(OsnmaMsgReceiverTest, LowerPkidDsmPkrIsRejectedWhilePublicKeyInForce)
{
    const std::vector<uint8_t> merkle_root = helper.convert_from_hex_string("A10C440F3AA62453526DB4AF76DF8D9410D35D8277397D7053C700D192702B0D");
    osnma->set_merkle_root(merkle_root);
    osnma->d_public_key_verified = true;
    osnma->d_active_public_key_id = 3;
    osnma->d_active_public_key_id_valid = true;
    osnma->d_count_failed_pubKey = 0;
    osnma->d_osnma_data.d_dsm_header.dsm_id = 12;

    osnma->process_dsm_message(make_valid_dsm_pkr_p256(helper), static_cast<uint8_t>((2 << 6) | (1 << 1)));

    EXPECT_TRUE(osnma->d_public_key_verified);
    EXPECT_FALSE(osnma->d_flag_NPK_set);
    EXPECT_EQ(osnma->d_new_public_key_id, 0);
    EXPECT_TRUE(osnma->d_new_public_key.empty());
    EXPECT_FALSE(osnma->d_material_manager->candidate_public_key(2).valid);
    EXPECT_EQ(osnma->d_count_failed_pubKey, 1);
}


TEST_F(OsnmaMsgReceiverTest, VerifiedDsmPkrSetsActiveKeyIdOnColdStart)
{
    const std::vector<uint8_t> merkle_root = helper.convert_from_hex_string("A10C440F3AA62453526DB4AF76DF8D9410D35D8277397D7053C700D192702B0D");
    osnma->set_merkle_root(merkle_root);
    osnma->d_public_key_verified = false;
    osnma->d_kroot_verified = false;
    osnma->d_active_public_key_id_valid = false;
    osnma->d_osnma_data.d_dsm_header.dsm_id = 12;

    osnma->process_dsm_message(make_valid_dsm_pkr_p256(helper), static_cast<uint8_t>((2 << 6) | (1 << 1)));

    EXPECT_TRUE(osnma->d_public_key_verified);
    EXPECT_TRUE(osnma->d_active_public_key_id_valid);
    EXPECT_EQ(osnma->d_active_public_key_id, 2);
}


TEST_F(OsnmaMsgReceiverTest, VerifiedDsmPkrUpdatesPendingKeyIdAcrossRollover)
{
    const std::vector<uint8_t> merkle_root = helper.convert_from_hex_string("A10C440F3AA62453526DB4AF76DF8D9410D35D8277397D7053C700D192702B0D");
    const std::vector<uint8_t> expected_public_key = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");
    osnma->d_public_key_verified = true;
    osnma->d_kroot_verified = true;
    osnma->d_active_public_key_id = 9;
    osnma->d_active_public_key_id_valid = true;
    osnma->d_flag_merkle_tree_renewal = true;
    osnma->d_merkle_root_at_renewal_start = std::vector<uint8_t>(32, 0x00);
    osnma->d_merkle_hash_function_at_renewal_start = osnma->d_crypto->get_merkle_tree_hash_function();
    osnma->set_merkle_root(merkle_root);
    osnma->d_new_public_key_id = 9;
    osnma->d_new_public_key = {0xAA};
    osnma->d_osnma_data.d_dsm_header.dsm_id = 12;

    osnma->process_dsm_message(make_valid_dsm_pkr_p256(helper), static_cast<uint8_t>((2 << 6) | (6 << 1)));

    EXPECT_TRUE(osnma->d_public_key_verified);
    EXPECT_TRUE(osnma->d_flag_NPK_set);
    EXPECT_EQ(osnma->d_new_public_key_id, 2);
    EXPECT_EQ(osnma->d_new_public_key, expected_public_key);
}


TEST_F(OsnmaMsgReceiverTest, NmtDsmPkrWaitsForNewMerkleTree)
{
    const std::vector<uint8_t> merkle_root = helper.convert_from_hex_string("A10C440F3AA62453526DB4AF76DF8D9410D35D8277397D7053C700D192702B0D");
    osnma->set_merkle_root(merkle_root);
    osnma->d_flag_merkle_tree_renewal = true;
    osnma->d_merkle_root_at_renewal_start = merkle_root;
    osnma->d_merkle_hash_function_at_renewal_start = osnma->d_crypto->get_merkle_tree_hash_function();
    osnma->d_new_merkle_tree_loaded = false;
    osnma->d_merkle_file_path.clear();
    osnma->d_public_key_verified = true;
    osnma->d_kroot_verified = true;
    osnma->d_flag_NPK_set = false;
    osnma->d_new_public_key_id = 9;
    osnma->d_new_public_key = {0xAA};
    osnma->d_osnma_data.d_dsm_header.dsm_id = 12;

    osnma->process_dsm_message(make_valid_dsm_pkr_p256(helper), static_cast<uint8_t>((2 << 6) | (6 << 1)));

    EXPECT_FALSE(osnma->d_new_merkle_tree_loaded);
    EXPECT_FALSE(osnma->d_flag_NPK_set);
    EXPECT_EQ(osnma->d_new_public_key_id, 9);
    EXPECT_EQ(osnma->d_new_public_key, (std::vector<uint8_t>{0xAA}));
}


TEST_F(OsnmaMsgReceiverTest, NmtDsmPkrUsesCandidateMerkleTree)
{
    const std::vector<uint8_t> active_merkle_root(32, 0xFF);
    const std::vector<uint8_t> candidate_merkle_root = helper.convert_from_hex_string("A10C440F3AA62453526DB4AF76DF8D9410D35D8277397D7053C700D192702B0D");
    const std::vector<uint8_t> expected_public_key = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");
    osnma->set_merkle_root(active_merkle_root);
    osnma->d_public_key_verified = true;
    osnma->d_kroot_verified = true;
    osnma->d_flag_merkle_tree_renewal = true;
    osnma->d_merkle_root_at_renewal_start = active_merkle_root;
    osnma->d_merkle_hash_function_at_renewal_start = osnma->d_crypto->get_merkle_tree_hash_function();
    osnma->set_merkle_root(candidate_merkle_root);
    osnma->d_new_public_key_id = 9;
    osnma->d_new_public_key = {0xAA};
    osnma->d_osnma_data.d_dsm_header.dsm_id = 12;

    osnma->process_dsm_message(make_valid_dsm_pkr_p256(helper), static_cast<uint8_t>((2 << 6) | (6 << 1)));

    EXPECT_TRUE(osnma->d_new_merkle_tree_loaded);
    EXPECT_EQ(osnma->d_crypto->get_merkle_root(), active_merkle_root);
    EXPECT_TRUE(osnma->d_flag_NPK_set);
    EXPECT_EQ(osnma->d_new_public_key_id, 2);
    EXPECT_EQ(osnma->d_new_public_key, expected_public_key);
    EXPECT_TRUE(osnma->d_material_manager->candidate_public_key(2).valid);
}


TEST_F(OsnmaMsgReceiverTest, MerkleXmlDoesNotChangeActivePublicKeyType)
{
    const std::string merkle_path = "./osnma_test_merkle_p521.xml";
    remove_file_if_present(merkle_path);
    {
        std::ofstream file(merkle_path, std::ios::binary | std::ios::trunc);
        file << "<signalData><header><GAL-header><issueDate>2026-01-01T00:00:00Z</issueDate>"
                "<signalVersion>1</signalVersion><dataVersion>1</dataVersion></GAL-header></header>"
                "<body><MerkleTree><UID>test</UID><Applicability>2026-01-01T00:00:00Z</Applicability>"
                "<State>Applicable</State><N>1</N><HashFunction>SHA-256</HashFunction>"
                "<PublicKey><i>0</i><PKID>3</PKID><lengthInBits>536</lengthInBits>"
                "<point>00</point><PKType>ECDSA P-521/SHA-512</PKType></PublicKey>"
                "<TreeNode><j>4</j><i>0</i><lengthInBits>256</lengthInBits>"
                "<x_ji>A10C440F3AA62453526DB4AF76DF8D9410D35D8277397D7053C700D192702B0D</x_ji>"
                "</TreeNode></MerkleTree></body></signalData>";
    }

    osnma->d_crypto->set_public_key(helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA"));
    ASSERT_EQ(osnma->d_crypto->get_public_key_type(), "ECDSA P-256");

    osnma->read_merkle_xml(merkle_path);

    EXPECT_EQ(osnma->d_crypto->get_public_key_type(), "ECDSA P-256");
    remove_file_if_present(merkle_path);
}


TEST_F(OsnmaMsgReceiverTest, PublicKeyMetadataRestoresPkidForPemHotStart)
{
    remove_file_if_present(PEMFILE_DEFAULT);
    remove_file_if_present(PEMFILE_DEFAULT + ".meta");

    const std::vector<uint8_t> public_key = helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA");
    Gnss_Crypto crypto;
    crypto.set_public_key(public_key);
    ASSERT_TRUE(crypto.store_public_key(PEMFILE_DEFAULT));
    const auto fingerprint = canonical_public_key_fingerprint(2, 1, public_key);
    ASSERT_FALSE(fingerprint.empty());
    {
        std::ofstream metadata(PEMFILE_DEFAULT + ".meta", std::ios::binary | std::ios::trunc);
        metadata << "version=1\n"
                 << "pkid=2\n"
                 << "npkt=1\n"
                 << "pk_type=ECDSA P-256\n"
                 << "source=DSM-PKR\n"
                 << "public_key_sha256=" << fingerprint << "\n";
    }

    auto hot_start_osnma = osnma_msg_receiver_make("./missing-osnma-public-key.crt", MERKLEFILE_DEFAULT);

    EXPECT_TRUE(hot_start_osnma->d_active_public_key_id_valid);
    EXPECT_EQ(hot_start_osnma->d_active_public_key_id, 2);
    EXPECT_EQ(hot_start_osnma->d_material_manager->active_public_key().key_type, "ECDSA P-256");
    EXPECT_EQ(hot_start_osnma->d_material_manager->active_public_key().fingerprint_sha256, fingerprint);
}


TEST_F(OsnmaMsgReceiverTest, PublicKeyMetadataRejectsMismatchedPemFingerprint)
{
    const std::string pem_path = "./OSNMA_PublicKey_plain_mismatch.pem";
    remove_file_if_present(pem_path);
    remove_file_if_present(pem_path + ".meta");

    Gnss_Crypto crypto;
    crypto.set_public_key(helper.convert_from_hex_string("0303B2CE64BC207BDD8BC4DF859187FCB686320D63FFA091410FC158FBB77980EA"));
    ASSERT_TRUE(crypto.store_public_key(pem_path));
    {
        std::ofstream metadata(pem_path + ".meta", std::ios::binary | std::ios::trunc);
        metadata << "version=1\n"
                 << "pkid=2\n"
                 << "npkt=1\n"
                 << "pk_type=ECDSA P-256\n"
                 << "source=DSM-PKR\n"
                 << "public_key_sha256=00\n";
    }

    auto hot_start_osnma = osnma_msg_receiver_make(pem_path, MERKLEFILE_DEFAULT);

    EXPECT_FALSE(hot_start_osnma->d_active_public_key_id_valid);
    remove_file_if_present(pem_path);
    remove_file_if_present(pem_path + ".meta");
}


TEST_F(OsnmaMsgReceiverTest, MalformedDsmPkrRejectedBeforeLengthCopies)
{
    osnma->d_public_key_verified = true;
    osnma->d_osnma_data.d_dsm_header.dsm_id = 12;
    osnma->d_osnma_data.d_dsm_pkr_message.npktid = 7;
    osnma->d_osnma_data.d_dsm_pkr_message.npk = {0xAA};

    std::vector<uint8_t> malformed_pkr(129, 0);
    osnma->process_dsm_message(malformed_pkr, static_cast<uint8_t>((2 << 6) | (1 << 1)));

    EXPECT_EQ(osnma->d_osnma_data.d_dsm_pkr_message.npktid, 7);
    EXPECT_EQ(osnma->d_osnma_data.d_dsm_pkr_message.npk, (std::vector<uint8_t>{0xAA}));
    EXPECT_TRUE(osnma->d_public_key_verified);
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

    osnma->d_tesla_keys[helper.compute_gst(1248, 345600)] =
        {0xEF, 0xF9, 0x99, 0x04, 0x0E, 0x19, 0xB5, 0x70, 0x83, 0x50, 0x60, 0xBE, 0xBD, 0x23, 0xED, 0x92};                         // K1, not needed, just for reference.
    std::vector<uint8_t> key = {0x2D, 0xC3, 0xA3, 0xCD, 0xB1, 0x17, 0xFA, 0xAD, 0xB8, 0x3B, 0x5F, 0x0B, 0x6F, 0xEA, 0x88, 0xEB};  // K2
    uint32_t TOW = 345630;

    // Act
    bool result = osnma->verify_tesla_key(key, helper.compute_gst(1248, TOW));  // TODO - refactor so that output is not a boolean. Or use last_verified_tesla_key?

    // Assert
    ASSERT_TRUE(result);
}


TEST_F(OsnmaMsgReceiverTest, FailedTeslaKeyDoesNotReturnPersistentSuccess)
{
    osnma->d_tesla_key_verified = false;
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = {0x5B, 0xF8, 0xC9, 0xCB, 0xFC, 0xF7, 0x04, 0x22, 0x08, 0x14, 0x75, 0xFD, 0x44, 0x5D, 0xF0, 0xFF};
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;
    osnma->d_osnma_data.d_dsm_kroot_message.alpha = 0x610BDF26D77B;
    osnma->d_GST_0 = ((1248 & 0x00000FFF) << 20 | (345600 & 0x000FFFFF));
    osnma->d_GST_Sf = ((1248 & 0x00000FFF) << 20 | (345630 & 0x000FFFFF));

    std::vector<uint8_t> key = {0x2D, 0xC3, 0xA3, 0xCD, 0xB1, 0x17, 0xFA, 0xAD, 0xB8, 0x3B, 0x5F, 0x0B, 0x6F, 0xEA, 0x88, 0xEB};
    ASSERT_TRUE(osnma->verify_tesla_key(key, helper.compute_gst(1248, 345630)));
    ASSERT_TRUE(osnma->d_tesla_key_verified);

    osnma->d_GST_Sf = ((1248 & 0x00000FFF) << 20 | (345660 & 0x000FFFFF));
    std::vector<uint8_t> bad_key(key.size(), 0x00);

    EXPECT_FALSE(osnma->verify_tesla_key(bad_key, helper.compute_gst(1248, 345660)));
    EXPECT_TRUE(osnma->d_tesla_key_verified);
    EXPECT_EQ(osnma->d_tesla_keys.find(helper.compute_gst(1248, 345660)), osnma->d_tesla_keys.end());
}


TEST_F(OsnmaMsgReceiverTest, OutOfOrderTeslaKeyRejected)
{
    osnma->d_tesla_key_verified = false;
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = {0x5B, 0xF8, 0xC9, 0xCB, 0xFC, 0xF7, 0x04, 0x22, 0x08, 0x14, 0x75, 0xFD, 0x44, 0x5D, 0xF0, 0xFF};
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;
    osnma->d_osnma_data.d_dsm_kroot_message.alpha = 0x610BDF26D77B;
    osnma->d_GST_0 = helper.compute_gst(1248, 345600);
    osnma->d_GST_Sf = helper.compute_gst(1248, 345630);

    std::vector<uint8_t> key = {0x2D, 0xC3, 0xA3, 0xCD, 0xB1, 0x17, 0xFA, 0xAD, 0xB8, 0x3B, 0x5F, 0x0B, 0x6F, 0xEA, 0x88, 0xEB};
    ASSERT_TRUE(osnma->verify_tesla_key(key, helper.compute_gst(1248, 345630)));
    const auto last_verified_key_gst = osnma->d_last_verified_key_GST;
    const auto verified_key_count = osnma->d_tesla_keys.size();

    osnma->d_GST_Sf = helper.compute_gst(1248, 345600);

    EXPECT_FALSE(osnma->verify_tesla_key(key, helper.compute_gst(1248, 345600)));
    EXPECT_EQ(osnma->d_last_verified_key_GST, last_verified_key_gst);
    EXPECT_EQ(osnma->d_tesla_keys.size(), verified_key_count);
}


TEST_F(OsnmaMsgReceiverTest, TeslaKeyVerificationAcrossWeekRollover)
{
    osnma->d_tesla_key_verified = false;
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4;
    osnma->d_osnma_data.d_dsm_kroot_message.hf = 0;
    osnma->d_osnma_data.d_dsm_kroot_message.alpha = 0x610BDF26D77B;
    osnma->d_GST_0 = helper.compute_gst(1248, 604770);
    osnma->d_GST_Sf = helper.compute_gst(1249, 30);

    std::vector<uint8_t> key = {0x2D, 0xC3, 0xA3, 0xCD, 0xB1, 0x17, 0xFA, 0xAD, 0xB8, 0x3B, 0x5F, 0x0B, 0x6F, 0xEA, 0x88, 0xEB};
    auto gst_with_offset = [this](uint32_t gst, int32_t offset_seconds) {
        constexpr int64_t seconds_per_week = 604800;
        int64_t absolute_seconds = static_cast<int64_t>(helper.get_WN(gst)) * seconds_per_week + helper.get_TOW(gst) + offset_seconds;
        if (absolute_seconds < 0)
            {
                absolute_seconds = 0;
            }
        return helper.compute_gst(static_cast<uint32_t>(absolute_seconds / seconds_per_week), static_cast<uint32_t>(absolute_seconds % seconds_per_week));
    };
    auto hash_step = [this](const std::vector<uint8_t>& input_key, uint32_t gst) {
        std::vector<uint8_t> msg;
        msg.insert(msg.end(), input_key.cbegin(), input_key.cend());
        const auto gst_bytes = helper.gst_to_uint8(gst);
        msg.insert(msg.end(), gst_bytes.cbegin(), gst_bytes.cend());
        for (int k = 5; k >= 0; k--)
            {
                msg.push_back(static_cast<uint8_t>((osnma->d_osnma_data.d_dsm_kroot_message.alpha >> (k * 8)) & 0xFF));
            }
        auto hash = osnma->d_crypto->compute_SHA_256(msg);
        hash.resize(input_key.size());
        return hash;
    };

    std::vector<uint8_t> expected_kroot = key;
    uint32_t hash_gst = helper.compute_gst(1249, 0);
    for (int i = 0; i < 3; i++)
        {
            expected_kroot = hash_step(expected_kroot, hash_gst);
            hash_gst = gst_with_offset(hash_gst, -30);
        }
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = expected_kroot;

    EXPECT_TRUE(osnma->verify_tesla_key(key, osnma->d_GST_Sf));
    EXPECT_EQ(osnma->d_tesla_keys[osnma->d_GST_Sf], key);
}


TEST_F(OsnmaMsgReceiverTest, PendingMackQueueExpiresOldEntries)
{
    osnma->d_kroot_verified = true;
    osnma->d_tesla_key_verified = false;
    osnma->d_GST_Sf = helper.compute_gst(1258, 1400);

    MACK_message stale_mack;
    stale_mack.WN = 1258;
    stale_mack.TOW = 1000;
    osnma->d_macks_awaiting_MACSEQ_verification.push_back(stale_mack);

    osnma->d_osnma_data.d_mack_message.WN = 1258;
    osnma->d_osnma_data.d_mack_message.TOW = 1400;
    osnma->d_osnma_data.d_mack_message.PRNa = 2;

    osnma->process_mack_message();

    ASSERT_EQ(osnma->d_macks_awaiting_MACSEQ_verification.size(), 1);
    EXPECT_EQ(osnma->d_macks_awaiting_MACSEQ_verification.front().TOW, 1400);
}


TEST_F(OsnmaMsgReceiverTest, FastTagsWaitForNavDataUntilVerificationWindowExpires)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 1000;
    const uint32_t prna = 2;

    MACK_tag_and_info adkd0_mti;
    adkd0_mti.tag_info.PRN_d = 2;
    adkd0_mti.tag_info.ADKD = 0;
    adkd0_mti.tag_info.cop = 1;
    Tag adkd0_tag(adkd0_mti, tow, wn, prna, 1, 0b10);
    osnma->d_tags_awaiting_verify.insert({adkd0_tag.TOW, adkd0_tag});

    MACK_tag_and_info adkd4_mti;
    adkd4_mti.tag_info.PRN_d = 2;
    adkd4_mti.tag_info.ADKD = 4;
    adkd4_mti.tag_info.cop = 1;
    Tag adkd4_tag(adkd4_mti, tow, wn, prna, 2, 0b10);
    osnma->d_tags_awaiting_verify.insert({adkd4_tag.TOW, adkd4_tag});

    osnma->d_last_verified_key_GST = helper.compute_gst(wn, tow + 30);
    osnma->remove_verified_tags();

    EXPECT_EQ(osnma->d_tags_awaiting_verify.size(), 2);

    osnma->d_last_verified_key_GST = helper.compute_gst(wn, tow + 331);
    osnma->remove_verified_tags();

    EXPECT_TRUE(osnma->d_tags_awaiting_verify.empty());
}


TEST_F(OsnmaMsgReceiverTest, MacseqVerificationUsesMackTime)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 120000;
    const uint32_t prna = 2;
    const std::vector<uint8_t> key = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6,
        0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDB, 0xBC, 0x73};

    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 34;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_tesla_keys[helper.compute_gst(wn, tow + 30)] = key;
    osnma->d_GST_Sf = helper.compute_gst(wn, tow + 90);

    MACK_message mack;
    mack.WN = wn;
    mack.TOW = tow;
    mack.PRNa = prna;
    mack.nmas = 0b10;
    mack.tag_and_info.resize(5);

    auto fill_tag = [](MACK_tag_and_info& tag, uint8_t prn_d, uint8_t adkd, uint8_t cop, uint32_t counter) {
        tag.tag = static_cast<uint64_t>(counter);
        tag.counter = counter;
        tag.tag_info.PRN_d = prn_d;
        tag.tag_info.ADKD = adkd;
        tag.tag_info.cop = cop;
    };
    fill_tag(mack.tag_and_info[0], 4, 0, 1, 2);   // FLX
    fill_tag(mack.tag_and_info[1], 2, 4, 1, 3);   // 04S
    fill_tag(mack.tag_and_info[2], 5, 12, 2, 4);  // FLX
    fill_tag(mack.tag_and_info[3], 2, 12, 1, 5);  // 12S
    fill_tag(mack.tag_and_info[4], 6, 0, 1, 6);   // 00E

    std::vector<uint8_t> macseq_message;
    macseq_message.push_back(static_cast<uint8_t>(prna));
    const auto gst_bytes = helper.gst_to_uint8(helper.compute_gst(wn, tow));
    macseq_message.insert(macseq_message.end(), gst_bytes.cbegin(), gst_bytes.cend());
    macseq_message.push_back(mack.tag_and_info[0].tag_info.PRN_d);
    macseq_message.push_back((mack.tag_and_info[0].tag_info.ADKD << 4) | mack.tag_and_info[0].tag_info.cop);
    macseq_message.push_back(mack.tag_and_info[2].tag_info.PRN_d);
    macseq_message.push_back((mack.tag_and_info[2].tag_info.ADKD << 4) | mack.tag_and_info[2].tag_info.cop);

    const auto mac = osnma->d_crypto->compute_HMAC_SHA_256(key, macseq_message);
    const uint16_t mac_msb = static_cast<uint16_t>((mac[0] << 8) + mac[1]);
    mack.header.macseq = (mac_msb & 0xFFF0) >> 4;
    mack.header.macseq_valid = true;

    const auto verified_tags = osnma->verify_macseq(mack);

    EXPECT_EQ(verified_tags.size(), mack.tag_and_info.size());
}


TEST_F(OsnmaMsgReceiverTest, MacseqVerificationMissingMacseqKeepsFixedTagsOnly)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 120000;
    const uint32_t prna = 2;

    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 34;

    MACK_message mack;
    mack.WN = wn;
    mack.TOW = tow;
    mack.PRNa = prna;
    mack.nmas = 0b10;
    mack.header.macseq_valid = false;
    mack.tag_and_info.resize(5);

    auto fill_tag = [](MACK_tag_and_info& tag, uint8_t prn_d, uint8_t adkd, uint8_t cop, uint32_t counter) {
        tag.tag = static_cast<uint64_t>(counter);
        tag.counter = counter;
        tag.tag_info.PRN_d = prn_d;
        tag.tag_info.ADKD = adkd;
        tag.tag_info.cop = cop;
    };
    fill_tag(mack.tag_and_info[0], 4, 0, 1, 2);   // FLX
    fill_tag(mack.tag_and_info[1], 2, 4, 1, 3);   // 04S
    fill_tag(mack.tag_and_info[2], 5, 12, 2, 4);  // FLX
    fill_tag(mack.tag_and_info[3], 2, 12, 1, 5);  // 12S
    fill_tag(mack.tag_and_info[4], 6, 0, 1, 6);   // 00E

    const auto verified_tags = osnma->verify_macseq(mack);
    auto contains_counter = [&verified_tags](uint32_t counter) {
        for (const auto& tag : verified_tags)
            {
                if (tag.counter == counter)
                    {
                        return true;
                    }
            }
        return false;
    };

    EXPECT_EQ(verified_tags.size(), 3);
    EXPECT_FALSE(contains_counter(2));
    EXPECT_TRUE(contains_counter(3));
    EXPECT_FALSE(contains_counter(4));
    EXPECT_TRUE(contains_counter(5));
    EXPECT_TRUE(contains_counter(6));
}


TEST_F(OsnmaMsgReceiverTest, MacseqVerificationDiscardsReservedFlexibleTags)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 120030;
    const uint32_t prna = 2;
    const std::vector<uint8_t> key = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6,
        0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDB, 0xBC, 0x73};

    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 35;
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_tesla_keys[helper.compute_gst(wn, tow + 30)] = key;

    MACK_message mack;
    mack.WN = wn;
    mack.TOW = tow;
    mack.PRNa = prna;
    mack.nmas = 0b10;
    mack.tag_and_info.resize(5);

    auto fill_tag = [](MACK_tag_and_info& tag, uint8_t prn_d, uint8_t adkd, uint8_t cop, uint32_t counter) {
        tag.tag = static_cast<uint64_t>(counter);
        tag.counter = counter;
        tag.tag_info.PRN_d = prn_d;
        tag.tag_info.ADKD = adkd;
        tag.tag_info.cop = cop;
    };
    fill_tag(mack.tag_and_info[0], 4, 0, 1, 2);   // FLX, valid.
    fill_tag(mack.tag_and_info[1], 0, 4, 1, 3);   // FLX, reserved PRN_d.
    fill_tag(mack.tag_and_info[2], 2, 12, 1, 4);  // 12S, valid fixed slot.
    fill_tag(mack.tag_and_info[3], 37, 0, 1, 5);  // FLX, reserved PRN_d.
    fill_tag(mack.tag_and_info[4], 5, 15, 1, 6);  // FLX, reserved ADKD.

    std::vector<uint8_t> macseq_message;
    macseq_message.push_back(static_cast<uint8_t>(prna));
    const auto gst_bytes = helper.gst_to_uint8(helper.compute_gst(wn, tow));
    macseq_message.insert(macseq_message.end(), gst_bytes.cbegin(), gst_bytes.cend());
    const std::vector<uint8_t> flx_tag_indices = {0, 1, 3, 4};
    for (uint8_t flx_tag_index : flx_tag_indices)
        {
            macseq_message.push_back(mack.tag_and_info[flx_tag_index].tag_info.PRN_d);
            macseq_message.push_back((mack.tag_and_info[flx_tag_index].tag_info.ADKD << 4) |
                                     mack.tag_and_info[flx_tag_index].tag_info.cop);
        }

    const auto mac = osnma->d_crypto->compute_HMAC_SHA_256(key, macseq_message);
    const uint16_t mac_msb = static_cast<uint16_t>((mac[0] << 8) + mac[1]);
    mack.header.macseq = (mac_msb & 0xFFF0) >> 4;
    mack.header.macseq_valid = true;

    const auto verified_tags = osnma->verify_macseq(mack);
    auto contains_counter = [&verified_tags](uint32_t counter) {
        for (const auto& tag : verified_tags)
            {
                if (tag.counter == counter)
                    {
                        return true;
                    }
            }
        return false;
    };

    EXPECT_EQ(verified_tags.size(), 2);
    EXPECT_TRUE(contains_counter(2));
    EXPECT_TRUE(contains_counter(4));
    EXPECT_FALSE(contains_counter(3));
    EXPECT_FALSE(contains_counter(5));
    EXPECT_FALSE(contains_counter(6));
}


TEST_F(OsnmaMsgReceiverTest, MacseqVerificationChecksFixedSelfAndCrossSlots)
{
    const uint32_t wn = 1258;
    const uint32_t tow = 120000;
    const uint32_t prna = 2;

    osnma->d_osnma_data.d_dsm_kroot_message.maclt = 40;

    MACK_message mack;
    mack.WN = wn;
    mack.TOW = tow;
    mack.PRNa = prna;
    mack.tag_and_info.resize(3);

    auto fill_tag = [](MACK_tag_and_info& tag, uint8_t prn_d, uint8_t adkd, uint32_t counter) {
        tag.tag = static_cast<uint64_t>(counter);
        tag.counter = counter;
        tag.tag_info.PRN_d = prn_d;
        tag.tag_info.ADKD = adkd;
    };
    fill_tag(mack.tag_and_info[0], 3, 0, 2);   // 00E
    fill_tag(mack.tag_and_info[1], 2, 4, 3);   // 04S
    fill_tag(mack.tag_and_info[2], 2, 12, 4);  // 12S

    EXPECT_EQ(osnma->verify_macseq(mack).size(), 3);

    mack.tag_and_info[0].tag_info.PRN_d = 2;  // 00E cannot self-authenticate.
    EXPECT_EQ(osnma->verify_macseq(mack).size(), 2);

    mack.tag_and_info[0].tag_info.PRN_d = 3;
    mack.tag_and_info[1].tag_info.PRN_d = 4;  // 04S must self-authenticate.
    EXPECT_EQ(osnma->verify_macseq(mack).size(), 2);
}


TEST_F(OsnmaMsgReceiverTest, TeslaKeyLookupUsesWeek)
{
    const uint32_t tow = 604770;
    const uint32_t next_week_tow = 0;
    const uint32_t prna = 2;
    MACK_tag_and_info mti;
    mti.tag_info.PRN_d = 2;
    mti.tag_info.ADKD = 0;
    mti.tag_info.cop = 1;
    Tag tag(mti, tow, 1258, prna, 1, 0b10);

    osnma->d_tesla_keys[helper.compute_gst(1258, next_week_tow)] = {0xAA};
    EXPECT_FALSE(osnma->tag_has_key_available(tag));

    osnma->d_tesla_keys[helper.compute_gst(1259, next_week_tow)] = {0xBB};
    EXPECT_TRUE(osnma->tag_has_key_available(tag));
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
