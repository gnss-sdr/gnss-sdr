/*!
 * \file beidou_b2a_signal_replica_test.cc
 * \brief Unit tests for BeiDou B2a ranging-code generation vs ICD Tables 5-2 / 5-3
 * \author huangchuhan, 2026. huangchh37(at)mail2.sysu.edu.cn
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
#include "Beidou_B2a.h"
#include "beidou_b2a_signal_replica.h"
#include <gtest/gtest.h>
#include <array>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace
{
// Named distinctly from other replica tests included via test_main.cc.
std::string chips_to_octal24_b2a(const int* chips, int start)
{
    uint32_t v = 0;
    for (int i = 0; i < 24; i++)
        {
            v = (v << 1) | static_cast<uint32_t>(chips[start + i] > 0 ? 1 : 0);
        }
    char buf[9];
    std::snprintf(buf, sizeof(buf), "%08o", v);
    return {buf};
}
std::string chips_to_octal24_b2a(const std::string& chips, int start)
{
    uint32_t v = 0;
    for (int i = 0; i < 24; i++)
        {
            v = (v << 1) | static_cast<uint32_t>(chips[start + i] == '1' ? 1 : 0);
        }
    char buf[9];
    std::snprintf(buf, sizeof(buf), "%08o", v);
    return {buf};
}
}  // namespace


TEST(BeidouB2aSignalReplicaTest, DataCodeLength)
{
    std::array<int, 10230> chips{};
    beidou_b2a_data_code_gen_int(chips, 19, 0);
    EXPECT_EQ(chips.size(), static_cast<size_t>(BEIDOU_B2A_CODE_LENGTH_CHIPS));
}


TEST(BeidouB2aSignalReplicaTest, DataPrn1FirstLast24MatchIcd)
{
    std::array<int, 10230> chips{};
    beidou_b2a_data_code_gen_int(chips, 1, 0);
    EXPECT_EQ(chips_to_octal24_b2a(chips.data(), 0), "26771056");
    EXPECT_EQ(chips_to_octal24_b2a(chips.data(), 10206), "42646672");
}


TEST(BeidouB2aSignalReplicaTest, DataPrn6FirstLast24MatchIcd)
{
    std::array<int, 10230> chips{};
    beidou_b2a_data_code_gen_int(chips, 6, 0);
    EXPECT_EQ(chips_to_octal24_b2a(chips.data(), 0), "42473731");
    EXPECT_EQ(chips_to_octal24_b2a(chips.data(), 10206), "32223757");
}


TEST(BeidouB2aSignalReplicaTest, DataPrn19FirstLast24MatchIcd)
{
    std::array<int, 10230> chips{};
    beidou_b2a_data_code_gen_int(chips, 19, 0);
    EXPECT_EQ(chips_to_octal24_b2a(chips.data(), 0), "40653671");
    EXPECT_EQ(chips_to_octal24_b2a(chips.data(), 10206), "01726764");
}


TEST(BeidouB2aSignalReplicaTest, PilotPrn1FirstLast24MatchIcd)
{
    std::array<int, 10230> chips{};
    beidou_b2a_pilot_code_gen_int(chips, 1, 0);
    EXPECT_EQ(chips_to_octal24_b2a(chips.data(), 0), "26772435");
    EXPECT_EQ(chips_to_octal24_b2a(chips.data(), 10206), "05133452");
}


TEST(BeidouB2aSignalReplicaTest, PilotPrn19FirstLast24MatchIcd)
{
    std::array<int, 10230> chips{};
    beidou_b2a_pilot_code_gen_int(chips, 19, 0);
    EXPECT_EQ(chips_to_octal24_b2a(chips.data(), 0), "40650022");
    EXPECT_EQ(chips_to_octal24_b2a(chips.data(), 10206), "62054544");
}


TEST(BeidouB2aSignalReplicaTest, SampledCodeLengthAt20Msps)
{
    constexpr int32_t fs = 20000000;
    const auto samples_per_code = static_cast<int32_t>(static_cast<double>(fs) /
                                                       (BEIDOU_B2A_CODE_RATE_CPS / BEIDOU_B2A_CODE_LENGTH_CHIPS));
    std::vector<std::complex<float>> sampled(static_cast<size_t>(samples_per_code));
    beidou_b2a_data_code_gen_complex_sampled(sampled, 19, fs, 0);
    EXPECT_EQ(static_cast<int32_t>(sampled.size()), samples_per_code);
    EXPECT_FLOAT_EQ(sampled.front().real(), 1.0F);  // PRN 19 first chip is 1 (octal 4...)
}

TEST(BeidouB2aSignalReplicaTest, PilotSecondaryTest)
{
    struct TEST_DATA_STRUCT
    {
        int32_t prn;
        std::string first24;
        std::string last24;
    };
    static const std::vector<TEST_DATA_STRUCT> data({{1, "32063050", "65322167"},
        {2, "51032554", "16507723"},
        {3, "26031355", "03244075"},
        {4, "00016672", "51467525"},
        {5, "43414712", "46604555"},
        {6, "03313653", "56202042"},
        {7, "41103653", "71007450"},
        {8, "42370454", "34256747"},
        {9, "06231051", "40430077"},
        {10, "37047570", "06442617"},
        {11, "36242432", "16314440"},
        {12, "62600563", "05321123"},
        {13, "77411542", "56573352"},
        {14, "41654772", "55730776"},
        {15, "63255352", "01324146"},
        {16, "16034451", "17500531"},
        {17, "56753432", "66634453"},
        {18, "62660722", "37240150"},
        {19, "11300714", "32673101"},
        {20, "46564670", "76643076"},
        {21, "51453710", "41236437"},
        {22, "75520773", "47126073"},
        {23, "55105576", "24605443"},
        {24, "31050323", "07347067"},
        {25, "76030274", "41470462"},
        {26, "61576715", "07552423"},
        {27, "21353627", "15306360"},
        {28, "11326621", "43507041"},
        {29, "77304426", "12537651"},
        {30, "26565352", "32362347"},
        {31, "34135261", "14550406"},
        {32, "30407566", "60014143"},
        {33, "52113374", "61116102"},
        {34, "54145235", "20702236"},
        {35, "63100104", "23455231"},
        {36, "35317452", "17352571"},
        {37, "10714032", "07417741"},
        {38, "43602423", "47415564"},
        {39, "13700511", "36550046"},
        {40, "67442654", "41615230"},
        {41, "42621301", "70270411"},
        {42, "25413532", "73527103"},
        {43, "73475715", "20344205"},
        {44, "60600610", "33470052"},
        {45, "22362271", "73213175"},
        {46, "73341370", "21175624"},
        {47, "76412463", "71174640"},
        {48, "10475522", "77336306"},
        {49, "31662361", "52645772"},
        {50, "72164341", "10166636"},
        {51, "03600703", "62442252"},
        {52, "12734207", "47205776"},
        {53, "66744236", "67053707"},
        {54, "66354613", "12103375"},
        {55, "42710457", "01304276"},
        {56, "72744364", "62223707"},
        {57, "76720625", "03111453"},
        {58, "46643276", "34250037"},
        {59, "53525215", "71514224"},
        {60, "42453402", "36620001"},
        {61, "26604754", "70502406"},
        {62, "35027021", "07344636"},
        {63, "12073317", "30264212"}});
    for (auto& d : data)
        {
            std::string pilot_secondary = beidou_b2a_pilot_secondary_code(d.prn);
            EXPECT_EQ(chips_to_octal24_b2a(pilot_secondary, 0), d.first24);
            EXPECT_EQ(chips_to_octal24_b2a(pilot_secondary, 76), d.last24);
        }
}
