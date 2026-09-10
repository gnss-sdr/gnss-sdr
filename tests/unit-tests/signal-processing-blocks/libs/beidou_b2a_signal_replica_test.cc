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
    beidou_b2a_code_gen_complex_sampled(sampled, 19, fs, 0);
    EXPECT_EQ(static_cast<int32_t>(sampled.size()), samples_per_code);
    EXPECT_FLOAT_EQ(sampled.front().real(), 1.0F);  // PRN 19 first chip is 1 (octal 4...)
}
