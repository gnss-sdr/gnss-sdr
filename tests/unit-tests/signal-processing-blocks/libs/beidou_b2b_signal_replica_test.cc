/*!
 * \file beidou_b2b_signal_replica_test.cc
 * \brief Unit tests for BeiDou B2b_I ranging-code generation vs ICD Table 5-1
 * \author Chandoss, 2026. huangchh37(at)mail2.sysu.edu.cn
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
#include "Beidou_B2b.h"
#include "beidou_b2b_signal_replica.h"
#include <gtest/gtest.h>
#include <array>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace
{
std::string chips_to_octal24(const int* chips, int start)
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


TEST(BeidouB2bSignalReplicaTest, CodeLength)
{
    std::array<int, 10230> chips{};
    beidou_b2b_code_gen_int(chips, 19, 0);
    EXPECT_EQ(chips.size(), static_cast<size_t>(BEIDOU_B2B_CODE_LENGTH_CHIPS));
}


TEST(BeidouB2bSignalReplicaTest, Prn6FirstLast24MatchIcd)
{
    std::array<int, 10230> chips{};
    beidou_b2b_code_gen_int(chips, 6, 0);
    EXPECT_EQ(chips_to_octal24(chips.data(), 0), "42471422");
    EXPECT_EQ(chips_to_octal24(chips.data(), 10206), "44530033");
}


TEST(BeidouB2bSignalReplicaTest, Prn19FirstLast24MatchIcd)
{
    std::array<int, 10230> chips{};
    beidou_b2b_code_gen_int(chips, 19, 0);
    EXPECT_EQ(chips_to_octal24(chips.data(), 0), "40652553");
    EXPECT_EQ(chips_to_octal24(chips.data(), 10206), "14350465");
}


TEST(BeidouB2bSignalReplicaTest, SampledCodeLengthAt20Msps)
{
    constexpr int32_t fs = 20000000;
    const auto samples_per_code = static_cast<int32_t>(static_cast<double>(fs) /
                                                       (BEIDOU_B2B_CODE_RATE_CPS / BEIDOU_B2B_CODE_LENGTH_CHIPS));
    std::vector<std::complex<float>> sampled(static_cast<size_t>(samples_per_code));
    beidou_b2b_code_gen_complex_sampled(sampled, 19, fs, 0);
    EXPECT_EQ(static_cast<int32_t>(sampled.size()), samples_per_code);
    EXPECT_FLOAT_EQ(sampled.front().real(), 1.0F);  // PRN 19 first chip is 1 (octal 4...)
}
