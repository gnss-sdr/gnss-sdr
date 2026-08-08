/*!
 * \file beidou_b1c_signal_replica_test.cc
 * \brief Unit tests for BeiDou B1C signal replica generation
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
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
#include "Beidou_B1C.h"
#include "beidou_b1c_signal_replica.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <complex>
#include <vector>

TEST(BeidouB1cSignalReplicaTest, SampledCodeLength)
{
    constexpr int32_t fs = 2046000;
    const auto samples_per_chip = static_cast<uint32_t>(std::lround(static_cast<double>(fs) / BEIDOU_B1C_CODE_RATE_CPS));
    const auto expected_length = static_cast<size_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS * samples_per_chip);
    std::vector<std::complex<float>> code(expected_length);
    const std::array<char, 3> data_signal = {{'1', 'D', '\0'}};
    beidou_b1c_code_gen_complex_sampled(code, data_signal, false, 1, fs, 0);
    EXPECT_EQ(code.size(), expected_length);
}


TEST(BeidouB1cSignalReplicaTest, Prn1DataFirst24ChipsMatchIcd)
{
    std::array<int32_t, 10230> chips{};
    const std::array<char, 3> data_signal = {{'1', 'D', '\0'}};
    beidou_b1c_code_gen_int(chips, data_signal, 1);
    const char* expected = "101011111111011001001110";
    for (int i = 0; i < 24; i++)
        {
            EXPECT_EQ(chips[i], expected[i] - '0') << "chip index " << i;
        }
}


TEST(BeidouB1cSignalReplicaTest, SinBoc11AutocorrelationShape)
{
    std::array<float, 20460> code{};
    const std::array<char, 3> data_signal = {{'1', 'D', '\0'}};
    beidou_b1c_code_gen_sinboc11_float(code, data_signal, 1);

    double peak = 0.0;
    double side = 0.0;
    const int32_t samples_per_chip = 2;
    const int32_t code_samples = static_cast<int32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS) * samples_per_chip;
    for (int32_t lag = -code_samples; lag <= code_samples; lag++)
        {
            double corr = 0.0;
            for (int32_t i = 0; i < code_samples; i++)
                {
                    const int32_t j = (i + lag + code_samples) % code_samples;
                    corr += static_cast<double>(code[i]) * static_cast<double>(code[j]);
                }
            corr /= static_cast<double>(code_samples);
            if (lag == 0)
                {
                    peak = std::abs(corr);
                }
            else if (std::abs(lag) == samples_per_chip)
                {
                    side = std::max(side, std::abs(corr));
                }
        }
    EXPECT_GT(peak, 0.0);
    EXPECT_GT(side, 0.0);
    EXPECT_LT(side, peak);
}


TEST(BeidouB1cSignalReplicaTest, QmbocPilotKeepsComponentsInQuadrature)
{
    // ICD eqs. (4-10)/(4-11): Re=BOC(6,1), Im=BOC(1,1).
    constexpr int32_t fs = 12 * 1023000;
    const auto expected_length = static_cast<size_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS * 12);
    std::vector<std::complex<float>> code(expected_length);
    const std::array<char, 3> pilot_signal = {{'1', 'P', '\0'}};
    beidou_b1c_code_gen_complex_sampled(code, pilot_signal, true, 1, fs, 0);

    double power_re = 0.0;
    double power_im = 0.0;
    double cross = 0.0;
    for (const auto& s : code)
        {
            power_re += static_cast<double>(s.real()) * static_cast<double>(s.real());
            power_im += static_cast<double>(s.imag()) * static_cast<double>(s.imag());
            cross += static_cast<double>(s.real()) * static_cast<double>(s.imag());
        }
    ASSERT_GT(power_re, 0.0);
    ASSERT_GT(power_im, 0.0);
    EXPECT_NEAR(power_re / power_im, 4.0 / 29.0, 1.0e-3);
    // Re and Im should be orthogonal.
    EXPECT_NEAR(cross / static_cast<double>(code.size()), 0.0, 1.0e-3);
}
