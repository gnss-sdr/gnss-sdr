/*!
 * \file beidou_b2b_signal_replica.cc
 * \brief BeiDou B2b_I ranging-code replica (BDS-SIS-ICD-B2b-1.0 Figure 5-1)
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

#include "beidou_b2b_signal_replica.h"
#include <array>
#include <cstdint>

namespace
{
const auto AUX_CEIL = [](float x) { return static_cast<int32_t>(static_cast<int64_t>((x) + 1)); };

// G2 initial phase s2,1 ... s2,13 for PRN 1..63 (ICD Table 5-1)
constexpr std::array<uint16_t, 63> B2B_G2_INIT = {
    0x1025, 0x1034, 0x10AD, 0x114F, 0x1155, 0x11AE, 0x11EE, 0x11FB,
    0x1329, 0x13DA, 0x1435, 0x1444, 0x1455, 0x145B, 0x145C, 0x14A3,
    0x14F7, 0x1501, 0x153E, 0x15AB, 0x15B1, 0x1653, 0x1662, 0x1698,
    0x16B6, 0x16F2, 0x16FF, 0x1712, 0x173C, 0x17A1, 0x17C8, 0x17D4,
    0x17EB, 0x17F3, 0x1851, 0x1894, 0x18B7, 0x1911, 0x1919, 0x19AB,
    0x19B1, 0x19D2, 0x1A55, 0x1A74, 0x1ACB, 0x1B57, 0x1C34, 0x1C83,
    0x1C8B, 0x1CA3, 0x1CA8, 0x1D3B, 0x1D97, 0x1E48, 0x1E94, 0x1E99,
    0x1EDA, 0x1EF8, 0x1EFF, 0x1FB5, 0x1FBD, 0x0B85, 0x0B3B};

constexpr int G1_TAPS[4] = {0, 8, 9, 12};
constexpr int G2_TAPS[6] = {2, 3, 5, 8, 11, 12};
}  // namespace


void beidou_b2b_code_gen_int(own::span<int> dest, int32_t prn, uint32_t chip_shift)
{
    constexpr uint32_t code_length = 10230;
    constexpr uint32_t g1_reset_chip = 8190;
    const int32_t prn_idx = prn - 1;
    if ((prn_idx < 0) || (prn_idx > 62) || (dest.size() < code_length))
        {
            return;
        }

    int g1[13];
    int g2[13];
    for (int i = 0; i < 13; i++)
        {
            g1[i] = 1;
            g2[i] = static_cast<int>((B2B_G2_INIT[static_cast<size_t>(prn_idx)] >> (12 - i)) & 1U);
        }

    std::array<int, code_length> chips{};
    for (uint32_t i = 0; i < code_length; i++)
        {
            chips[i] = g1[12] ^ g2[12];
            int fb1 = 0;
            int fb2 = 0;
            for (int t : G1_TAPS)
                {
                    fb1 ^= g1[t];
                }
            for (int t : G2_TAPS)
                {
                    fb2 ^= g2[t];
                }
            for (int k = 12; k > 0; k--)
                {
                    g1[k] = g1[k - 1];
                    g2[k] = g2[k - 1];
                }
            g1[0] = fb1;
            g2[0] = fb2;
            if ((i + 1U) == g1_reset_chip)
                {
                    for (int k = 0; k < 13; k++)
                        {
                            g1[k] = 1;
                        }
                }
        }

    for (uint32_t i = 0; i < code_length; i++)
        {
            const uint32_t idx = (i + chip_shift) % code_length;
            dest[i] = (chips[idx] == 1) ? 1 : -1;
        }
}


void beidou_b2b_code_gen_float(own::span<float> dest, int32_t prn, uint32_t chip_shift)
{
    constexpr uint32_t code_length = 10230;
    std::array<int, code_length> code_int{};
    beidou_b2b_code_gen_int(code_int, prn, chip_shift);
    for (uint32_t ii = 0; ii < code_length; ++ii)
        {
            dest[ii] = static_cast<float>(code_int[ii]);
        }
}


void beidou_b2b_code_gen_complex(own::span<std::complex<float>> dest, int32_t prn, uint32_t chip_shift)
{
    constexpr uint32_t code_length = 10230;
    std::array<int, code_length> code_int{};
    beidou_b2b_code_gen_int(code_int, prn, chip_shift);
    for (uint32_t ii = 0; ii < code_length; ++ii)
        {
            dest[ii] = std::complex<float>(static_cast<float>(code_int[ii]), 0.0F);
        }
}


void beidou_b2b_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int sampling_freq, uint32_t chip_shift)
{
    constexpr int32_t codeFreqBasis = 10230000;
    constexpr int32_t codeLength = 10230;
    constexpr float tc = 1.0F / static_cast<float>(codeFreqBasis);
    const float ts = 1.0F / static_cast<float>(sampling_freq);
    const auto samplesPerCode = static_cast<int32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(codeFreqBasis) / static_cast<double>(codeLength)));

    std::array<std::complex<float>, 10230> code_aux{};
    beidou_b2b_code_gen_complex(code_aux, static_cast<int32_t>(prn), chip_shift);

    for (int32_t i = 0; i < samplesPerCode; i++)
        {
            const float aux = (ts * (static_cast<float>(i) + 1)) / tc;
            const int32_t codeValueIndex = AUX_CEIL(aux) - 1;
            if (i == samplesPerCode - 1)
                {
                    dest[i] = code_aux[codeLength - 1];
                }
            else
                {
                    dest[i] = code_aux[codeValueIndex];
                }
        }
}
