/*!
 * \file beidou_b1i_signal_replica.cc
 * \brief This file implements various functions for BeiDou B1I signal replica
 * generation
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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

#include "beidou_b1i_signal_replica.h"
#include <array>
#include <bitset>
#include <string>

const auto AUX_CEIL = [](float x) { return static_cast<int32_t>(static_cast<int64_t>((x) + 1)); };

void beidou_b1i_code_gen_int(own::span<int32_t> dest, int32_t prn, uint32_t chip_shift)
{
    constexpr uint32_t code_length = 2046;
    const std::array<int32_t, 63> phase1 = {1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 8, 8, 8, 9, 9, 10, 2, 3, 3, 3, 3, 3, 4, 4, 5, 5, 5, 5, 6, 8, 9, 9, 3, 5, 7, 4, 4, 5, 5, 5, 5, 6};
    const std::array<int32_t, 63> phase2 = {3, 4, 5, 6, 8, 9, 10, 11, 7, 4, 5, 6, 8, 9, 10, 11, 5, 6, 8, 9, 10, 11, 6, 8, 9, 10, 11, 8, 9, 10, 11, 9, 10, 11, 10, 11, 11, 7, 4, 6, 8, 10, 11, 5, 9, 6, 8, 10, 11, 9, 9, 10, 11, 7, 7, 9, 5, 9, 6, 8, 10, 11, 9};
    const std::array<int32_t, 63> phase3 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3};

    std::bitset<code_length> G1{};
    std::bitset<code_length> G2{};

    std::bitset<11> G1_register(std::string("01010101010"));
    std::bitset<11> G2_register(std::string("01010101010"));

    bool feedback1;
    bool feedback2;
    bool aux;
    uint32_t lcv;
    uint32_t lcv2;
    uint32_t delay;
    int32_t prn_idx;

    // compute delay array index for given PRN number
    prn_idx = prn - 1;

    // A simple error check
    if ((prn_idx < 0) || (prn_idx > 62))
        {
            return;
        }

    // Generate G1 & G2 Register
    for (lcv = 0; lcv < code_length; lcv++)
        {
            G1[lcv] = G1_register[0];
            G2[lcv] = G2_register[-(phase1[prn_idx] - 11)] xor G2_register[-(phase2[prn_idx] - 11)] xor (phase3[prn_idx] ? G2_register[-(phase3[prn_idx] - 11)] : 0);

            feedback1 = G1_register[0] xor G1_register[1] xor G1_register[2] xor G1_register[3] xor G1_register[4] xor G1_register[10];
            feedback2 = G2_register[0] xor G2_register[2] xor G2_register[3] xor G2_register[6] xor G2_register[7] xor G2_register[8] xor G2_register[9] xor G2_register[10];

            for (lcv2 = 0; lcv2 < 10; lcv2++)
                {
                    G1_register[lcv2] = G1_register[lcv2 + 1];
                    G2_register[lcv2] = G2_register[lcv2 + 1];
                }

            G1_register[10] = feedback1;
            G2_register[10] = feedback2;
        }

    // Set the delay
    delay = code_length;  // *********************************
    delay += chip_shift;
    delay %= code_length;

    // Generate PRN from G1 and G2 Registers
    for (lcv = 0; lcv < code_length; lcv++)
        {
            aux = G1[(lcv + chip_shift) % code_length] xor G2[delay];
            if (aux == true)
                {
                    dest[lcv] = 1;
                }
            else
                {
                    dest[lcv] = -1;
                }

            delay++;
            delay %= code_length;
        }
}


void beidou_b1i_code_gen_float(own::span<float> dest, int32_t prn, uint32_t chip_shift)
{
    constexpr uint32_t code_length = 2046;
    std::array<int32_t, code_length> b1i_code_int{};

    beidou_b1i_code_gen_int(own::span<int32_t>(b1i_code_int.data(), code_length), prn, chip_shift);

    for (uint32_t ii = 0; ii < code_length; ++ii)
        {
            dest[ii] = static_cast<float>(b1i_code_int[ii]);
        }
}


void beidou_b1i_code_gen_complex(own::span<std::complex<float>> dest, int32_t prn, uint32_t chip_shift)
{
    constexpr uint32_t code_length = 2046;
    std::array<int32_t, code_length> b1i_code_int{};

    beidou_b1i_code_gen_int(own::span<int32_t>(b1i_code_int.data(), code_length), prn, chip_shift);

    for (uint32_t ii = 0; ii < code_length; ++ii)
        {
            dest[ii] = std::complex<float>(static_cast<float>(b1i_code_int[ii]), 0.0F);
        }
}


/*
 *  Generates complex BeiDou B1I code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1i_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift)
{
    constexpr int32_t codeFreqBasis = 2046000;  // chips per second
    constexpr int32_t codeLength = 2046;
    constexpr float tc = 1.0 / static_cast<float>(codeFreqBasis);  // B1I chip period in sec

    const auto samplesPerCode = static_cast<int32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(codeFreqBasis) / static_cast<double>(codeLength)));
    const float ts = 1.0F / static_cast<float>(sampling_freq);  // Sampling period in sec

    std::array<std::complex<float>, 2046> code_aux{};
    int32_t codeValueIndex;
    float aux;

    beidou_b1i_code_gen_complex(code_aux, prn, chip_shift);  // generate B1I code 1 sample per chip

    for (int32_t i = 0; i < samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read B1I code values --------------------
            // The length of the index array depends on the sampling frequency -
            // number of samples per millisecond (because one B1I code period is
            // one millisecond).

            aux = (ts * (static_cast<float>(i) + 1)) / tc;
            codeValueIndex = AUX_CEIL(aux) - 1;

            // --- Make the digitized version of the B1I code ------------------
            // The upsampled code is made by selecting values from the B1I code
            // chip array for the time instances of each sample.
            if (i == samplesPerCode - 1)
                {
                    // Correct the last index (due to number rounding issues)
                    dest[i] = code_aux[codeLength - 1];
                }
            else
                {
                    dest[i] = code_aux[codeValueIndex];  // repeat the chip -> upsample
                }
        }
}
