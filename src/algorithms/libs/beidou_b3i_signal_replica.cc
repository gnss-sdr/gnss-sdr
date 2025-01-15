/*!
 * \file beidou_b3i_signal_replica.cc
 * \brief This file implements various functions for BeiDou B3I signal replica
 * generation
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "beidou_b3i_signal_replica.h"
#include <array>
#include <bitset>
#include <string>

const auto AUX_CEIL = [](float x) { return static_cast<int32_t>(static_cast<int64_t>((x) + 1)); };

void beidou_b3i_code_gen_int(own::span<int> dest, int32_t prn, uint32_t chip_shift)
{
    constexpr uint32_t code_length = 10230;
    std::bitset<code_length> G1{};
    std::bitset<code_length> G2{};
    auto G1_register = std::bitset<13>{}.set();  // All true
    auto G2_register = std::bitset<13>{}.set();  // All true
    auto G1_register_reset = std::bitset<13>{}.set();
    G1_register_reset.reset(0);
    G1_register_reset.reset(1);  // {false, false, true, true, true, true, true, true, true, true, true, true, true};

    bool feedback1;
    bool feedback2;
    bool aux;
    uint32_t lcv;
    uint32_t lcv2;
    uint32_t delay;
    int32_t prn_idx = prn - 1;

    const std::array<std::bitset<13>, 63> G2_register_shifted =
        {std::bitset<13>(std::string("1010111111111")),
            std::bitset<13>(std::string("1111000101011")),
            std::bitset<13>(std::string("1011110001010")),
            std::bitset<13>(std::string("1111111111011")),
            std::bitset<13>(std::string("1100100011111")),
            std::bitset<13>(std::string("1001001100100")),
            std::bitset<13>(std::string("1111111010010")),
            std::bitset<13>(std::string("1110111111101")),
            std::bitset<13>(std::string("1010000000010")),
            std::bitset<13>(std::string("0010000011011")),
            std::bitset<13>(std::string("1110101110000")),
            std::bitset<13>(std::string("0010110011110")),
            std::bitset<13>(std::string("0110010010101")),
            std::bitset<13>(std::string("0111000100110")),
            std::bitset<13>(std::string("1000110001001")),
            std::bitset<13>(std::string("1110001111100")),
            std::bitset<13>(std::string("0010011000101")),
            std::bitset<13>(std::string("0000011101100")),
            std::bitset<13>(std::string("1000101010111")),
            std::bitset<13>(std::string("0001011011110")),
            std::bitset<13>(std::string("0010000101101")),
            std::bitset<13>(std::string("0010110001010")),
            std::bitset<13>(std::string("0001011001111")),
            std::bitset<13>(std::string("0011001100010")),
            std::bitset<13>(std::string("0011101001000")),
            std::bitset<13>(std::string("0100100101001")),
            std::bitset<13>(std::string("1011011010011")),
            std::bitset<13>(std::string("1010111100010")),
            std::bitset<13>(std::string("0001011110101")),
            std::bitset<13>(std::string("0111111111111")),
            std::bitset<13>(std::string("0110110001111")),
            std::bitset<13>(std::string("1010110001001")),
            std::bitset<13>(std::string("1001010101011")),
            std::bitset<13>(std::string("1100110100101")),
            std::bitset<13>(std::string("1101001011101")),
            std::bitset<13>(std::string("1111101110100")),
            std::bitset<13>(std::string("0010101100111")),
            std::bitset<13>(std::string("1110100010000")),
            std::bitset<13>(std::string("1101110010000")),
            std::bitset<13>(std::string("1101011001110")),
            std::bitset<13>(std::string("1000000110100")),
            std::bitset<13>(std::string("0101111011001")),
            std::bitset<13>(std::string("0110110111100")),
            std::bitset<13>(std::string("1101001110001")),
            std::bitset<13>(std::string("0011100100010")),
            std::bitset<13>(std::string("0101011000101")),
            std::bitset<13>(std::string("1001111100110")),
            std::bitset<13>(std::string("1111101001000")),
            std::bitset<13>(std::string("0000101001001")),
            std::bitset<13>(std::string("1000010101100")),
            std::bitset<13>(std::string("1111001001100")),
            std::bitset<13>(std::string("0100110001111")),
            std::bitset<13>(std::string("0000000011000")),
            std::bitset<13>(std::string("1000000000100")),
            std::bitset<13>(std::string("0011010100110")),
            std::bitset<13>(std::string("1011001000110")),
            std::bitset<13>(std::string("0111001111000")),
            std::bitset<13>(std::string("0010111001010")),
            std::bitset<13>(std::string("1100111110110")),
            std::bitset<13>(std::string("1001001000101")),
            std::bitset<13>(std::string("0111000100000")),
            std::bitset<13>(std::string("0011001000010")),
            std::bitset<13>(std::string("0010001001110"))};

    // A simple error check
    if ((prn_idx < 0) || (prn_idx > 63))
        {
            return;
        }

    // Assign shifted G2 register based on prn number
    G2_register = G2_register_shifted[prn_idx];

    // Generate G1 and G2 Register
    for (lcv = 0; lcv < code_length; lcv++)
        {
            G1[lcv] = G1_register[0];
            G2[lcv] = G2_register[0];

            feedback1 = G1_register[0] xor G1_register[9] xor G1_register[10] xor G1_register[12];
            feedback2 = G2_register[0] xor G2_register[1] xor G2_register[3] xor G2_register[4] xor
                        G2_register[6] xor G2_register[7] xor G2_register[8] xor G2_register[12];

            for (lcv2 = 0; lcv2 < 12; lcv2++)
                {
                    G1_register[lcv2] = G1_register[lcv2 + 1];
                    G2_register[lcv2] = G2_register[lcv2 + 1];
                }

            G1_register[12] = feedback1;
            G2_register[12] = feedback2;

            // Reset G1 register if sequence found
            if (G1_register == G1_register_reset)
                {
                    G1_register = std::bitset<13>{}.set();  // All true
                }
        }

    delay = code_length;
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


void beidou_b3i_code_gen_float(own::span<float> dest, int32_t prn, uint32_t chip_shift)
{
    constexpr uint32_t code_length = 10230;
    std::array<int, code_length> b3i_code_int{};

    beidou_b3i_code_gen_int(b3i_code_int, prn, chip_shift);

    for (uint32_t ii = 0; ii < code_length; ++ii)
        {
            dest[ii] = static_cast<float>(b3i_code_int[ii]);
        }
}


void beidou_b3i_code_gen_complex(own::span<std::complex<float>> dest, int32_t prn, uint32_t chip_shift)
{
    constexpr uint32_t code_length = 10230;
    std::array<int, code_length> b3i_code_int{};

    beidou_b3i_code_gen_int(b3i_code_int, prn, chip_shift);

    for (uint32_t ii = 0; ii < code_length; ++ii)
        {
            dest[ii] = std::complex<float>(static_cast<float>(b3i_code_int[ii]), 0.0F);
        }
}


void beidou_b3i_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int sampling_freq, uint32_t chip_shift)
{
    constexpr int32_t codeFreqBasis = 10230000;  // chips per second
    constexpr int32_t codeLength = 10230;
    constexpr float tc = 1.0 / static_cast<float>(codeFreqBasis);  // B3I chip period in sec

    const float ts = 1.0F / static_cast<float>(sampling_freq);  // Sampling period in secs
    const auto samplesPerCode = static_cast<int32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(codeFreqBasis) / static_cast<double>(codeLength)));

    std::array<std::complex<float>, 10230> code_aux{};

    int32_t codeValueIndex;
    float aux;

    beidou_b3i_code_gen_complex(code_aux, prn, chip_shift);  // generate B3I code 1 sample per chip

    for (int32_t i = 0; i < samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read B3I code values --------------------
            // The length of the index array depends on the sampling frequency -
            // number of samples per millisecond (because one B3I code period is
            // one millisecond).

            aux = (ts * (static_cast<float>(i) + 1)) / tc;
            codeValueIndex = AUX_CEIL(aux) - 1;

            // --- Make the digitized version of the B3I code ------------------
            // The upsampled code is made by selecting values from the B3I code
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
