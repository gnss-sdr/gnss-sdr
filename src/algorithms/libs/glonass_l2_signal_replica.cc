/*!
 * \file glonass_l2_signal_replica.cc
 * \brief This file implements various functions for GLONASS L2 CA signal
 * replica generation
 * \author Damian Miralles, 2018, dmiralles2009(at)gmail.com
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

#include "glonass_l2_signal_replica.h"
#include <array>
#include <bitset>

const auto AUX_CEIL = [](float x) { return static_cast<int32_t>(static_cast<int64_t>((x) + 1)); };

void glonass_l2_ca_code_gen_complex(own::span<std::complex<float>> dest, uint32_t chip_shift)
{
    const uint32_t code_length = 511;
    std::bitset<code_length> G1{};
    auto G1_register = std::bitset<9>{}.set();  // All true
    uint32_t lcv;
    uint32_t lcv2;
    bool feedback1;
    bool aux;

    /* Generate G1 Register */
    for (lcv = 0; lcv < code_length; lcv++)
        {
            G1[lcv] = G1_register[2];

            feedback1 = G1_register[4] ^ G1_register[0];

            for (lcv2 = 0; lcv2 < 8; lcv2++)
                {
                    G1_register[lcv2] = G1_register[lcv2 + 1];
                }

            G1_register[8] = feedback1;
        }

    /* Generate PRN from G1 Register */
    for (lcv = 0; lcv < code_length; lcv++)
        {
            aux = G1[lcv];
            if (aux == true)
                {
                    dest[lcv] = std::complex<float>(1, 0);
                }
            else
                {
                    dest[lcv] = std::complex<float>(-1, 0);
                }
        }

    /* Generate PRN from G1 and G2 Registers */
    for (lcv = 0; lcv < code_length; lcv++)
        {
            aux = G1[(lcv + chip_shift) % code_length];
            if (aux == true)
                {
                    dest[lcv] = std::complex<float>(1, 0);
                }
            else
                {
                    dest[lcv] = std::complex<float>(-1, 0);
                }
        }
}


/*
 *  Generates complex GLONASS L2 C/A code for the desired SV ID and sampled to specific sampling frequency
 */
void glonass_l2_ca_code_gen_complex_sampled(own::span<std::complex<float>> dest, int32_t sampling_freq, uint32_t chip_shift)
{
    constexpr int32_t codeFreqBasis = 511000;  // chips per second
    constexpr int32_t codeLength = 511;
    constexpr float tc = 1.0 / static_cast<float>(codeFreqBasis);  // C/A chip period in sec

    const auto samplesPerCode = static_cast<int32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(codeFreqBasis) / static_cast<double>(codeLength)));
    const float ts = 1.0F / static_cast<float>(sampling_freq);  // Sampling period in sec

    std::array<std::complex<float>, 511> code_aux{};
    int32_t codeValueIndex;
    float aux;

    glonass_l2_ca_code_gen_complex(code_aux, chip_shift);  // generate C/A code 1 sample per chip

    for (int32_t i = 0; i < samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read C/A code values --------------------
            // The length of the index array depends on the sampling frequency -
            // number of samples per millisecond (because one C/A code period is
            // one millisecond).

            aux = (ts * (static_cast<float>(i) + 1)) / tc;
            codeValueIndex = AUX_CEIL(aux) - 1;

            // --- Make the digitized version of the C/A code ------------------
            // The "upsampled" code is made by selecting values form the CA code
            // chip array (caCode) for the time instances of each sample.
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
