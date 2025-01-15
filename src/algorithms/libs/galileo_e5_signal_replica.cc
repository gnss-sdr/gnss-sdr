/*!
 * \file galileo_e5_signal_replica.cc
 * \brief This library implements various functions for Galileo E5 signal
 * replica generation
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \author Piyush Gupta, 2020. piyush04111999@gmail.com
 * \note Code added as part of GSoc 2020 Program.
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

#include "galileo_e5_signal_replica.h"
#include "Galileo_E5a.h"
#include "Galileo_E5b.h"
#include "gnss_signal_replica.h"
#include <gnuradio/gr_complex.h>
#include <memory>
#include <utility>
#include <vector>


void galileo_e5_a_code_gen_complex_primary(own::span<std::complex<float>> dest,
    int32_t prn,
    const std::array<char, 3>& signal_id)
{
    const uint32_t prn_ = prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((prn < 1) || (prn > 50))
        {
            return;
        }
    if (signal_id[0] == '5' && signal_id[1] == 'Q')
        {
            for (size_t i = 0; i < GALILEO_E5A_Q_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5A_Q_PRIMARY_CODE[prn_][i]);
                    dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
                    dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
                    dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
                    dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5A_Q_PRIMARY_CODE[prn_][GALILEO_E5A_Q_PRIMARY_CODE_STR_LENGTH - 1]);
            dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
        }
    else if (signal_id[0] == '5' && signal_id[1] == 'I')
        {
            for (size_t i = 0; i < GALILEO_E5A_I_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn_][i]);
                    dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
                    dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
                    dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
                    dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn_][GALILEO_E5A_I_PRIMARY_CODE_STR_LENGTH - 1]);
            dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
        }
    else if (signal_id[0] == '5' && signal_id[1] == 'X')
        {
            std::array<int32_t, 4> b{};
            for (size_t i = 0; i < GALILEO_E5A_I_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn_][i]);
                    hex_to_binary_converter(b, GALILEO_E5A_Q_PRIMARY_CODE[prn_][i]);
                    dest[index] = std::complex<float>(static_cast<float>(a[0]), static_cast<float>(b[0]));
                    dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), static_cast<float>(b[1]));
                    dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), static_cast<float>(b[2]));
                    dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), static_cast<float>(b[3]));
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn_][GALILEO_E5A_I_PRIMARY_CODE_STR_LENGTH - 1]);
            hex_to_binary_converter(b, GALILEO_E5A_Q_PRIMARY_CODE[prn_][GALILEO_E5A_Q_PRIMARY_CODE_STR_LENGTH - 1]);
            dest[index] = std::complex<float>(static_cast<float>(a[0]), static_cast<float>(b[0]));
            dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), static_cast<float>(b[1]));
        }
}


void galileo_e5_a_code_gen_complex_sampled(own::span<std::complex<float>> dest,
    uint32_t prn,
    const std::array<char, 3>& signal_id,
    int32_t sampling_freq,
    uint32_t chip_shift)
{
    constexpr uint32_t codeLength = GALILEO_E5A_CODE_LENGTH_CHIPS;
    constexpr int32_t codeFreqBasis = GALILEO_E5A_CODE_CHIP_RATE_CPS;

    const auto samplesPerCode = static_cast<uint32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(codeFreqBasis) / static_cast<double>(codeLength)));
    const uint32_t delay = ((codeLength - chip_shift) % codeLength) * samplesPerCode / codeLength;

    std::vector<std::complex<float>> code_aux(codeLength);
    galileo_e5_a_code_gen_complex_primary(code_aux, prn, signal_id);

    if (sampling_freq != codeFreqBasis)
        {
            std::vector<std::complex<float>> resampled_signal_aux(samplesPerCode);
            resampler(code_aux, resampled_signal_aux, codeFreqBasis, sampling_freq);  // resamples code to sampling_freq
            code_aux = std::move(resampled_signal_aux);
        }

    for (uint32_t i = 0; i < samplesPerCode; i++)
        {
            dest[(i + delay) % samplesPerCode] = code_aux[i];
        }
}


void galileo_e5_b_code_gen_complex_primary(own::span<std::complex<float>> dest,
    int32_t prn,
    const std::array<char, 3>& signal_id)
{
    const uint32_t prn_ = prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((prn < 1) || (prn > 50))
        {
            return;
        }
    if (signal_id[0] == '7' && signal_id[1] == 'Q')
        {
            for (size_t i = 0; i < GALILEO_E5B_Q_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5B_Q_PRIMARY_CODE[prn_][i]);
                    dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
                    dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
                    dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
                    dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5B_Q_PRIMARY_CODE[prn_][GALILEO_E5B_Q_PRIMARY_CODE_STR_LENGTH - 1]);
            dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
        }
    else if (signal_id[0] == '7' && signal_id[1] == 'I')
        {
            for (size_t i = 0; i < GALILEO_E5B_I_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5B_I_PRIMARY_CODE[prn_][i]);
                    dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
                    dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
                    dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
                    dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5B_I_PRIMARY_CODE[prn_][GALILEO_E5B_I_PRIMARY_CODE_STR_LENGTH - 1]);
            dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
        }
    else if (signal_id[0] == '7' && signal_id[1] == 'X')
        {
            std::array<int32_t, 4> b{};
            for (size_t i = 0; i < GALILEO_E5B_I_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5B_I_PRIMARY_CODE[prn_][i]);
                    hex_to_binary_converter(b, GALILEO_E5B_Q_PRIMARY_CODE[prn_][i]);
                    dest[index] = std::complex<float>(static_cast<float>(a[0]), static_cast<float>(b[0]));
                    dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), static_cast<float>(b[1]));
                    dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), static_cast<float>(b[2]));
                    dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), static_cast<float>(b[3]));
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5B_I_PRIMARY_CODE[prn_][GALILEO_E5B_I_PRIMARY_CODE_STR_LENGTH - 1]);
            hex_to_binary_converter(b, GALILEO_E5B_Q_PRIMARY_CODE[prn_][GALILEO_E5B_Q_PRIMARY_CODE_STR_LENGTH - 1]);
            dest[index] = std::complex<float>(static_cast<float>(a[0]), static_cast<float>(b[0]));
            dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), static_cast<float>(b[1]));
        }
}


void galileo_e5_b_code_gen_complex_sampled(own::span<std::complex<float>> dest,
    uint32_t prn,
    const std::array<char, 3>& signal_id,
    int32_t sampling_freq,
    uint32_t chip_shift)
{
    constexpr uint32_t codeLength = GALILEO_E5B_CODE_LENGTH_CHIPS;
    constexpr int32_t codeFreqBasis = GALILEO_E5B_CODE_CHIP_RATE_CPS;

    const auto samplesPerCode = static_cast<uint32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(codeFreqBasis) / static_cast<double>(codeLength)));
    const uint32_t delay = ((codeLength - chip_shift) % codeLength) * samplesPerCode / codeLength;

    std::vector<std::complex<float>> code_aux(codeLength);
    galileo_e5_b_code_gen_complex_primary(code_aux, prn, signal_id);

    if (sampling_freq != codeFreqBasis)
        {
            std::vector<std::complex<float>> resampled_signal_aux(samplesPerCode);
            resampler(code_aux, resampled_signal_aux, codeFreqBasis, sampling_freq);  // resamples code to sampling_freq
            code_aux = std::move(resampled_signal_aux);
        }

    for (uint32_t i = 0; i < samplesPerCode; i++)
        {
            dest[(i + delay) % samplesPerCode] = code_aux[i];
        }
}
