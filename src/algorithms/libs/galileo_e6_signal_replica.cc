/*!
 * \file galileo_e6_signal_replica.cc
 * \brief This library implements various functions for Galileo E6 signal
 * replica generation
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
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


#include "galileo_e6_signal_replica.h"
#include "Galileo_E6.h"
#include "gnss_signal_replica.h"
#include <iostream>
#include <vector>

void galileo_e6_b_code_gen_complex_primary(own::span<std::complex<float>> dest,
    int32_t prn)
{
    const uint32_t prn_ = prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((prn < 1) || (prn > 50))
        {
            return;
        }

    for (size_t i = 0; i < GALILEO_E6_B_PRIMARY_CODE_STR_LENGTH - 1; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_B_PRIMARY_CODE[prn_][i]);
            dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
            dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
            dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
            index = index + 4;
        }
    // last bit is filled up with a zero
    hex_to_binary_converter(a, GALILEO_E6_B_PRIMARY_CODE[prn_][GALILEO_E6_B_PRIMARY_CODE_STR_LENGTH - 1]);
    dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
    dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
    dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
}


void galileo_e6_b_code_gen_float_primary(own::span<float> dest, int32_t prn)
{
    const uint32_t prn_ = prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((prn < 1) || (prn > 50))
        {
            return;
        }

    for (size_t i = 0; i < GALILEO_E6_B_PRIMARY_CODE_STR_LENGTH - 1; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_B_PRIMARY_CODE[prn_][i]);
            dest[index] = static_cast<float>(a[0]);
            dest[index + 1] = static_cast<float>(a[1]);
            dest[index + 2] = static_cast<float>(a[2]);
            dest[index + 3] = static_cast<float>(a[3]);
            index = index + 4;
        }
    // last bit is filled up with a zero
    hex_to_binary_converter(a, GALILEO_E6_B_PRIMARY_CODE[prn_][GALILEO_E6_B_PRIMARY_CODE_STR_LENGTH - 1]);
    dest[index] = static_cast<float>(a[0]);
    dest[index + 1] = static_cast<float>(a[1]);
    dest[index + 2] = static_cast<float>(a[2]);
}


void galileo_e6_b_code_gen_complex_sampled(own::span<std::complex<float>> dest,
    uint32_t prn,
    int32_t sampling_freq,
    uint32_t chip_shift)
{
    constexpr uint32_t codeLength = GALILEO_E6_B_CODE_LENGTH_CHIPS;
    constexpr int32_t codeFreqBasis = GALILEO_E6_B_CODE_CHIP_RATE_CPS;

    const auto samplesPerCode = static_cast<uint32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(codeFreqBasis) / static_cast<double>(codeLength)));
    const uint32_t delay = ((codeLength - chip_shift) % codeLength) * samplesPerCode / codeLength;

    std::vector<std::complex<float>> code_aux(codeLength);
    galileo_e6_b_code_gen_complex_primary(code_aux, prn);

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


void galileo_e6_c_code_gen_complex_primary(own::span<std::complex<float>> dest,
    int32_t prn)
{
    const uint32_t prn_ = prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((prn < 1) || (prn > 50))
        {
            return;
        }
    for (size_t i = 0; i < GALILEO_E6_C_PRIMARY_CODE_STR_LENGTH - 1; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_C_PRIMARY_CODE[prn_][i]);
            dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
            dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
            dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
            index = index + 4;
        }
    // last bit is filled up with a zero
    hex_to_binary_converter(a, GALILEO_E6_C_PRIMARY_CODE[prn_][GALILEO_E6_C_PRIMARY_CODE_STR_LENGTH - 1]);
    dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
    dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
    dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
}


void galileo_e6_c_code_gen_float_primary(own::span<float> dest, int32_t prn)
{
    const uint32_t prn_ = prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((prn < 1) || (prn > 50))
        {
            return;
        }
    for (size_t i = 0; i < GALILEO_E6_C_PRIMARY_CODE_STR_LENGTH - 1; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_C_PRIMARY_CODE[prn_][i]);
            dest[index] = static_cast<float>(a[0]);
            dest[index + 1] = static_cast<float>(a[1]);
            dest[index + 2] = static_cast<float>(a[2]);
            dest[index + 3] = static_cast<float>(a[3]);
            index = index + 4;
        }
    // last bit is filled up with a zero
    hex_to_binary_converter(a, GALILEO_E6_C_PRIMARY_CODE[prn_][GALILEO_E6_C_PRIMARY_CODE_STR_LENGTH - 1]);
    dest[index] = static_cast<float>(a[0]);
    dest[index + 1] = static_cast<float>(a[1]);
    dest[index + 2] = static_cast<float>(a[2]);
}


void galileo_e6_c_code_gen_complex_sampled(own::span<std::complex<float>> dest,
    uint32_t prn,
    int32_t sampling_freq,
    uint32_t chip_shift)
{
    constexpr uint32_t codeLength = GALILEO_E6_C_CODE_LENGTH_CHIPS;
    constexpr int32_t codeFreqBasis = GALILEO_E6_C_CODE_CHIP_RATE_CPS;

    const auto samplesPerCode = static_cast<uint32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(codeFreqBasis) / static_cast<double>(codeLength)));
    const uint32_t delay = ((codeLength - chip_shift) % codeLength) * samplesPerCode / codeLength;

    std::vector<std::complex<float>> code_aux(codeLength);
    galileo_e6_c_code_gen_complex_primary(code_aux, prn);

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


void galileo_e6_c_secondary_code_gen_complex(own::span<std::complex<float>> dest,
    int32_t prn)
{
    const uint32_t prn_ = prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((prn < 1) || (prn > 50))
        {
            return;
        }
    for (size_t i = 0; i < GALILEO_E6_C_SECONDARY_CODE_STR_LENGTH; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_C_SECONDARY_CODE[prn_][i]);
            dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
            dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
            dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
            index = index + 4;
        }
}


void galileo_e6_c_secondary_code_gen_float(own::span<float> dest,
    int32_t prn)
{
    const uint32_t prn_ = prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((prn < 1) || (prn > 50))
        {
            return;
        }
    for (size_t i = 0; i < GALILEO_E6_C_SECONDARY_CODE_STR_LENGTH; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_C_SECONDARY_CODE[prn_][i]);
            dest[index] = static_cast<float>(a[0]);
            dest[index + 1] = static_cast<float>(a[1]);
            dest[index + 2] = static_cast<float>(a[2]);
            dest[index + 3] = static_cast<float>(a[3]);
            index = index + 4;
        }
}


std::string galileo_e6_c_secondary_code(int32_t prn)
{
    std::string dest(static_cast<size_t>(GALILEO_E6_C_SECONDARY_CODE_LENGTH_CHIPS), '0');
    const uint32_t prn_ = prn - 1;
    uint32_t index = 0;
    for (size_t i = 0; i < GALILEO_E6_C_SECONDARY_CODE_STR_LENGTH; i++)
        {
            std::string aux = hex_to_binary_string(GALILEO_E6_C_SECONDARY_CODE[prn_][i]);
            dest[index] = aux[0];
            dest[index + 1] = aux[1];
            dest[index + 2] = aux[2];
            dest[index + 3] = aux[3];
            index = index + 4;
        }

    return dest;
}
