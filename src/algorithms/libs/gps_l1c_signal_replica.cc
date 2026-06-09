/*!
 * \file galileo_e1_signal_replica.cc
 * \brief This library implements various functions for Galileo E1 signal
 * replica generation
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "galileo_e1_signal_replica.h"
#include "Galileo_E1.h"
#include "gnss_signal_replica.h"
#include <cmath>
#include <cstddef>  // for size_t
#include <memory>
#include <string>
#include <utility>
#include <vector>


void galileo_e1_code_gen_int(own::span<int> dest, const std::array<char, 3>& signal_id, int32_t prn)
{
    const std::string galileo_signal = signal_id.data();
    const int32_t prn_ = prn - 1;
    int32_t index = 0;

    // A simple error check
    if ((prn < 1) || (prn > 50))
        {
            return;
        }

    if (galileo_signal.rfind("1B") != std::string::npos && galileo_signal.length() >= 2)
        {
            for (size_t i = 0; i < GALILEO_E1_B_PRIMARY_CODE_STR_LENGTH; i++)
                {
                    hex_to_binary_converter(dest.subspan(index, 4), GALILEO_E1_B_PRIMARY_CODE[prn_][i]);
                    index += 4;
                }
        }
    else if (galileo_signal.rfind("1C") != std::string::npos && galileo_signal.length() >= 2)
        {
            for (size_t i = 0; i < GALILEO_E1_C_PRIMARY_CODE_STR_LENGTH; i++)
                {
                    hex_to_binary_converter(dest.subspan(index, 4), GALILEO_E1_C_PRIMARY_CODE[prn_][i]);
                    index += 4;
                }
        }
}


void galileo_e1_sinboc_11_gen_int(own::span<int> dest, own::span<const int> prn)
{
    constexpr uint32_t length_in = GALILEO_E1_B_CODE_LENGTH_CHIPS;
    const auto period = static_cast<uint32_t>(dest.size() / length_in);
    for (uint32_t i = 0; i < length_in; i++)
        {
            for (uint32_t j = 0; j < (period / 2); j++)
                {
                    dest[i * period + j] = prn[i];
                }
            for (uint32_t j = (period / 2); j < period; j++)
                {
                    dest[i * period + j] = -prn[i];
                }
        }
}


void galileo_e1_sinboc_61_gen_int(own::span<int> dest, own::span<const int> prn)
{
    constexpr uint32_t length_in = GALILEO_E1_B_CODE_LENGTH_CHIPS;
    const auto period = static_cast<uint32_t>(dest.size() / length_in);

    for (uint32_t i = 0; i < length_in; i++)
        {
            for (uint32_t j = 0; j < period; j += 2)
                {
                    dest[i * period + j] = prn[i];
                }
            for (uint32_t j = 1; j < period; j += 2)
                {
                    dest[i * period + j] = -prn[i];
                }
        }
}


void galileo_e1_code_gen_sinboc11_float(own::span<float> dest, const std::array<char, 3>& signal_id, uint32_t prn)
{
    const auto codeLength = static_cast<uint32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS);
    std::array<int32_t, 4092> primary_code_E1_chips{};
    galileo_e1_code_gen_int(primary_code_E1_chips, signal_id, prn);  // generate Galileo E1 code, 1 sample per chip
    for (uint32_t i = 0; i < codeLength; i++)
        {
            dest[2 * i] = static_cast<float>(primary_code_E1_chips[i]);
            dest[2 * i + 1] = -dest[2 * i];
        }
}


void galileo_e1_gen_float(own::span<float> dest, own::span<int> prn, const std::array<char, 3>& signal_id)
{
    const auto codeLength = dest.size();
    const float alpha = std::sqrt(10.0F / 11.0F);
    const float beta = std::sqrt(1.0F / 11.0F);
    const std::string galileo_signal = signal_id.data();

    std::vector<int32_t> sinboc_11(codeLength);
    std::vector<int32_t> sinboc_61(codeLength);

    galileo_e1_sinboc_11_gen_int(sinboc_11, prn);  // generate sinboc(1,1) 12 samples per chip
    galileo_e1_sinboc_61_gen_int(sinboc_61, prn);  // generate sinboc(6,1) 12 samples per chip

    if (galileo_signal.rfind("1B") != std::string::npos && galileo_signal.length() >= 2)
        {
            for (size_t i = 0; i < codeLength; i++)
                {
                    dest[i] = alpha * static_cast<float>(sinboc_11[i]) +
                              beta * static_cast<float>(sinboc_61[i]);
                }
        }
    else if (galileo_signal.rfind("1C") != std::string::npos && galileo_signal.length() >= 2)
        {
            for (size_t i = 0; i < codeLength; i++)
                {
                    dest[i] = alpha * static_cast<float>(sinboc_11[i]) -
                              beta * static_cast<float>(sinboc_61[i]);
                }
        }
}


void galileo_e1_code_gen_float_sampled(own::span<float> dest, const std::array<char, 3>& signal_id,
    bool cboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift,
    bool secondary_flag)
{
    constexpr int32_t codeFreqBasis = GALILEO_E1_CODE_CHIP_RATE_CPS;  // chips per second
    const int32_t samplesPerChip = (cboc == true) ? 12 : 2;
    const uint32_t codeLength = samplesPerChip * GALILEO_E1_B_CODE_LENGTH_CHIPS;
    const std::string galileo_signal = signal_id.data();
    auto samplesPerCode = static_cast<uint32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(codeFreqBasis) / GALILEO_E1_B_CODE_LENGTH_CHIPS));
    const uint32_t delay = ((static_cast<int32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS) - chip_shift) % static_cast<int32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS)) * samplesPerCode / GALILEO_E1_B_CODE_LENGTH_CHIPS;

    std::vector<int32_t> primary_code_E1_chips(static_cast<int32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS));

    galileo_e1_code_gen_int(primary_code_E1_chips, signal_id, prn);  // generate Galileo E1 code, 1 sample per chip

    std::vector<float> signal_E1(codeLength);

    if (cboc == true)
        {
            galileo_e1_gen_float(signal_E1, primary_code_E1_chips, signal_id);  // generate cboc 12 samples per chip
        }
    else
        {
            std::vector<int32_t> signal_E1_int(static_cast<int32_t>(codeLength));
            galileo_e1_sinboc_11_gen_int(signal_E1_int, primary_code_E1_chips);  // generate sinboc(1,1) 2 samples per chip

            for (uint32_t ii = 0; ii < codeLength; ++ii)
                {
                    signal_E1[ii] = static_cast<float>(signal_E1_int[ii]);
                }
        }

    if (sampling_freq != samplesPerChip * codeFreqBasis)
        {
            std::vector<float> resampled_signal(samplesPerCode);

            resampler(signal_E1, resampled_signal, static_cast<float>(samplesPerChip * codeFreqBasis), sampling_freq);  // resamples code to fs

            signal_E1 = std::move(resampled_signal);
        }

    if (galileo_signal.rfind("1C") != std::string::npos && galileo_signal.length() >= 2 && secondary_flag)
        {
            std::vector<float> signal_E1C_secondary(static_cast<int32_t>(GALILEO_E1_C_SECONDARY_CODE_LENGTH) * samplesPerCode);
            for (uint32_t i = 0; i < static_cast<uint32_t>(GALILEO_E1_C_SECONDARY_CODE_LENGTH); i++)
                {
                    for (uint32_t k = 0; k < samplesPerCode; k++)
                        {
                            signal_E1C_secondary[i * samplesPerCode + k] = signal_E1[k] * (GALILEO_E1_C_SECONDARY_CODE[i] == '0' ? 1.0F : -1.0F);
                        }
                }

            samplesPerCode *= static_cast<int32_t>(GALILEO_E1_C_SECONDARY_CODE_LENGTH);

            signal_E1 = std::move(signal_E1C_secondary);
        }

    for (uint32_t i = 0; i < samplesPerCode; i++)
        {
            dest[(i + delay) % samplesPerCode] = signal_E1[i];
        }
}


void galileo_e1_code_gen_complex_sampled(own::span<std::complex<float>> dest, const std::array<char, 3>& signal_id,
    bool cboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift,
    bool secondary_flag)
{
    constexpr int32_t codeFreqBasis = GALILEO_E1_CODE_CHIP_RATE_CPS;  // Hz
    const std::string galileo_signal = signal_id.data();
    auto samplesPerCode = static_cast<uint32_t>(static_cast<double>(sampling_freq) /
                                                (static_cast<double>(codeFreqBasis) / GALILEO_E1_B_CODE_LENGTH_CHIPS));

    if (galileo_signal.rfind("1C") != std::string::npos && galileo_signal.length() >= 2 && secondary_flag)
        {
            samplesPerCode *= static_cast<int32_t>(GALILEO_E1_C_SECONDARY_CODE_LENGTH);
        }

    std::vector<float> real_code(samplesPerCode);
    galileo_e1_code_gen_float_sampled(real_code, signal_id, cboc, prn, sampling_freq, chip_shift, secondary_flag);

    for (uint32_t ii = 0; ii < samplesPerCode; ++ii)
        {
            dest[ii] = std::complex<float>(real_code[ii], 0.0F);
        }
}


void galileo_e1_code_gen_float_sampled(own::span<float> dest, const std::array<char, 3>& signal_id,
    bool cboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift)
{
    galileo_e1_code_gen_float_sampled(dest, signal_id, cboc, prn, sampling_freq, chip_shift, false);
}


void galileo_e1_code_gen_complex_sampled(own::span<std::complex<float>> dest, const std::array<char, 3>& signal_id,
    bool cboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift)
{
    galileo_e1_code_gen_complex_sampled(dest, signal_id, cboc, prn, sampling_freq, chip_shift, false);
}
