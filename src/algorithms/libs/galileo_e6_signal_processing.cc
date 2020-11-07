/*!
 * \file galileo_e6_signal_processing.cc
 * \brief This library implements various functions for Galileo E6 signals such
 * as replica code generation
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "galileo_e6_signal_processing.h"
#include "Galileo_E6.h"
#include "gnss_signal_processing.h"
#include <iostream>
#include <vector>

void galileo_e6_b_code_gen_complex_primary(own::span<std::complex<float>> _dest,
    int32_t _prn)
{
    const uint32_t prn = _prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }

    for (size_t i = 0; i < GALILEO_E6_B_PRIMARY_CODE_STR_LENGTH - 1; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_B_PRIMARY_CODE[prn][i]);
            _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
            _dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
            _dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
            index = index + 4;
        }
    // last bit is filled up with a zero
    hex_to_binary_converter(a, GALILEO_E6_B_PRIMARY_CODE[prn][GALILEO_E6_B_PRIMARY_CODE_STR_LENGTH - 1]);
    _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
    _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
    _dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
}


void galileo_e6_b_code_gen_float_primary(own::span<float> _dest, int32_t _prn)
{
    const uint32_t prn = _prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }

    for (size_t i = 0; i < GALILEO_E6_B_PRIMARY_CODE_STR_LENGTH - 1; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_B_PRIMARY_CODE[prn][i]);
            _dest[index] = static_cast<float>(a[0]);
            _dest[index + 1] = static_cast<float>(a[1]);
            _dest[index + 2] = static_cast<float>(a[2]);
            _dest[index + 3] = static_cast<float>(a[3]);
            index = index + 4;
        }
    // last bit is filled up with a zero
    hex_to_binary_converter(a, GALILEO_E6_B_PRIMARY_CODE[prn][GALILEO_E6_B_PRIMARY_CODE_STR_LENGTH - 1]);
    _dest[index] = static_cast<float>(a[0]);
    _dest[index + 1] = static_cast<float>(a[1]);
    _dest[index + 2] = static_cast<float>(a[2]);
}


void galileo_e6_b_code_gen_complex_sampled(own::span<std::complex<float>> _dest,
    uint32_t _prn,
    int32_t _fs,
    uint32_t _chip_shift)
{
    constexpr uint32_t _codeLength = GALILEO_E6_B_CODE_LENGTH_CHIPS;
    constexpr int32_t _codeFreqBasis = GALILEO_E6_B_CODE_CHIP_RATE_CPS;

    const auto _samplesPerCode = static_cast<uint32_t>(static_cast<double>(_fs) / (static_cast<double>(_codeFreqBasis) / static_cast<double>(_codeLength)));
    const uint32_t delay = ((_codeLength - _chip_shift) % _codeLength) * _samplesPerCode / _codeLength;

    std::vector<std::complex<float>> _code(_codeLength);
    galileo_e6_b_code_gen_complex_primary(_code, _prn);

    if (_fs != _codeFreqBasis)
        {
            std::vector<std::complex<float>> _resampled_signal(_samplesPerCode);
            resampler(_code, _resampled_signal, _codeFreqBasis, _fs);  // resamples code to fs
            _code = std::move(_resampled_signal);
        }

    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            _dest[(i + delay) % _samplesPerCode] = _code[i];
        }
}


void galileo_e6_c_code_gen_complex_primary(own::span<std::complex<float>> _dest,
    int32_t _prn)
{
    const uint32_t prn = _prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }
    for (size_t i = 0; i < GALILEO_E6_C_PRIMARY_CODE_STR_LENGTH - 1; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_C_PRIMARY_CODE[prn][i]);
            _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
            _dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
            _dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
            index = index + 4;
        }
    // last bit is filled up with a zero
    hex_to_binary_converter(a, GALILEO_E6_C_PRIMARY_CODE[prn][GALILEO_E6_C_PRIMARY_CODE_STR_LENGTH - 1]);
    _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
    _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
    _dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
}


void galileo_e6_c_code_gen_float_primary(own::span<float> _dest, int32_t _prn)
{
    const uint32_t prn = _prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }
    for (size_t i = 0; i < GALILEO_E6_C_PRIMARY_CODE_STR_LENGTH - 1; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_C_PRIMARY_CODE[prn][i]);
            _dest[index] = static_cast<float>(a[0]);
            _dest[index + 1] = static_cast<float>(a[1]);
            _dest[index + 2] = static_cast<float>(a[2]);
            _dest[index + 3] = static_cast<float>(a[3]);
            index = index + 4;
        }
    // last bit is filled up with a zero
    hex_to_binary_converter(a, GALILEO_E6_C_PRIMARY_CODE[prn][GALILEO_E6_C_PRIMARY_CODE_STR_LENGTH - 1]);
    _dest[index] = static_cast<float>(a[0]);
    _dest[index + 1] = static_cast<float>(a[1]);
    _dest[index + 2] = static_cast<float>(a[2]);
}


void galileo_e6_c_code_gen_complex_sampled(own::span<std::complex<float>> _dest,
    uint32_t _prn,
    int32_t _fs,
    uint32_t _chip_shift)
{
    constexpr uint32_t _codeLength = GALILEO_E6_C_CODE_LENGTH_CHIPS;
    constexpr int32_t _codeFreqBasis = GALILEO_E6_C_CODE_CHIP_RATE_CPS;

    const auto _samplesPerCode = static_cast<uint32_t>(static_cast<double>(_fs) / (static_cast<double>(_codeFreqBasis) / static_cast<double>(_codeLength)));
    const uint32_t delay = ((_codeLength - _chip_shift) % _codeLength) * _samplesPerCode / _codeLength;

    std::vector<std::complex<float>> _code(_codeLength);
    galileo_e6_c_code_gen_complex_primary(_code, _prn);

    if (_fs != _codeFreqBasis)
        {
            std::vector<std::complex<float>> _resampled_signal(_samplesPerCode);
            resampler(_code, _resampled_signal, _codeFreqBasis, _fs);  // resamples code to fs
            _code = std::move(_resampled_signal);
        }

    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            _dest[(i + delay) % _samplesPerCode] = _code[i];
        }
}


void galileo_e6_c_secondary_code_gen_complex(own::span<std::complex<float>> _dest,
    int32_t _prn)
{
    const uint32_t prn = _prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }
    for (size_t i = 0; i < GALILEO_E6_C_SECONDARY_CODE_STR_LENGTH; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_C_SECONDARY_CODE[prn][i]);
            _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
            _dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
            _dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
            index = index + 4;
        }
}


void galileo_e6_c_secondary_code_gen_float(own::span<float> _dest,
    int32_t _prn)
{
    const uint32_t prn = _prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }
    for (size_t i = 0; i < GALILEO_E6_C_SECONDARY_CODE_STR_LENGTH; i++)
        {
            hex_to_binary_converter(a, GALILEO_E6_C_SECONDARY_CODE[prn][i]);
            _dest[index] = static_cast<float>(a[0]);
            _dest[index + 1] = static_cast<float>(a[1]);
            _dest[index + 2] = static_cast<float>(a[2]);
            _dest[index + 3] = static_cast<float>(a[3]);
            index = index + 4;
        }
}


std::string galileo_e6_c_secondary_code(int32_t _prn)
{
    std::string dest(static_cast<size_t>(GALILEO_E6_C_SECONDARY_CODE_LENGTH_CHIPS), '0');
    const uint32_t prn = _prn - 1;
    uint32_t index = 0;
    for (size_t i = 0; i < GALILEO_E6_C_SECONDARY_CODE_STR_LENGTH; i++)
        {
            std::string aux = hex_to_binary_string(GALILEO_E6_C_SECONDARY_CODE[prn][i]);
            dest[index] = aux[0];
            dest[index + 1] = aux[1];
            dest[index + 2] = aux[2];
            dest[index + 3] = aux[3];
            index = index + 4;
        }

    return dest;
}
