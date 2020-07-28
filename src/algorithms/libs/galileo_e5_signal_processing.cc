/*!
 * \file galileo_e5_signal_processing.cc
 * \brief This library implements various functions for Galileo E5 signals such
 * as replica code generation
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \author Piyush Gupta, 2020. piyush04111999@gmail.com
 * \note Code added as part of GSoc 2020 Program.
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

#include "galileo_e5_signal_processing.h"
#include "Galileo_E5a.h"
#include "Galileo_E5b.h"
#include "gnss_signal_processing.h"
#include <gnuradio/gr_complex.h>
#include <memory>
#include <utility>
#include <vector>


void galileo_e5_a_code_gen_complex_primary(own::span<std::complex<float>> _dest,
    int32_t _prn,
    const std::array<char, 3>& _Signal)
{
    const uint32_t prn = _prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }
    if (_Signal[0] == '5' && _Signal[1] == 'Q')
        {
            for (size_t i = 0; i < GALILEO_E5A_Q_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5A_Q_PRIMARY_CODE[prn][i]);
                    _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
                    _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
                    _dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
                    _dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5A_Q_PRIMARY_CODE[prn][GALILEO_E5A_Q_PRIMARY_CODE_STR_LENGTH - 1]);
            _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
        }
    else if (_Signal[0] == '5' && _Signal[1] == 'I')
        {
            for (size_t i = 0; i < GALILEO_E5A_I_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn][i]);
                    _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
                    _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
                    _dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
                    _dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn][GALILEO_E5A_I_PRIMARY_CODE_STR_LENGTH - 1]);
            _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
        }
    else if (_Signal[0] == '5' && _Signal[1] == 'X')
        {
            std::array<int32_t, 4> b{};
            for (size_t i = 0; i < GALILEO_E5A_I_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn][i]);
                    hex_to_binary_converter(b, GALILEO_E5A_Q_PRIMARY_CODE[prn][i]);
                    _dest[index] = std::complex<float>(static_cast<float>(a[0]), static_cast<float>(b[0]));
                    _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), static_cast<float>(b[1]));
                    _dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), static_cast<float>(b[2]));
                    _dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), static_cast<float>(b[3]));
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn][GALILEO_E5A_I_PRIMARY_CODE_STR_LENGTH - 1]);
            hex_to_binary_converter(b, GALILEO_E5A_Q_PRIMARY_CODE[prn][GALILEO_E5A_Q_PRIMARY_CODE_STR_LENGTH - 1]);
            _dest[index] = std::complex<float>(static_cast<float>(a[0]), static_cast<float>(b[0]));
            _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), static_cast<float>(b[1]));
        }
}


void galileo_e5_a_code_gen_complex_sampled(own::span<std::complex<float>> _dest,
    uint32_t _prn,
    const std::array<char, 3>& _Signal,
    int32_t _fs,
    uint32_t _chip_shift)
{
    constexpr uint32_t _codeLength = GALILEO_E5A_CODE_LENGTH_CHIPS;
    constexpr int32_t _codeFreqBasis = GALILEO_E5A_CODE_CHIP_RATE_CPS;

    const auto _samplesPerCode = static_cast<uint32_t>(static_cast<double>(_fs) / (static_cast<double>(_codeFreqBasis) / static_cast<double>(_codeLength)));
    const uint32_t delay = ((_codeLength - _chip_shift) % _codeLength) * _samplesPerCode / _codeLength;

    std::vector<std::complex<float>> _code(_codeLength);
    galileo_e5_a_code_gen_complex_primary(_code, _prn, _Signal);

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


void galileo_e5_b_code_gen_complex_primary(own::span<std::complex<float>> _dest,
    int32_t _prn,
    const std::array<char, 3>& _Signal)
{
    const uint32_t prn = _prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }
    if (_Signal[0] == '7' && _Signal[1] == 'Q')
        {
            for (size_t i = 0; i < GALILEO_E5B_Q_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5B_Q_PRIMARY_CODE[prn][i]);
                    _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
                    _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
                    _dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
                    _dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5B_Q_PRIMARY_CODE[prn][GALILEO_E5B_Q_PRIMARY_CODE_STR_LENGTH - 1]);
            _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
        }
    else if (_Signal[0] == '7' && _Signal[1] == 'I')
        {
            for (size_t i = 0; i < GALILEO_E5B_I_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5B_I_PRIMARY_CODE[prn][i]);
                    _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
                    _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
                    _dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), 0.0);
                    _dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), 0.0);
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5B_I_PRIMARY_CODE[prn][GALILEO_E5B_I_PRIMARY_CODE_STR_LENGTH - 1]);
            _dest[index] = std::complex<float>(static_cast<float>(a[0]), 0.0);
            _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), 0.0);
        }
    else if (_Signal[0] == '7' && _Signal[1] == 'X')
        {
            std::array<int32_t, 4> b{};
            for (size_t i = 0; i < GALILEO_E5B_I_PRIMARY_CODE_STR_LENGTH - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5B_I_PRIMARY_CODE[prn][i]);
                    hex_to_binary_converter(b, GALILEO_E5B_Q_PRIMARY_CODE[prn][i]);
                    _dest[index] = std::complex<float>(static_cast<float>(a[0]), static_cast<float>(b[0]));
                    _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), static_cast<float>(b[1]));
                    _dest[index + 2] = std::complex<float>(static_cast<float>(a[2]), static_cast<float>(b[2]));
                    _dest[index + 3] = std::complex<float>(static_cast<float>(a[3]), static_cast<float>(b[3]));
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5B_I_PRIMARY_CODE[prn][GALILEO_E5B_I_PRIMARY_CODE_STR_LENGTH - 1]);
            hex_to_binary_converter(b, GALILEO_E5B_Q_PRIMARY_CODE[prn][GALILEO_E5B_Q_PRIMARY_CODE_STR_LENGTH - 1]);
            _dest[index] = std::complex<float>(static_cast<float>(a[0]), static_cast<float>(b[0]));
            _dest[index + 1] = std::complex<float>(static_cast<float>(a[1]), static_cast<float>(b[1]));
        }
}


void galileo_e5_b_code_gen_complex_sampled(own::span<std::complex<float>> _dest,
    uint32_t _prn,
    const std::array<char, 3>& _Signal,
    int32_t _fs,
    uint32_t _chip_shift)
{
    constexpr uint32_t _codeLength = GALILEO_E5B_CODE_LENGTH_CHIPS;
    constexpr int32_t _codeFreqBasis = GALILEO_E5B_CODE_CHIP_RATE_CPS;

    const auto _samplesPerCode = static_cast<uint32_t>(static_cast<double>(_fs) / (static_cast<double>(_codeFreqBasis) / static_cast<double>(_codeLength)));
    const uint32_t delay = ((_codeLength - _chip_shift) % _codeLength) * _samplesPerCode / _codeLength;

    std::vector<std::complex<float>> _code(_codeLength);
    galileo_e5_b_code_gen_complex_primary(_code, _prn, _Signal);

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
