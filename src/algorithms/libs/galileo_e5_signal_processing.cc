/*!
 * \file galileo_e5_signal_processing.cc
 * \brief This library implements various functions for Galileo E5 signals such
 * as replica code generation
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "galileo_e5_signal_processing.h"
#include "Galileo_E5a.h"
#include "gnss_signal_processing.h"
#include <gnuradio/gr_complex.h>
#include <memory>


void galileo_e5_a_code_gen_complex_primary(gsl::span<std::complex<float>> _dest, int32_t _prn, const std::array<char, 3>& _Signal)
{
    uint32_t prn = _prn - 1;
    uint32_t index = 0;
    std::array<int32_t, 4> a{};
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }
    if (_Signal[0] == '5' && _Signal[1] == 'Q')
        {
            for (size_t i = 0; i < GALILEO_E5A_Q_PRIMARY_CODE[prn].length() - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5A_Q_PRIMARY_CODE[prn].at(i));
                    _dest[index] = std::complex<float>(0.0, float(a[0]));
                    _dest[index + 1] = std::complex<float>(0.0, float(a[1]));
                    _dest[index + 2] = std::complex<float>(0.0, float(a[2]));
                    _dest[index + 3] = std::complex<float>(0.0, float(a[3]));
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5A_Q_PRIMARY_CODE[prn].at(GALILEO_E5A_Q_PRIMARY_CODE[prn].length() - 1));
            _dest[index] = std::complex<float>(float(0.0), a[0]);
            _dest[index + 1] = std::complex<float>(float(0.0), a[1]);
        }
    else if (_Signal[0] == '5' && _Signal[1] == 'I')
        {
            for (size_t i = 0; i < GALILEO_E5A_I_PRIMARY_CODE[prn].length() - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn].at(i));
                    _dest[index] = std::complex<float>(float(a[0]), 0.0);
                    _dest[index + 1] = std::complex<float>(float(a[1]), 0.0);
                    _dest[index + 2] = std::complex<float>(float(a[2]), 0.0);
                    _dest[index + 3] = std::complex<float>(float(a[3]), 0.0);
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn].at(GALILEO_E5A_I_PRIMARY_CODE[prn].length() - 1));
            _dest[index] = std::complex<float>(float(a[0]), 0.0);
            _dest[index + 1] = std::complex<float>(float(a[1]), 0.0);
        }
    else if (_Signal[0] == '5' && _Signal[1] == 'X')
        {
            std::array<int32_t, 4> b{};
            for (size_t i = 0; i < GALILEO_E5A_I_PRIMARY_CODE[prn].length() - 1; i++)
                {
                    hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn].at(i));
                    hex_to_binary_converter(b, GALILEO_E5A_Q_PRIMARY_CODE[prn].at(i));
                    _dest[index] = std::complex<float>(float(a[0]), float(b[0]));
                    _dest[index + 1] = std::complex<float>(float(a[1]), float(b[1]));
                    _dest[index + 2] = std::complex<float>(float(a[2]), float(b[2]));
                    _dest[index + 3] = std::complex<float>(float(a[3]), float(b[3]));
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, GALILEO_E5A_I_PRIMARY_CODE[prn].at(GALILEO_E5A_I_PRIMARY_CODE[prn].length() - 1));
            hex_to_binary_converter(b, GALILEO_E5A_Q_PRIMARY_CODE[prn].at(GALILEO_E5A_Q_PRIMARY_CODE[prn].length() - 1));
            _dest[index] = std::complex<float>(float(a[0]), float(b[0]));
            _dest[index + 1] = std::complex<float>(float(a[1]), float(b[1]));
        }
}


void galileo_e5_a_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, const std::array<char, 3>& _Signal,
    uint32_t _prn, int32_t _fs, uint32_t _chip_shift)
{
    uint32_t _samplesPerCode;
    uint32_t delay;
    const uint32_t _codeLength = GALILEO_E5A_CODE_LENGTH_CHIPS;
    const int32_t _codeFreqBasis = GALILEO_E5A_CODE_CHIP_RATE_HZ;

    std::unique_ptr<std::complex<float>> _code{new std::complex<float>[_codeLength]};
    gsl::span<std::complex<float>> _code_span(_code, _codeLength);
    galileo_e5_a_code_gen_complex_primary(_code_span, _prn, _Signal);

    _samplesPerCode = static_cast<uint32_t>(static_cast<double>(_fs) / (static_cast<double>(_codeFreqBasis) / static_cast<double>(_codeLength)));

    delay = ((_codeLength - _chip_shift) % _codeLength) * _samplesPerCode / _codeLength;

    if (_fs != _codeFreqBasis)
        {
            std::unique_ptr<std::complex<float>> _resampled_signal{new std::complex<float>[_samplesPerCode]};
            resampler(_code_span, gsl::span<std::complex<float>>(_resampled_signal, _samplesPerCode), _codeFreqBasis, _fs);  // resamples code to fs
            _code = std::move(_resampled_signal);
        }
    uint32_t size_code = _codeLength;
    if (_fs != _codeFreqBasis)
        {
            size_code = _samplesPerCode;
        }
    gsl::span<std::complex<float>> _code_span_aux(_code, size_code);
    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            _dest[(i + delay) % _samplesPerCode] = _code_span_aux[i];
        }
}
