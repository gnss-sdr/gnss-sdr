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
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "galileo_e5_signal_processing.h"
#include "Galileo_E5a.h"
#include "gnss_signal_processing.h"
#include <gnuradio/gr_complex.h>


void galileo_e5_a_code_gen_complex_primary(std::complex<float>* _dest, signed int _prn, char _Signal[3])
{
    unsigned int prn = _prn - 1;
    unsigned int index = 0;
    int a[4];
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }
    if (_Signal[0] == '5' && _Signal[1] == 'Q')
        {
            for (size_t i = 0; i < Galileo_E5a_Q_PRIMARY_CODE[prn].length() - 1; i++)
                {
                    hex_to_binary_converter(a, Galileo_E5a_Q_PRIMARY_CODE[prn].at(i));
                    _dest[index] = std::complex<float>(0.0, float(a[0]));
                    _dest[index + 1] = std::complex<float>(0.0, float(a[1]));
                    _dest[index + 2] = std::complex<float>(0.0, float(a[2]));
                    _dest[index + 3] = std::complex<float>(0.0, float(a[3]));
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, Galileo_E5a_Q_PRIMARY_CODE[prn].at(Galileo_E5a_Q_PRIMARY_CODE[prn].length() - 1));
            _dest[index] = std::complex<float>(float(0.0), a[0]);
            _dest[index + 1] = std::complex<float>(float(0.0), a[1]);
        }
    else if (_Signal[0] == '5' && _Signal[1] == 'I')
        {
            for (size_t i = 0; i < Galileo_E5a_I_PRIMARY_CODE[prn].length() - 1; i++)
                {
                    hex_to_binary_converter(a, Galileo_E5a_I_PRIMARY_CODE[prn].at(i));
                    _dest[index] = std::complex<float>(float(a[0]), 0.0);
                    _dest[index + 1] = std::complex<float>(float(a[1]), 0.0);
                    _dest[index + 2] = std::complex<float>(float(a[2]), 0.0);
                    _dest[index + 3] = std::complex<float>(float(a[3]), 0.0);
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, Galileo_E5a_I_PRIMARY_CODE[prn].at(Galileo_E5a_I_PRIMARY_CODE[prn].length() - 1));
            _dest[index] = std::complex<float>(float(a[0]), 0.0);
            _dest[index + 1] = std::complex<float>(float(a[1]), 0.0);
        }
    else if (_Signal[0] == '5' && _Signal[1] == 'X')
        {
            int b[4];
            for (size_t i = 0; i < Galileo_E5a_I_PRIMARY_CODE[prn].length() - 1; i++)
                {
                    hex_to_binary_converter(a, Galileo_E5a_I_PRIMARY_CODE[prn].at(i));
                    hex_to_binary_converter(b, Galileo_E5a_Q_PRIMARY_CODE[prn].at(i));
                    _dest[index] = std::complex<float>(float(a[0]), float(b[0]));
                    _dest[index + 1] = std::complex<float>(float(a[1]), float(b[1]));
                    _dest[index + 2] = std::complex<float>(float(a[2]), float(b[2]));
                    _dest[index + 3] = std::complex<float>(float(a[3]), float(b[3]));
                    index = index + 4;
                }
            // last 2 bits are filled up zeros
            hex_to_binary_converter(a, Galileo_E5a_I_PRIMARY_CODE[prn].at(Galileo_E5a_I_PRIMARY_CODE[prn].length() - 1));
            hex_to_binary_converter(b, Galileo_E5a_Q_PRIMARY_CODE[prn].at(Galileo_E5a_Q_PRIMARY_CODE[prn].length() - 1));
            _dest[index] = std::complex<float>(float(a[0]), float(b[0]));
            _dest[index + 1] = std::complex<float>(float(a[1]), float(b[1]));
        }
}

void galileo_e5_a_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
    unsigned int _prn, signed int _fs, unsigned int _chip_shift)
{
    unsigned int _samplesPerCode;
    unsigned int delay;
    const unsigned int _codeLength = Galileo_E5a_CODE_LENGTH_CHIPS;
    const int _codeFreqBasis = Galileo_E5a_CODE_CHIP_RATE_HZ;

    std::complex<float>* _code = new std::complex<float>[_codeLength]();

    galileo_e5_a_code_gen_complex_primary(_code, _prn, _Signal);

    _samplesPerCode = static_cast<unsigned int>(static_cast<double>(_fs) / (static_cast<double>(_codeFreqBasis) / static_cast<double>(_codeLength)));

    delay = ((_codeLength - _chip_shift) % _codeLength) * _samplesPerCode / _codeLength;

    if (_fs != _codeFreqBasis)
        {
            std::complex<float>* _resampled_signal;
            if (posix_memalign((void**)&_resampled_signal, 16, _samplesPerCode * sizeof(gr_complex)) == 0)
                {
                };
            resampler(_code, _resampled_signal, _codeFreqBasis, _fs, _codeLength, _samplesPerCode);  //resamples code to fs
            delete[] _code;
            _code = _resampled_signal;
        }

    for (unsigned int i = 0; i < _samplesPerCode; i++)
        {
            _dest[(i + delay) % _samplesPerCode] = _code[i];
        }
    if (_fs != _codeFreqBasis)
        {
            free(_code);
        }
    else
        {
            delete[] _code;
        }
}
