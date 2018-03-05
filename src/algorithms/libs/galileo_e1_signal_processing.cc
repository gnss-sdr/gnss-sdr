/*!
 * \file galileo_e1_signal_processing.cc
 * \brief This library implements various functions for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "galileo_e1_signal_processing.h"
#include "Galileo_E1.h"
#include "gnss_signal_processing.h"
#include <string>


void galileo_e1_code_gen_int(int* _dest, char _Signal[3], signed int _prn)
{
    std::string _galileo_signal = _Signal;
    signed int prn = _prn - 1;
    int index = 0;

    /* A simple error check */
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }

    if (_galileo_signal.rfind("1B") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (size_t i = 0; i < Galileo_E1_B_PRIMARY_CODE[prn].length(); i++)
                {
                    hex_to_binary_converter(&_dest[index], Galileo_E1_B_PRIMARY_CODE[prn].at(i));
                    index = index + 4;
                }
        }
    else if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (size_t i = 0; i < Galileo_E1_C_PRIMARY_CODE[prn].length(); i++)
                {
                    hex_to_binary_converter(&_dest[index], Galileo_E1_C_PRIMARY_CODE[prn].at(i));
                    index = index + 4;
                }
        }
    else
        {
            return;
        }
}


void galileo_e1_sinboc_11_gen_int(int* _dest, int* _prn, unsigned int _length_out)
{
    const unsigned int _length_in = Galileo_E1_B_CODE_LENGTH_CHIPS;
    unsigned int _period = static_cast<unsigned int>(_length_out / _length_in);
    for (unsigned int i = 0; i < _length_in; i++)
        {
            for (unsigned int j = 0; j < (_period / 2); j++)
                {
                    _dest[i * _period + j] = _prn[i];
                }
            for (unsigned int j = (_period / 2); j < _period; j++)
                {
                    _dest[i * _period + j] = -_prn[i];
                }
        }
}


void galileo_e1_sinboc_61_gen_int(int* _dest, int* _prn, unsigned int _length_out)
{
    const unsigned int _length_in = Galileo_E1_B_CODE_LENGTH_CHIPS;
    unsigned int _period = static_cast<unsigned int>(_length_out / _length_in);

    for (unsigned int i = 0; i < _length_in; i++)
        {
            for (unsigned int j = 0; j < _period; j += 2)
                {
                    _dest[i * _period + j] = _prn[i];
                }
            for (unsigned int j = 1; j < _period; j += 2)
                {
                    _dest[i * _period + j] = -_prn[i];
                }
        }
}


void galileo_e1_gen_float(float* _dest, int* _prn, char _Signal[3])
{
    std::string _galileo_signal = _Signal;
    const unsigned int _codeLength = 12 * Galileo_E1_B_CODE_LENGTH_CHIPS;
    const float alpha = sqrt(10.0 / 11.0);
    const float beta = sqrt(1.0 / 11.0);

    int sinboc_11[12 * 4092];  //  _codeLength not accepted by Clang
    int sinboc_61[12 * 4092];

    galileo_e1_sinboc_11_gen_int(sinboc_11, _prn, _codeLength);  //generate sinboc(1,1) 12 samples per chip
    galileo_e1_sinboc_61_gen_int(sinboc_61, _prn, _codeLength);  //generate sinboc(6,1) 12 samples per chip

    if (_galileo_signal.rfind("1B") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (unsigned int i = 0; i < _codeLength; i++)
                {
                    _dest[i] = alpha * static_cast<float>(sinboc_11[i]) +
                               beta * static_cast<float>(sinboc_61[i]);
                }
        }
    else if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (unsigned int i = 0; i < _codeLength; i++)
                {
                    _dest[i] = alpha * static_cast<float>(sinboc_11[i]) -
                               beta * static_cast<float>(sinboc_61[i]);
                }
        }
    else
        return;
}


void galileo_e1_code_gen_float_sampled(float* _dest, char _Signal[3],
    bool _cboc, unsigned int _prn, signed int _fs, unsigned int _chip_shift,
    bool _secondary_flag)
{
    // This function is based on the GNU software GPS for MATLAB in Kay Borre's book
    std::string _galileo_signal = _Signal;
    unsigned int _samplesPerCode;
    const int _codeFreqBasis = Galileo_E1_CODE_CHIP_RATE_HZ;  //Hz
    unsigned int _codeLength = Galileo_E1_B_CODE_LENGTH_CHIPS;
    int primary_code_E1_chips[static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS)];
    _samplesPerCode = static_cast<unsigned int>(static_cast<double>(_fs) / (static_cast<double>(_codeFreqBasis) / static_cast<double>(_codeLength)));
    const int _samplesPerChip = (_cboc == true) ? 12 : 2;

    const unsigned int delay = ((static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS) - _chip_shift) % static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS)) * _samplesPerCode / Galileo_E1_B_CODE_LENGTH_CHIPS;

    galileo_e1_code_gen_int(primary_code_E1_chips, _Signal, _prn);  //generate Galileo E1 code, 1 sample per chip

    float* _signal_E1;

    _codeLength = _samplesPerChip * Galileo_E1_B_CODE_LENGTH_CHIPS;
    _signal_E1 = new float[_codeLength];

    if (_cboc == true)
        {
            galileo_e1_gen_float(_signal_E1, primary_code_E1_chips, _Signal);  //generate cboc 12 samples per chip
        }
    else
        {
            int _signal_E1_int[_codeLength];
            galileo_e1_sinboc_11_gen_int(_signal_E1_int, primary_code_E1_chips, _codeLength);  //generate sinboc(1,1) 2 samples per chip

            for (unsigned int ii = 0; ii < _codeLength; ++ii)
                {
                    _signal_E1[ii] = static_cast<float>(_signal_E1_int[ii]);
                }
        }

    if (_fs != _samplesPerChip * _codeFreqBasis)
        {
            float* _resampled_signal = new float[_samplesPerCode];
            resampler(_signal_E1, _resampled_signal, _samplesPerChip * _codeFreqBasis, _fs,
                _codeLength, _samplesPerCode);  //resamples code to fs

            delete[] _signal_E1;
            _signal_E1 = _resampled_signal;
        }

    if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2 && _secondary_flag)
        {
            float* _signal_E1C_secondary = new float[static_cast<int>(Galileo_E1_C_SECONDARY_CODE_LENGTH) * _samplesPerCode];

            for (unsigned int i = 0; i < static_cast<unsigned int>(Galileo_E1_C_SECONDARY_CODE_LENGTH); i++)
                {
                    for (unsigned k = 0; k < _samplesPerCode; k++)
                        {
                            _signal_E1C_secondary[i * _samplesPerCode + k] = _signal_E1[k] * (Galileo_E1_C_SECONDARY_CODE.at(i) == '0' ? 1.0f : -1.0f);
                        }
                }

            _samplesPerCode *= static_cast<int>(Galileo_E1_C_SECONDARY_CODE_LENGTH);

            delete[] _signal_E1;
            _signal_E1 = _signal_E1C_secondary;
        }

    for (unsigned int i = 0; i < _samplesPerCode; i++)
        {
            _dest[(i + delay) % _samplesPerCode] = _signal_E1[i];
        }

    delete[] _signal_E1;
}


void galileo_e1_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
    bool _cboc, unsigned int _prn, signed int _fs, unsigned int _chip_shift,
    bool _secondary_flag)
{
    std::string _galileo_signal = _Signal;
    const int _codeFreqBasis = Galileo_E1_CODE_CHIP_RATE_HZ;  //Hz
    unsigned int _samplesPerCode = static_cast<unsigned int>(static_cast<double>(_fs) /
                                                             (static_cast<double>(_codeFreqBasis) / static_cast<double>(Galileo_E1_B_CODE_LENGTH_CHIPS)));

    if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2 && _secondary_flag)
        {
            _samplesPerCode *= static_cast<int>(Galileo_E1_C_SECONDARY_CODE_LENGTH);
        }

    float real_code[_samplesPerCode];

    galileo_e1_code_gen_float_sampled(real_code, _Signal, _cboc, _prn, _fs, _chip_shift, _secondary_flag);

    for (unsigned int ii = 0; ii < _samplesPerCode; ++ii)
        {
            _dest[ii] = std::complex<float>(real_code[ii], 0.0f);
        }
}


void galileo_e1_code_gen_float_sampled(float* _dest, char _Signal[3],
    bool _cboc, unsigned int _prn, signed int _fs, unsigned int _chip_shift)
{
    galileo_e1_code_gen_float_sampled(_dest, _Signal, _cboc, _prn, _fs, _chip_shift, false);
}


void galileo_e1_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
    bool _cboc, unsigned int _prn, signed int _fs, unsigned int _chip_shift)
{
    galileo_e1_code_gen_complex_sampled(_dest, _Signal, _cboc, _prn, _fs, _chip_shift, false);
}
