/*!
 * \file galileo_e1_signal_processing.cc
 * \brief This library implements various functions for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#include "galileo_e1_signal_processing.h"
#include "Galileo_E1.h"
#include "gnss_signal_processing.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <string>


void galileo_e1_code_gen_int(int* _dest, char _Signal[3], int32_t _prn)
{
    std::string _galileo_signal = _Signal;
    int32_t prn = _prn - 1;
    int32_t index = 0;

    // A simple error check
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }

    if (_galileo_signal.rfind("1B") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (size_t i = 0; i < Galileo_E1_B_PRIMARY_CODE[prn].length(); i++)
                {
                    hex_to_binary_converter(&_dest[index], Galileo_E1_B_PRIMARY_CODE[prn].at(i));
                    index += 4;
                }
        }
    else if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (size_t i = 0; i < Galileo_E1_C_PRIMARY_CODE[prn].length(); i++)
                {
                    hex_to_binary_converter(&_dest[index], Galileo_E1_C_PRIMARY_CODE[prn].at(i));
                    index += 4;
                }
        }
}


void galileo_e1_sinboc_11_gen_int(int* _dest, int* _prn, uint32_t _length_out)
{
    const uint32_t _length_in = Galileo_E1_B_CODE_LENGTH_CHIPS;
    uint32_t _period = static_cast<uint32_t>(_length_out / _length_in);
    for (uint32_t i = 0; i < _length_in; i++)
        {
            for (uint32_t j = 0; j < (_period / 2); j++)
                {
                    _dest[i * _period + j] = _prn[i];
                }
            for (uint32_t j = (_period / 2); j < _period; j++)
                {
                    _dest[i * _period + j] = -_prn[i];
                }
        }
}


void galileo_e1_sinboc_61_gen_int(int* _dest, int* _prn, uint32_t _length_out)
{
    const uint32_t _length_in = Galileo_E1_B_CODE_LENGTH_CHIPS;
    uint32_t _period = static_cast<uint32_t>(_length_out / _length_in);

    for (uint32_t i = 0; i < _length_in; i++)
        {
            for (uint32_t j = 0; j < _period; j += 2)
                {
                    _dest[i * _period + j] = _prn[i];
                }
            for (uint32_t j = 1; j < _period; j += 2)
                {
                    _dest[i * _period + j] = -_prn[i];
                }
        }
}


void galileo_e1_code_gen_sinboc11_float(float* _dest, char _Signal[3], uint32_t _prn)
{
    std::string _galileo_signal = _Signal;
    const uint32_t _codeLength = static_cast<uint32_t>(Galileo_E1_B_CODE_LENGTH_CHIPS);
    int32_t primary_code_E1_chips[4092];                            // _codeLength not accepted by Clang
    galileo_e1_code_gen_int(primary_code_E1_chips, _Signal, _prn);  //generate Galileo E1 code, 1 sample per chip
    for (uint32_t i = 0; i < _codeLength; i++)
        {
            _dest[2 * i] = static_cast<float>(primary_code_E1_chips[i]);
            _dest[2 * i + 1] = -_dest[2 * i];
        }
}


void galileo_e1_gen_float(float* _dest, int* _prn, char _Signal[3])
{
    std::string _galileo_signal = _Signal;
    const uint32_t _codeLength = 12 * Galileo_E1_B_CODE_LENGTH_CHIPS;
    const float alpha = sqrt(10.0 / 11.0);
    const float beta = sqrt(1.0 / 11.0);

    int32_t sinboc_11[12 * 4092] = {0};  //  _codeLength not accepted by Clang
    int32_t sinboc_61[12 * 4092] = {0};

    galileo_e1_sinboc_11_gen_int(sinboc_11, _prn, _codeLength);  //generate sinboc(1,1) 12 samples per chip
    galileo_e1_sinboc_61_gen_int(sinboc_61, _prn, _codeLength);  //generate sinboc(6,1) 12 samples per chip

    if (_galileo_signal.rfind("1B") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (uint32_t i = 0; i < _codeLength; i++)
                {
                    _dest[i] = alpha * static_cast<float>(sinboc_11[i]) +
                               beta * static_cast<float>(sinboc_61[i]);
                }
        }
    else if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (uint32_t i = 0; i < _codeLength; i++)
                {
                    _dest[i] = alpha * static_cast<float>(sinboc_11[i]) -
                               beta * static_cast<float>(sinboc_61[i]);
                }
        }
}


void galileo_e1_code_gen_float_sampled(float* _dest, char _Signal[3],
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift,
    bool _secondary_flag)
{
    // This function is based on the GNU software GPS for MATLAB in Kay Borre's book
    std::string _galileo_signal = _Signal;
    uint32_t _samplesPerCode;
    const int32_t _codeFreqBasis = Galileo_E1_CODE_CHIP_RATE_HZ;  // Hz
    uint32_t _codeLength = static_cast<uint32_t>(Galileo_E1_B_CODE_LENGTH_CHIPS);
    int32_t* primary_code_E1_chips = static_cast<int32_t*>(volk_gnsssdr_malloc(static_cast<uint32_t>(Galileo_E1_B_CODE_LENGTH_CHIPS) * sizeof(int32_t), volk_gnsssdr_get_alignment()));

    _samplesPerCode = static_cast<uint32_t>(static_cast<double>(_fs) / (static_cast<double>(_codeFreqBasis) / static_cast<double>(_codeLength)));
    const int32_t _samplesPerChip = (_cboc == true) ? 12 : 2;

    const uint32_t delay = ((static_cast<int32_t>(Galileo_E1_B_CODE_LENGTH_CHIPS) - _chip_shift) % static_cast<int32_t>(Galileo_E1_B_CODE_LENGTH_CHIPS)) * _samplesPerCode / Galileo_E1_B_CODE_LENGTH_CHIPS;

    galileo_e1_code_gen_int(primary_code_E1_chips, _Signal, _prn);  // generate Galileo E1 code, 1 sample per chip

    float* _signal_E1;

    _codeLength = _samplesPerChip * Galileo_E1_B_CODE_LENGTH_CHIPS;
    _signal_E1 = new float[_codeLength];

    if (_cboc == true)
        {
            galileo_e1_gen_float(_signal_E1, primary_code_E1_chips, _Signal);  // generate cboc 12 samples per chip
        }
    else
        {
            int32_t* _signal_E1_int = static_cast<int32_t*>(volk_gnsssdr_malloc(_codeLength * sizeof(int32_t), volk_gnsssdr_get_alignment()));
            galileo_e1_sinboc_11_gen_int(_signal_E1_int, primary_code_E1_chips, _codeLength);  // generate sinboc(1,1) 2 samples per chip

            for (uint32_t ii = 0; ii < _codeLength; ++ii)
                {
                    _signal_E1[ii] = static_cast<float>(_signal_E1_int[ii]);
                }
            volk_gnsssdr_free(_signal_E1_int);
        }

    if (_fs != _samplesPerChip * _codeFreqBasis)
        {
            float* _resampled_signal = new float[_samplesPerCode];

            resampler(_signal_E1, _resampled_signal, _samplesPerChip * _codeFreqBasis, _fs,
                _codeLength, _samplesPerCode);  // resamples code to fs

            delete[] _signal_E1;
            _signal_E1 = _resampled_signal;
        }

    if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2 && _secondary_flag)
        {
            float* _signal_E1C_secondary = new float[static_cast<int32_t>(Galileo_E1_C_SECONDARY_CODE_LENGTH) * _samplesPerCode];

            for (uint32_t i = 0; i < static_cast<uint32_t>(Galileo_E1_C_SECONDARY_CODE_LENGTH); i++)
                {
                    for (unsigned k = 0; k < _samplesPerCode; k++)
                        {
                            _signal_E1C_secondary[i * _samplesPerCode + k] = _signal_E1[k] * (Galileo_E1_C_SECONDARY_CODE.at(i) == '0' ? 1.0f : -1.0f);
                        }
                }

            _samplesPerCode *= static_cast<int32_t>(Galileo_E1_C_SECONDARY_CODE_LENGTH);

            delete[] _signal_E1;
            _signal_E1 = _signal_E1C_secondary;
        }

    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            _dest[(i + delay) % _samplesPerCode] = _signal_E1[i];
        }

    delete[] _signal_E1;
    volk_gnsssdr_free(primary_code_E1_chips);
}


void galileo_e1_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift,
    bool _secondary_flag)
{
    std::string _galileo_signal = _Signal;
    const int32_t _codeFreqBasis = Galileo_E1_CODE_CHIP_RATE_HZ;  // Hz
    uint32_t _samplesPerCode = static_cast<uint32_t>(static_cast<double>(_fs) /
                                                     (static_cast<double>(_codeFreqBasis) / static_cast<double>(Galileo_E1_B_CODE_LENGTH_CHIPS)));

    if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2 && _secondary_flag)
        {
            _samplesPerCode *= static_cast<int32_t>(Galileo_E1_C_SECONDARY_CODE_LENGTH);
        }

    float* real_code = static_cast<float*>(volk_gnsssdr_malloc(_samplesPerCode * sizeof(float), volk_gnsssdr_get_alignment()));

    galileo_e1_code_gen_float_sampled(real_code, _Signal, _cboc, _prn, _fs, _chip_shift, _secondary_flag);

    for (uint32_t ii = 0; ii < _samplesPerCode; ++ii)
        {
            _dest[ii] = std::complex<float>(real_code[ii], 0.0f);
        }
    volk_gnsssdr_free(real_code);
}


void galileo_e1_code_gen_float_sampled(float* _dest, char _Signal[3],
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift)
{
    galileo_e1_code_gen_float_sampled(_dest, _Signal, _cboc, _prn, _fs, _chip_shift, false);
}


void galileo_e1_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift)
{
    galileo_e1_code_gen_complex_sampled(_dest, _Signal, _cboc, _prn, _fs, _chip_shift, false);
}
