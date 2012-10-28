/*!
 * \file galileo_e1_signal_processing.cc
 * \brief This library implements various functions for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

void
galileo_e1_code_gen_int(int* _dest, char _Signal[3], signed int _prn,
        unsigned int _chip_shift)
{
    std::string _galileo_signal = _Signal;
    signed int prn = _prn - 1;
    int index = 0;
    //int* dest = _dest;

    /* A simple error check */
    if ((_prn < 1) || (_prn > 50))
        {
            return;
        }

    if (_galileo_signal.rfind("1B") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (size_t i = 0; i < Galileo_E1_B_PRIMARY_CODE[prn].length(); i++)
                {
                    hex_to_binary_converter(&_dest[index],
                            Galileo_E1_B_PRIMARY_CODE[prn].at(i));
                    index = index +4;
                }

        }
    else if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (size_t i = 0; i < Galileo_E1_C_PRIMARY_CODE[prn].length(); i++)
                {
                    hex_to_binary_converter(&_dest[index],
                            Galileo_E1_C_PRIMARY_CODE[prn].at(i));
                    index = index +4;
                }
        }
    else
        {
            return;
        }
}



void
galileo_e1_sinboc_11_gen(std::complex<float>* _dest, int* _prn,
        unsigned int _length_out)
{
    const unsigned int _length_in = Galileo_E1_B_CODE_LENGTH_CHIPS;
    unsigned int _period = (unsigned int) (_length_out / _length_in);
    for (unsigned int i = 0; i < _length_in; i++)
        {
            for (unsigned int j = 0; j < (_period / 2); j++)
                _dest[i * _period + j] = std::complex<float>((float) _prn[i],
                        0.0);

            for (unsigned int j = (_period / 2); j < _period; j++)
                _dest[i * _period + j] = std::complex<float>((float) (-_prn[i]),
                        0.0);
        }
}



void
galileo_e1_sinboc_61_gen(std::complex<float>* _dest, int* _prn,
        unsigned int _length_out)
{
    const unsigned int _length_in = Galileo_E1_B_CODE_LENGTH_CHIPS;
    unsigned int _period = (unsigned int) (_length_out / _length_in);

    for (unsigned int i = 0; i < _length_in; i++)
        {
            for (unsigned int j = 0; j < _period; j += 2)
                _dest[i * _period + j] = std::complex<float>((float) _prn[i],
                        0.0);
            for (unsigned int j = 1; j < _period; j += 2)
                _dest[i * _period + j] = std::complex<float>((float) (-_prn[i]),
                        0.0);
        }
}



void
galileo_e1_gen(std::complex<float>* _dest, int* _prn, char _Signal[3])
{
    std::string _galileo_signal = _Signal;
    const unsigned int _codeLength = 12 * Galileo_E1_B_CODE_LENGTH_CHIPS;
    const float alpha = sqrt(10.0 / 11.0);
    const float beta = sqrt(1.0 / 11.0);

    std::complex<float> sinboc_11[_codeLength];
    std::complex<float> sinboc_61[_codeLength];

    galileo_e1_sinboc_11_gen(sinboc_11, _prn, _codeLength); //generate sinboc(1,1) 12 samples per chip
    galileo_e1_sinboc_61_gen(sinboc_61, _prn, _codeLength); //generate sinboc(6,1) 12 samples per chip

    if (_galileo_signal.rfind("1B") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (unsigned int i = 0; i < _codeLength; i++)
                {
                    _dest[i] = alpha * sinboc_11[i] + beta * sinboc_61[i];
                }
        }
    else if (_galileo_signal.rfind("1C") != std::string::npos && _galileo_signal.length() >= 2)
        {
            for (unsigned int i = 0; i < _codeLength; i++)
                {
                    _dest[i] = alpha * sinboc_11[i] - beta * sinboc_61[i];
                }
        }
    else
        return;
}



void
galileo_e1_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
        bool _cboc, unsigned int _prn, signed int _fs, unsigned int _chip_shift)
{
    // This function is based on the GNU software GPS for MATLAB in the Kay Borre book
    unsigned int _samplesPerCode;
    const unsigned int _codeFreqBasis = Galileo_E1_CODE_CHIP_RATE_HZ; //Hz
    unsigned int _codeLength = Galileo_E1_B_CODE_LENGTH_CHIPS;
    int primary_code_E1_chips[_codeLength];
    _samplesPerCode = round(_fs / (_codeFreqBasis / _codeLength));

    galileo_e1_code_gen_int(primary_code_E1_chips, _Signal, _prn, 0); //generate Galileo E1 code, 1 sample per chip

    if (_cboc == true)
        {
            _codeLength = 12 * Galileo_E1_B_CODE_LENGTH_CHIPS;
            if (_fs != 12 * _codeFreqBasis)
                {
                    std::complex<float> _signal_E1[_codeLength];
                    galileo_e1_gen(_signal_E1, primary_code_E1_chips, _Signal); //generate cboc 12 samples per chip
                    resampler(_signal_E1, _dest, 12 * _codeFreqBasis, _fs,
                            _codeLength, _samplesPerCode); //resamples code to fs
                }
            else
                {
                    galileo_e1_gen(_dest, primary_code_E1_chips, _Signal); //generate cboc 12 samples per chip
                }
        }
    else
        {
            //--- Find number of samples per spreading code ----------------------------
            _codeLength = 2 * Galileo_E1_B_CODE_LENGTH_CHIPS;
            if (_fs != 2 * _codeFreqBasis)
                {
                    std::complex<float> _signal_E1[_codeLength];
                    galileo_e1_sinboc_11_gen(_signal_E1, primary_code_E1_chips,
                            _codeLength); //generate sinboc(1,1) 2 samples per chip
                    resampler(_signal_E1, _dest, 2 * _codeFreqBasis, _fs,
                            _codeLength, _samplesPerCode); //resamples code to fs
                }
            else
                {
                    galileo_e1_sinboc_11_gen(_dest, primary_code_E1_chips,
                            _codeLength); //generate sinboc(1,1) 2 samples per chip                }
                }
        }
}
