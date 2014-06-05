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
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
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

#include "galileo_e5_signal_processing.h"

void galileo_e5_a_code_gen_complex(std::complex<float>* _dest, signed int _prn, char _Signal[3])
{
	std::string _galileo_signal = _Signal;
	unsigned int prn=_prn-1;
	unsigned int index=0;
	//int _code_int[(int)Galileo_E5a_CODE_LENGTH_CHIPS];
	int a[4];

	if ((_prn < 1) || (_prn > 50))
	        {
	            return;
	        }
	if (_galileo_signal.rfind("5Q") != std::string::npos && _galileo_signal.length() >= 2)
	    {
            for (size_t i = 0; i < Galileo_E5a_Q_PRIMARY_CODE[prn].length(); i++)
                {
//                hex_to_binary_converter(&_dest[index],
//                		Galileo_E5a_Q_PRIMARY_CODE[prn].at(i));
//                hex_to_binary_converter(&_code_int[index],
//                		Galileo_E5a_Q_PRIMARY_CODE[prn].at(i));
                    hex_to_binary_converter(&a[0],
                    		Galileo_E5a_Q_PRIMARY_CODE[prn].at(i));
                    _dest[index]=std::complex<float>(float(a[0]),0.0);
                    _dest[index+1]=std::complex<float>(float(a[1]),0.0);
                    _dest[index+2]=std::complex<float>(float(a[2]),0.0);
                    _dest[index+3]=std::complex<float>(float(a[3]),0.0);
                    index = index + 4;
                }
	    }
	else if (_galileo_signal.rfind("5I") != std::string::npos && _galileo_signal.length() >= 2)
	    {
            for (size_t i = 0; i < Galileo_E5a_I_PRIMARY_CODE[prn].length(); i++)
                {
//                hex_to_binary_converter(&_code_int[index],
//                		Galileo_E5a_I_PRIMARY_CODE[prn].at(i));
            	    hex_to_binary_converter(&a[0],
            	           Galileo_E5a_I_PRIMARY_CODE[prn].at(i));
            	    _dest[index]=std::complex<float>(float(a[0]),0.0);
            	    _dest[index+1]=std::complex<float>(float(a[1]),0.0);
            	    _dest[index+2]=std::complex<float>(float(a[2]),0.0);
            	    _dest[index+3]=std::complex<float>(float(a[3]),0.0);
                    index = index + 4;
                }
	    }
}

void galileo_e5_a_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
		unsigned int _prn, signed int _fs, unsigned int _chip_shift,
        bool _secondary_flag)
{
    // This function is based on the GNU software GPS for MATLAB in the Kay Borre book

    std::string _galileo_signal = _Signal;
    signed int _samplesPerCode;
    const int _codeFreqBasis = Galileo_E5a_CODE_CHIP_RATE_HZ; //Hz
    unsigned int _codeLength = Galileo_E5a_CODE_LENGTH_CHIPS;
    std::complex<float>* _code;
    _code=new std::complex<float>[_codeLength];
    std::complex<float> primary_code_E5a_chips[(int)Galileo_E5a_CODE_LENGTH_CHIPS];
    _samplesPerCode = round(_fs / (_codeFreqBasis / _codeLength));
    const unsigned int delay = (((int)Galileo_E5a_CODE_LENGTH_CHIPS - _chip_shift)
                                % (int)Galileo_E5a_CODE_LENGTH_CHIPS)
                                * _samplesPerCode / Galileo_E5a_CODE_LENGTH_CHIPS;

    galileo_e5_a_code_gen_complex(_code , _prn , _Signal);

    if (_fs != _codeFreqBasis)
        {
        	std::complex<float>* _resampled_signal = new std::complex<float>[_codeLength];
        	resampler(_code, _resampled_signal, _codeFreqBasis, _fs,
        	                    _codeLength, _samplesPerCode); //resamples code to fs
        	delete[] _code;
        	_code = _resampled_signal;
        }
    // TODO secondary code generated here??
    for (unsigned int i = 0; i < _samplesPerCode; i++)
        {
            _dest[(i+delay)%_samplesPerCode] = _code[i];
        }

    delete[] _code;

}
