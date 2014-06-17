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

void galileo_e5_a_code_gen_complex_primary(std::complex<float>* _dest, signed int _prn, char _Signal[3])
{
    std::string _galileo_signal = _Signal;
    unsigned int prn=_prn-1;
    unsigned int index=0;
    int a[4];
    if ((_prn < 1) || (_prn > 50))
	{
	    return;
	}
    if (_galileo_signal.rfind("5Q") != std::string::npos && _galileo_signal.length() >= 2)
	{
	    for (size_t i = 0; i < Galileo_E5a_Q_PRIMARY_CODE[prn].length()-1; i++)
		{
		    hex_to_binary_converter(a,
		                            Galileo_E5a_Q_PRIMARY_CODE[prn].at(i));
		    _dest[index]=std::complex<float>(0.0,float(a[0]));
		    _dest[index+1]=std::complex<float>(0.0,float(a[1]));
		    _dest[index+2]=std::complex<float>(0.0,float(a[2]));
		    _dest[index+3]=std::complex<float>(0.0,float(a[3]));
		    index = index + 4;
		}
	    // last 2 bits are filled up zeros
	    hex_to_binary_converter(a,
	    		 Galileo_E5a_Q_PRIMARY_CODE[prn].at(Galileo_E5a_Q_PRIMARY_CODE[prn].length()-1));
	    _dest[index]=std::complex<float>(float(0.0),a[0]);
	    _dest[index+1]=std::complex<float>(float(0.0),a[1]);
	}
    else if (_galileo_signal.rfind("5I") != std::string::npos && _galileo_signal.length() >= 2)
	{
	    for (size_t i = 0; i < Galileo_E5a_I_PRIMARY_CODE[prn].length()-1; i++)
		{
		    hex_to_binary_converter(a,
		                            Galileo_E5a_I_PRIMARY_CODE[prn].at(i));
		    _dest[index]=std::complex<float>(float(a[0]),0.0);
		    _dest[index+1]=std::complex<float>(float(a[1]),0.0);
		    _dest[index+2]=std::complex<float>(float(a[2]),0.0);
		    _dest[index+3]=std::complex<float>(float(a[3]),0.0);
		    index = index + 4;
		}
	    // last 2 bits are filled up zeros
	    hex_to_binary_converter(a,
	    		 Galileo_E5a_I_PRIMARY_CODE[prn].at(Galileo_E5a_I_PRIMARY_CODE[prn].length()-1));
	    _dest[index]=std::complex<float>(float(a[0]),0.0);
	    _dest[index+1]=std::complex<float>(float(a[1]),0.0);
	}
    else if (_galileo_signal.rfind("5X") != std::string::npos && _galileo_signal.length() >= 2)
	{
	    int b[4];
	    for (size_t i = 0; i < Galileo_E5a_I_PRIMARY_CODE[prn].length()-1; i++)
		{
		    hex_to_binary_converter(a,
		                            Galileo_E5a_I_PRIMARY_CODE[prn].at(i));
		    hex_to_binary_converter(b,
		                            Galileo_E5a_Q_PRIMARY_CODE[prn].at(i));
		    _dest[index]=std::complex<float>(float(a[0]),float(b[0]));
		    _dest[index+1]=std::complex<float>(float(a[1]),float(b[1]));
		    _dest[index+2]=std::complex<float>(float(a[2]),float(b[2]));
		    _dest[index+3]=std::complex<float>(float(a[3]),float(b[3]));
		    index = index + 4;
		}
	    // last 2 bits are filled up zeros
	    hex_to_binary_converter(a,
	    		 Galileo_E5a_I_PRIMARY_CODE[prn].at(Galileo_E5a_I_PRIMARY_CODE[prn].length()-1));
	    hex_to_binary_converter(a,
	    		 Galileo_E5a_Q_PRIMARY_CODE[prn].at(Galileo_E5a_Q_PRIMARY_CODE[prn].length()-1));
	    _dest[index]=std::complex<float>(float(a[0]),float(b[0]));
	    _dest[index+1]=std::complex<float>(float(a[1]),float(b[1]));
	}
}

void galileo_e5_a_code_gen_tiered(std::complex<float>* _dest,std::complex<float>* _primary ,unsigned int _prn, char _Signal[3])
{

    std::string _galileo_signal = _Signal;
    unsigned int prn=_prn-1;
    // Note: always generates 100 ms of tiered code
    if (_galileo_signal.rfind("5Q") != std::string::npos && _galileo_signal.length() >= 2)
	{
	    for (size_t i = 0; i < Galileo_E5a_Q_SECONDARY_CODE_LENGTH; i++)
		{
		    for (size_t k=0; k< Galileo_E5a_CODE_LENGTH_CHIPS; k++)
			{
			    //_dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k] = _primary[k];
			    //_dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k] = new std::complex<float>(0,0);
			    _dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k].imag( _primary[k].imag() *
				    (Galileo_E5a_Q_SECONDARY_CODE[prn].at(i)=='0' ? (float)1 : (float)-1));

			    _dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k].real((float)0);
			}
		}
	}
    else if (_galileo_signal.rfind("5I") != std::string::npos && _galileo_signal.length() >= 2)
	{
	    for (size_t i = 0; i < Galileo_E5a_Q_SECONDARY_CODE_LENGTH; i++)
		{
		    for (size_t k=0; k< Galileo_E5a_CODE_LENGTH_CHIPS; k++)
			{
			    //_dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k] = _primary[k];
			    //_dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k] = new std::complex<float>(0,0);
			    // Modulo operator i%20 since i[0,99] and sec code[0,19]
			    _dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k].real( _primary[k].real() *
				    (Galileo_E5a_I_SECONDARY_CODE.at(i%20)=='0' ? (float)1 : (float)-1));

			    _dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k].imag((float)0);

			}
		}
	}
    else if (_galileo_signal.rfind("5X") != std::string::npos && _galileo_signal.length() >= 2)
	{
	    for (size_t i = 0; i < Galileo_E5a_Q_SECONDARY_CODE_LENGTH; i++)
		{
		    for (size_t k=0; k< Galileo_E5a_CODE_LENGTH_CHIPS; k++)
			{
			    //_dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k] = _primary[k];
			    //_dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k] = new std::complex<float>(0,0);

			    _dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k].imag( _primary[k].imag() *
				    (Galileo_E5a_Q_SECONDARY_CODE[prn].at(i)=='0' ?  (float)1 : (float)-1));

			    _dest[i*Galileo_E5a_CODE_LENGTH_CHIPS + k].real( _primary[k].real() *
				    (Galileo_E5a_I_SECONDARY_CODE.at(i%20)=='0' ?  (float)1 : (float)-1));
			}
		}
	}
    else
	{
	    std::cout << "Signal doesn't correspond to E5a signal" << std::endl;
	}
}

void galileo_e5_a_code_gen_complex_sampled(std::complex<float>* _dest, char _Signal[3],
		unsigned int _prn, signed int _fs, unsigned int _chip_shift,
        bool _secondary_flag)
{
    // This function is based on the GNU software GPS for MATLAB in the Kay Borre book

    //std::string _galileo_signal = _Signal;
    unsigned int _samplesPerCode;
    unsigned int delay;
    unsigned int _codeLength = Galileo_E5a_CODE_LENGTH_CHIPS;
    const int _codeFreqBasis = Galileo_E5a_CODE_CHIP_RATE_HZ; //Hz


    std::complex<float>* _code;
//    if (posix_memalign((void**)&_code, 16, _codeLength * sizeof(gr_complex)) == 0){};
    _code=new std::complex<float>[_codeLength];
    //std::complex<float> primary_code_E5a_chips[(int)Galileo_E5a_CODE_LENGTH_CHIPS];


    galileo_e5_a_code_gen_complex_primary(_code , _prn , _Signal);

    if (_secondary_flag)
	{
	    std::complex<float>* _tiered_code = new std::complex<float>
		[Galileo_E5a_Q_SECONDARY_CODE_LENGTH * Galileo_E5a_CODE_LENGTH_CHIPS];

	    _codeLength *= Galileo_E5a_Q_SECONDARY_CODE_LENGTH;
//	    std::complex<float>* _tiered_code;
//	    if (posix_memalign((void**)&_tiered_code, 16, _codeLength * sizeof(gr_complex)) == 0){};
//	    std::complex<float> _tiered_code[Galileo_E5a_Q_SECONDARY_CODE_LENGTH * Galileo_E5a_CODE_LENGTH_CHIPS];
	    //malloc(_tiered_code, )
	    //std::cout << sizeof (&_tiered_code) << std::endl;

	    galileo_e5_a_code_gen_tiered(_tiered_code, _code,_prn, _Signal);

	    delete[] _code;
	    //free(_code);
	    //if (posix_memalign((void**)&_code, 16, _codeLength * sizeof(gr_complex)) == 0){};
	    _code = _tiered_code;
//	    delete[] _tiered_code;
	    free(_tiered_code);



	}

    _samplesPerCode = round(_fs / (_codeFreqBasis / _codeLength));
    // NOTE: if secondary, delay accounts for tiered code delay and samples/codesecondary
    delay = ((_codeLength - _chip_shift)
	    % _codeLength) * _samplesPerCode / _codeLength;

    //std::cout << "check tiered code delay" << delay << std::endl;

    //std::cout << "check codelength" << _codeLength << std::endl;
    if (_fs != _codeFreqBasis)
        {
        	std::complex<float>* _resampled_signal;// = new std::complex<float>[_codeLength];
        	if (posix_memalign((void**)&_resampled_signal, 16, _samplesPerCode * sizeof(gr_complex)) == 0){};
        	resampler(_code, _resampled_signal, _codeFreqBasis, _fs,
        	                    _codeLength, _samplesPerCode); //resamples code to fs

//        	free(_code);
//        	if (posix_memalign((void**)&_code, 16, _samplesPerCode * sizeof(gr_complex)) == 0){};
        	delete[] _code;
        	_code = _resampled_signal;
//        	delete[] _resampled_signal;
        	//free(_resampled_signal);
        }
    //std::cout << _fs << "fs" << std::endl;

    for (unsigned int i = 0; i < _samplesPerCode; i++)
        {
            _dest[(i+delay)%_samplesPerCode] = _code[i];
        }

    if (_code[0]==gr_complex(0,0))
	{
	    std::cout <<"ERROR: first chip is 0. prn:"<< _prn << std::endl;
	    std::cout << _Signal << "signal" << std::endl;
	}

    //std::cout << "no problem gen sampled code" <<_prn << " " << _code[0] <<std::endl;
    free(_code);

}
