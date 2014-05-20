/*
 * galileo_e5_signal_processing.cc
 *
 *  Created on: May 20, 2014
 *      Author: marc
 */

#include "galileo_e5_signal_processing.h"

void galileo_e5_a_code_gen_complex(std::complex<float>* _dest, signed int _prn, bool _pilot)
{
	unsigned int prn=_prn-1;
	unsigned int index=0;
	//int _code_int[(int)Galileo_E5a_CODE_LENGTH_CHIPS];
	int a[4];

	if ((_prn < 1) || (_prn > 50))
	        {
	            return;
	        }
	if (_pilot)
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
	else
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

void galileo_e5_a_code_gen_complex_sampled(std::complex<float>* _dest, bool _pilot,
		unsigned int _prn, signed int _fs, unsigned int _chip_shift,
        bool _secondary_flag)
{
    // This function is based on the GNU software GPS for MATLAB in the Kay Borre book

    std::complex<float> _code[Galileo_E5a_CODE_LENGTH_CHIPS];
    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int _codeFreqBasis = Galileo_E5a_CODE_CHIP_RATE_HZ; //Hz
    unsigned int _codeLength = Galileo_E5a_CODE_LENGTH_CHIPS;
    std::complex<float> primary_code_E5a_chips[(int)Galileo_E5a_CODE_LENGTH_CHIPS];
    _samplesPerCode = round(_fs / (_codeFreqBasis / _codeLength));
    const unsigned int delay = (((int)Galileo_E5a_CODE_LENGTH_CHIPS - _chip_shift)
                                % (int)Galileo_E5a_CODE_LENGTH_CHIPS)
                                * _samplesPerCode / Galileo_E5a_CODE_LENGTH_CHIPS;

    galileo_e5_a_code_gen_complex(_code , _prn , _pilot);

    if (_fs != _codeFreqBasis)
        {
        	std::complex<float>* _resampled_signal = new std::complex<float>[_codeLength];
        	resampler(_code, _resampled_signal, _codeFreqBasis, _fs,
        	                    _codeLength, _samplesPerCode); //resamples code to fs
        	delete[] _code;
        	_code = _resampled_signal;
        }
    // TODO generar codigo secundario cuando sepamos si se hace aqui o se replica en el tracking
    // o en una funcion a parte en esta misma clase
    for (unsigned int i = 0; i < _samplesPerCode; i++)
        {
            _dest[(i+delay)%_samplesPerCode] = _code[i];
        }

    delete[] _code;

}
