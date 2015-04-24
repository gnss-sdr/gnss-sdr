/*!
 * \file gps_l2c_signal.cc
 * \brief This class implements signal generators for the GPS L2C signals
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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

#include "GPS_L2C.h"
#include <stdlib.h>
#include <stdint.h>
#include <cmath>


int32_t gps_l2c_m_shift(int32_t x)
{
	return (int32_t)((x>>1)^((x&1)*0445112474));
}

void gps_l2c_m_code(int32_t * _dest, unsigned int _prn)
{
	int32_t x;
	x= GPS_L2C_M_INIT_REG[_prn-1];
	for (int n=0; n<GPS_L2_M_CODE_LENGTH_CHIPS; n++)
	{
		x= gps_l2c_m_shift(x);
		_dest[n]=(int8_t)(x&1);
	}
}

/*
 *  Generates complex GPS L2C M code for the desired SV ID and sampled to specific sampling frequency
 */
void gps_l2c_m_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs)
{
	int32_t _code[GPS_L2_M_CODE_LENGTH_CHIPS];

	if (_prn<51)
	{
		gps_l2c_m_code(_code, _prn);
	}

    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const signed int _codeFreqBasis = GPS_L2_M_CODE_RATE_HZ; //Hz
    const signed int _codeLength = GPS_L2_M_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = round(_fs / (_codeFreqBasis / _codeLength));

    //--- Find time constants --------------------------------------------------
    _ts = 1/(float)_fs;   // Sampling period in sec
    _tc = 1/(float)_codeFreqBasis;  // C/A chip period in sec
    //gps_l1_ca_code_gen_complex(_code,_prn); //generate C/A code 1 sample per chip

    for (signed int i=0; i<_samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read C/A code values -------------------------
            // The length of the index array depends on the sampling frequency -
            // number of samples per millisecond (because one C/A code period is one
            // millisecond).

            _codeValueIndex = ceil((_ts * ((float)i + 1)) / _tc) - 1;

            //--- Make the digitized version of the C/A code -----------------------
            // The "upsampled" code is made by selecting values form the CA code
            // chip array (caCode) for the time instances of each sample.
            if (i == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0-2.0*_code[_codeLength - 1],0);

                }
            else
                {
                    _dest[i] = std::complex<float>(1.0-2.0*_code[_codeValueIndex],0);; //repeat the chip -> upsample
                }
        }
}




