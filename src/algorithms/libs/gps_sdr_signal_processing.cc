/*!
 * \file gps_sdr_signal_processing.cc
 * \brief This class implements various functions for GPS L1 CA signals
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
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

#include "gps_sdr_signal_processing.h"

#include <math.h>
#include <stdlib.h>
#include <cmath>


void gps_l1_ca_code_gen_complex(std::complex<float>* _dest, signed int _prn, unsigned int _chip_shift)
{

	unsigned int G1[1023];
	unsigned int G2[1023];
	unsigned int G1_register[10], G2_register[10];
	unsigned int feedback1, feedback2;
	unsigned int lcv, lcv2;
	unsigned int delay;
	signed int prn = _prn-1; //Move the PRN code to fit an array indices

	/* G2 Delays as defined in GPS-ISD-200D */
	signed int delays[51] = {5, 6, 7, 8, 17, 18, 139, 140, 141, 251, 252, 254 ,255, 256, 257, 258, 469, 470, 471, 472,
		473, 474, 509, 512, 513, 514, 515, 516, 859, 860, 861, 862, 145, 175, 52, 21, 237, 235, 886, 657, 634, 762,
		355, 1012, 176, 603, 130, 359, 595, 68, 386};

	/* A simple error check */
	if((prn < 0) || (prn > 51))
		return;

	for(lcv = 0; lcv < 10; lcv++)
	{
		G1_register[lcv] = 1;
		G2_register[lcv] = 1;
	}

	/* Generate G1 & G2 Register */
	for(lcv = 0; lcv < 1023; lcv++)
	{
		G1[lcv] = G1_register[0];
		G2[lcv] = G2_register[0];

		feedback1 = G1_register[7]^G1_register[0];
		feedback2 = (G2_register[8] + G2_register[7] + G2_register[4] + G2_register[2] + G2_register[1] + G2_register[0]) & 0x1;

		for(lcv2 = 0; lcv2 < 9; lcv2++)
		{
			G1_register[lcv2] = G1_register[lcv2 + 1];
			G2_register[lcv2] = G2_register[lcv2 + 1];
		}

		G1_register[9] = feedback1;
		G2_register[9] = feedback2;
	}

	/* Set the delay */
	delay = 1023 - delays[prn];
	delay += _chip_shift;
	delay %= 1023;
	/* Generate PRN from G1 and G2 Registers */
	for(lcv = 0; lcv < 1023; lcv++)
	{
		_dest[lcv] = std::complex<float>(G1[(lcv +  _chip_shift)%1023]^G2[delay], 0);
		if(_dest[lcv].real() == 0.0) //javi
		{
			_dest[lcv].real(-1.0);
		}
		delay++;
		delay %= 1023;
		//std::cout<<_dest[lcv].real(); //OK
	}

}


/*!
 * \
 * code_gen_complex_sampled, generate GPS L1 C/A code complex for the desired SV ID and sampled to specific sampling frequency
 * \
 */
void gps_l1_ca_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs, unsigned int _chip_shift)
{
	// This function is based on the GNU software GPS for MATLAB in the Kay Borre book
    std::complex<float> _code[1023];
    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const signed int _codeFreqBasis = 1023000; //Hz
    const signed int _codeLength = 1023;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = round(_fs / (_codeFreqBasis / _codeLength));

    //--- Find time constants --------------------------------------------------
    _ts = 1/(float)_fs;   // Sampling period in sec
    _tc = 1/(float)_codeFreqBasis;  // C/A chip period in sec
    gps_l1_ca_code_gen_complex(_code,_prn, _chip_shift); //generate C/A code 1 sample per chip
    //std::cout<<"ts="<<_ts<<std::endl;
    //std::cout<<"tc="<<_tc<<std::endl;
    //std::cout<<"sv="<<_prn<<std::endl;
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
                    _dest[i] = _code[_codeLength - 1];

                }
            else
                {
                    _dest[i] = _code[_codeValueIndex]; //repeat the chip -> upsample
                }
        }
}




