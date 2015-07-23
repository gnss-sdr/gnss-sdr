/*
 * beidou_sdr_signal_processing.cc
 *
 *  Created on: Jul 15, 2015
 *      Author: giorgio
 */

#include "beidou_sdr_signal_processing.h"
#include <stdlib.h>
#include <cmath>

auto auxCeil = [](float x){ return static_cast<int>(static_cast<long>((x)+1)); };

void beidou_b1i_code_gen_complex(std::complex<float>* _dest, signed int _prn, unsigned int _chip_shift)
{
    const unsigned int _code_length = 2046;
    signed int G1[_code_length];
    signed int G2[_code_length];
    signed int G1_register[11] = {-1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1};
    signed int G2_register[11] = {-1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1};
    signed int feedback1, feedback2;
    bool aux;
    unsigned int lcv, lcv2;
    unsigned int delay;
    signed int prn_idx;

    /* Generate G1 */
    for(lcv = 0; lcv < _code_length; lcv++)
        {
            // equal to the last value of the shift register
    		G1[lcv] = G1_register[10];
    		// computation of the G1 feedback
            feedback1 = G1_register[0]*G1_register[6]*G1_register[7]*G1_register[8]*G1_register[9]*G1_register[10];

            // shift to the right
            for(lcv2 = 0; lcv2 < 10; lcv2++)
                {
                    G1_register[lcv2] = G1_register[lcv2 + 1];
                }

            // put feedback in position 1
            G1_register[0] = feedback1;
        }


    /* Generate G2 by tapping the shift register */
    for(lcv = 0; lcv < _code_length; lcv++)
        {
    		if(_prn == 1)
    		{
    			G2[lcv] = G2_register[0]*G2_register[2];
    		}
    		else if(_prn == 2)
    		{
    			G2[lcv] = G2_register[0]*G2_register[3];
    		}
    		else if(_prn == 3)
    		{
    			G2[lcv] = G2_register[0]*G2_register[4];
    		}
    		else if(_prn == 4)
    		{
    			G2[lcv] = G2_register[0]*G2_register[5];
    		}
    		else if(_prn == 5)
    		{
    			G2[lcv] = G2_register[0]*G2_register[7];
    		}
    		else if(_prn == 6)
    		{
    			G2[lcv] = G2_register[0]*G2_register[8];
    		}
    		else if(_prn == 7)
    		{
    			G2[lcv] = G2_register[0]*G2_register[9];
    		}
    		else if(_prn == 8)
    		{
    			G2[lcv] = G2_register[0]*G2_register[10];
    		}
    		else if(_prn == 9)
    		{
    			G2[lcv] = G2_register[1]*G2_register[6];
    		}
    		else if(_prn == 10)
    		{
    			G2[lcv] = G2_register[2]*G2_register[3];
    		}
    		else if(_prn == 11)
    		{
    			G2[lcv] = G2_register[2]*G2_register[4];
    		}
    		else if(_prn == 12)
    		{
    			G2[lcv] = G2_register[2]*G2_register[5];
    		}
    		else if(_prn == 13)
    		{
    			G2[lcv] = G2_register[2]*G2_register[7];
    		}
    		else if(_prn == 14)
    		{
    			G2[lcv] = G2_register[2]*G2_register[8];
    		}
    		else if(_prn == 15)
    		{
    			G2[lcv] = G2_register[2]*G2_register[9];
    		}
    		else if(_prn == 16)
    		{
    			G2[lcv] = G2_register[2]*G2_register[10];
    		}
    		else if(_prn == 17)
    		{
    			G2[lcv] = G2_register[3]*G2_register[4];
    		}
    		else if(_prn == 18)
    		{
    			G2[lcv] = G2_register[3]*G2_register[5];
    		}
    		else if(_prn == 19)
    		{
    			G2[lcv] = G2_register[3]*G2_register[7];
    		}
    		else if(_prn == 20)
    		{
    			G2[lcv] = G2_register[3]*G2_register[8];
    		}
    		else if(_prn == 21)
    		{
    			G2[lcv] = G2_register[3]*G2_register[9];
    		}
    		else if(_prn == 22)
    		{
    			G2[lcv] = G2_register[3]*G2_register[10];
    		}
    		else if(_prn == 23)
    		{
    			G2[lcv] = G2_register[4]*G2_register[5];
    		}
    		else if(_prn == 24)
    		{
    			G2[lcv] = G2_register[4]*G2_register[7];
    		}
    		else if(_prn == 25)
    		{
    			G2[lcv] = G2_register[4]*G2_register[8];
    		}
    		else if(_prn == 26)
    		{
    			G2[lcv] = G2_register[4]*G2_register[9];
    		}
    		else if(_prn == 27)
    		{
    			G2[lcv] = G2_register[4]*G2_register[10];
    		}
    		else if(_prn == 28)
    		{
    			G2[lcv] = G2_register[5]*G2_register[7];
    		}
    		else if(_prn == 29)
    		{
    			G2[lcv] = G2_register[5]*G2_register[8];
    		}
    		else if(_prn == 30)
    		{
    			G2[lcv] = G2_register[5]*G2_register[9];
    		}
    		else if(_prn == 31)
    		{
    			G2[lcv] = G2_register[5]*G2_register[10];
    		}
    		else if(_prn == 32)
    		{
    			G2[lcv] = G2_register[7]*G2_register[8];
    		}
    		else if(_prn == 33)
    		{
    			G2[lcv] = G2_register[7]*G2_register[9];
    		}
    		else if(_prn == 34)
    		{
    			G2[lcv] = G2_register[7]*G2_register[10];
    		}
    		else if(_prn == 35)
    		{
    			G2[lcv] = G2_register[8]*G2_register[9];
    		}
    		else if(_prn == 36)
    		{
    			G2[lcv] = G2_register[8]*G2_register[10];
    		}
    		else if(_prn == 36)
    		{
    			G2[lcv] = G2_register[9]*G2_register[10];
    		}

    		// computation of the G2 feedback
            feedback2 = G2_register[0]*G2_register[1]*G2_register[2]*G2_register[3]*G2_register[4]*G2_register[7]*G2_register[8]*G2_register[10];

            // shift to the right
            for(lcv2 = 0; lcv2 < 10; lcv2++)
                {
            		G2_register[lcv2] = G2_register[lcv2 + 1];
                }
            // put feedback in position 1
            G2_register[0] = feedback2;
        }


    /* Generate PRN from G1 and G2 Registers */
    for(lcv = 0; lcv < _code_length; lcv++)
        {
    		_dest[lcv] = G1[lcv]*G2[lcv];
        }
}

//    /* G2 Delays as defined in GPS-ISD-200D */
//    const signed int delays[37] = {5 /*PRN1*/, 6, 7, 8, 17, 18, 139, 140, 141, 251, 252, 254 ,255, 256, 257, 258, 469, 470, 471, 472,
//            473, 474, 509, 512, 513, 514, 515, 516, 859, 860, 861, 862 /*PRN32*/,
//            145, 175, 52, 21, 237};
//
//    // compute delay array index for given PRN number
//    if(120 <= _prn && _prn <= 138)
//    {
//    	prn_idx = _prn - 88;	// SBAS PRNs are at array indices 31 to 50 (offset: -120+33-1 =-88)
//    	//prn_idx = _prn - 87;	// SBAS PRNs are at array indices 31 to 50 (offset: -120+33 =-87)
//    }
//    else
//    {
//    	prn_idx = _prn - 1;
//    }
//
//    /* A simple error check */
//    if((prn_idx < 0) || (prn_idx > 51))
//        return;
//
//
//    /* Generate G1 & G2 Register */
//    for(lcv = 0; lcv < _code_length; lcv++)
//        {
//            G1[lcv] = G1_register[0];
//            G2[lcv] = G2_register[0];
//
//            feedback1 = G1_register[7]^G1_register[0];
//            feedback2 = (G2_register[8] + G2_register[7] + G2_register[4] + G2_register[2] + G2_register[1] + G2_register[0]) & 0x1;
//
//            for(lcv2 = 0; lcv2 < 10; lcv2++)
//                {
//                    G1_register[lcv2] = G1_register[lcv2 + 1];
//                    G2_register[lcv2] = G2_register[lcv2 + 1];
//                }
//
//            G1_register[10] = feedback1;
//            G2_register[10] = feedback2;
//        }
//
//    /* Set the delay */
//    delay = _code_length - delays[prn_idx];
//    delay += _chip_shift;
//    delay %= _code_length;
//
//    /* Generate PRN from G1 and G2 Registers */
//    for(lcv = 0; lcv < _code_length; lcv++)
//        {
//            aux = G1[(lcv + _chip_shift) % _code_length]^G2[delay];
//            if(aux == true)
//                {
//                    _dest[lcv] = std::complex<float>(1, 0);
//                }
//            else
//                {
//                    _dest[lcv] = std::complex<float>(-1, 0);
//                }
//            delay++;
//            delay %= _code_length;
//        }
//}



/*
 *  Generates complex GPS L1 C/A code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1i_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs, unsigned int _chip_shift)
{
    // This function is based on the GNU software GPS for MATLAB in the Kay Borre book
    std::complex<float> _code[2046];
    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    float aux;
    const signed int _codeFreqBasis = 2046000;    //Hz
    const signed int _codeLength = 2046;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<signed int>(static_cast<double>(_fs) / static_cast<double>(_codeFreqBasis / _codeLength));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                    // Sampling period in sec
    _tc = 1.0 / static_cast<float>(_codeFreqBasis);         // B1I chip period in sec
    beidou_b1i_code_gen_complex(_code, _prn, _chip_shift);  //generate B1I code 1 sample per chip

    for (signed int i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read C/A code values -------------------------
            // The length of the index array depends on the sampling frequency -
            // number of samples per millisecond (because one C/A code period is one
            // millisecond).

            // _codeValueIndex = ceil((_ts * ((float)i + 1)) / _tc) - 1;
            aux = (_ts * (i + 1)) / _tc;
            _codeValueIndex = auxCeil( aux ) - 1;

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




