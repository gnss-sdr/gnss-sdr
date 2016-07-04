 /*!
 * \file beidou_b1i_signal_processing.cc
 * \brief This class implements various functions for BeiDou B1I signals
 * \author Giorgio Savastano, 2015. giorgio.savastano(at)uniroma1.it
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

#include "beidou_b1i_signal_processing.h"
#include <stdlib.h>
#include <cmath>

auto auxCeil = [](float x){ return static_cast<int>(static_cast<long>((x)+1)); };

void beidou_b1i_code_gen_complex(std::complex<float>* _dest, signed int _prn, unsigned int _chip_shift)
{
    const unsigned int _code_length = 2046;
    signed int G1[_code_length];
    signed int G2[_code_length];
    signed int G1_register[11] = { 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1};       /* 1=>-1, 0=>1 */
    signed int G2_register[11] = { 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1};       /* 1=>-1, 0=>1 */
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
                G1_register[10 - lcv2] = G1_register[9 - lcv2];
            }
            // put feedback in position 1
            G1_register[0] = feedback1;
        }

    /* Generate G2 by tapping the shift register */
    for(lcv = 0; lcv < _code_length; lcv++)
        {
    		switch(_prn){
                case 1:
                   G2[lcv] = G2_register[0]*G2_register[2];
                   break;
                case 2:
                   G2[lcv] = G2_register[0]*G2_register[3];
                   break;
                case 3:
                   G2[lcv] = G2_register[0]*G2_register[4];
                   break;
                case 4:
                   G2[lcv] = G2_register[0]*G2_register[5];
                   break;
                case 5:
                   G2[lcv] = G2_register[0]*G2_register[7];
                   break;
                case 6:
                   G2[lcv] = G2_register[0]*G2_register[8];
                   break;
                case 7:
                   G2[lcv] = G2_register[0]*G2_register[9];
                   break;
                case 8:
                   G2[lcv] = G2_register[0]*G2_register[10];
                   break;
                case 9:
                   G2[lcv] = G2_register[1]*G2_register[6];
                   break;
                case 10:
                   G2[lcv] = G2_register[2]*G2_register[3];
                   break;
                case 11:
                   G2[lcv] = G2_register[2]*G2_register[4];
                   break;
                case 12:
                   G2[lcv] = G2_register[2]*G2_register[5];
                   break;
                case 13:
                   G2[lcv] = G2_register[2]*G2_register[7];
                   break;
                case 14:
                   G2[lcv] = G2_register[2]*G2_register[8];
                   break;
                case 15:
                   G2[lcv] = G2_register[2]*G2_register[9];
                   break;
                case 16:
                   G2[lcv] = G2_register[2]*G2_register[10];
                   break;
                case 17:
                   G2[lcv] = G2_register[3]*G2_register[4];
                   break;
                case 18:
                   G2[lcv] = G2_register[3]*G2_register[5];
                   break;
                case 19:
                   G2[lcv] = G2_register[3]*G2_register[7];
                   break;
                case 20:
                   G2[lcv] = G2_register[3]*G2_register[8];
                   break;
                case 21:
                   G2[lcv] = G2_register[3]*G2_register[9];
                   break;
                case 22:
                   G2[lcv] = G2_register[3]*G2_register[10];
                   break;
                case 23:
                   G2[lcv] = G2_register[4]*G2_register[5];
                   break;
                case 24:
                   G2[lcv] = G2_register[4]*G2_register[7];
                   break;
                case 25:
                    G2[lcv] = G2_register[4]*G2_register[8];
                    break;
                case 26:
                    G2[lcv] = G2_register[4]*G2_register[9];
                    break;
                case 27:
                    G2[lcv] = G2_register[4]*G2_register[10];
                    break;
                case 28:
                    G2[lcv] = G2_register[5]*G2_register[7];
                    break;
                case 29:
                    G2[lcv] = G2_register[5]*G2_register[8];
                    break;
                case 30:
                    G2[lcv] = G2_register[5]*G2_register[9];
                    break;
                case 31:
                    G2[lcv] = G2_register[5]*G2_register[10];
                    break;
                case 32:
                    G2[lcv] = G2_register[7]*G2_register[8];
                    break;
                case 33:
                    G2[lcv] = G2_register[7]*G2_register[9];
                    break;
                case 34:
                    G2[lcv] = G2_register[7]*G2_register[10];
                    break;
                case 35:
                    G2[lcv] = G2_register[8]*G2_register[9];
                    break;
                case 36:
                    G2[lcv] = G2_register[8]*G2_register[10];
                    break;
                case 37:
                    G2[lcv] = G2_register[9]*G2_register[10];
                    break;                                                              
            }

            // computation of the G2 feedback
            feedback2 = G2_register[0]*G2_register[1]*G2_register[2]*G2_register[3]*G2_register[4]*G2_register[7]*G2_register[8]*G2_register[10];
            // shift to the right
            for(lcv2 = 0; lcv2 < 10; lcv2++)
            {
                G2_register[10 - lcv2] = G2_register[9 - lcv2];
            }
            // put feedback in position 1
            G2_register[0] = feedback2;
        }

    /* Generate PRN from G1 and G2 Registers */
    for(lcv = 0; lcv < _code_length; lcv++)
        {
        _dest[lcv] = -G1[lcv]*G2[lcv];
        }
}

/*
 *  Generates complex BeiDou B1I code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1i_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs, unsigned int _chip_shift)
{
    // This function is based on the GNU software GPS for MATLAB in the Kay Borre book
    std::complex<float> _code[2046];
    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    float aux;
    const signed int _codeFreqBasis = 2046000;    // [Hz]
    const signed int _codeLength    = 2046;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<signed int>(static_cast<double>(_fs) / static_cast<double>(_codeFreqBasis / _codeLength));

        //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                    // Sampling period in sec
    _tc = 1.0 / static_cast<float>(_codeFreqBasis);         // B1I chip period in sec
    beidou_b1i_code_gen_complex(_code, _prn, _chip_shift);  //generate B1I code 1 sample per chip

    for (signed int i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1I code values -------------------------
            // The length of the index array depends on the sampling frequency -
            // number of samples per millisecond (because one B1I code period is one
            // millisecond).

            // _codeValueIndex = ceil((_ts * ((float)i + 1)) / _tc) - 1;
            aux = (_ts * (i + 1)) / _tc;
            _codeValueIndex = auxCeil( aux ) - 1;

            //--- Make the digitized version of the B1I code -----------------------
            // The "upsampled" code is made by selecting values form the B1I code
            // chip array (B1I) for the time instances of each sample.
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




