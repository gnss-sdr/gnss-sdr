 /*!
 * \file beidou_sdr_signal_processing.cc
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
#include "BEIDOU_B1I.h"
#include <stdlib.h>
#include <cmath>

auto auxCeil = [](float x){ return static_cast<int>(static_cast<long>((x)+1)); };

static int mod(double a, double N)
{
    return static_cast<int>(a - N*floor(a/N)); //return in range [0, N)
}

void beidou_b1i_code_gen_complex(std::complex<float>* _dest, 
                                 signed int _prn, 
                                 unsigned int _chip_shift)
{
    const unsigned int _code_length = BEIDOU_B1I_CODE_LENGTH_CHIPS;
    signed int G1[_code_length];
    signed int G2[_code_length];
    signed int G1_register[11] = { 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1};       /* 1=>-1, 0=>1 */
    signed int G2_register[11] = { 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1};       /* 1=>-1, 0=>1 */
    signed int feedback1, feedback2;
    unsigned int lcv, lcv2;

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
    signed int _offset_prn, _offset_nh;
    double _phi_prn, _phi_nh;

    const double _fs_in                 = static_cast<double>(_fs);
    const signed int _codeLength        = static_cast<int>(BEIDOU_B1I_CODE_LENGTH_CHIPS);
    const signed int _codeDelayChips    = (_codeLength - _chip_shift) % _codeLength;
    const signed int _codeDelaySamples  = static_cast<int>(_codeDelayChips * (_fs_in / BEIDOU_B1I_CODE_RATE_HZ));

    //--- Find number of samples per spreading code ----------------------------
    const signed int _samplesPerCode = static_cast<signed int>(_fs_in / (BEIDOU_B1I_CODE_RATE_HZ / BEIDOU_B1I_CODE_LENGTH_CHIPS));

    //generate B1I code 1 sample per chip
    beidou_b1i_code_gen_complex(_code, _prn, _chip_shift);

    for (signed int i = 0; i < _samplesPerCode; i++)
        {
            // Offset for the PRN codes in order to add the proper phase 
            _phi_prn = static_cast<double>(i - _codeDelaySamples) * (BEIDOU_B1I_CODE_RATE_HZ / _fs_in);
            _offset_prn = mod(_phi_prn, BEIDOU_B1I_CODE_LENGTH_CHIPS);

            // Offset for the NH code in order to add the proper phase
            _phi_nh = static_cast<double>(i - _codeDelaySamples) * (NH_BITS_RATE / _fs_in);
            _offset_nh = mod(_phi_nh, NH_BIT_DURATION);

            _dest[i] = _code[_offset_prn] * static_cast<float>(NH_CODE[_offset_nh]);
        }
}




