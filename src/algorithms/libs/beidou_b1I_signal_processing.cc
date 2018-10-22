/*!
 * \file beidou_b1I_signal_processing.cc
 * \brief This class implements various functions for BeiDou B1I signal
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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

#include "beidou_b1I_signal_processing.h"

auto auxCeil = [](float x) { return static_cast<int>(static_cast<long>((x) + 1)); };

void beidou_b1i_code_gen_int(int* _dest, signed int _prn, unsigned int _chip_shift)
{
    const unsigned int _code_length = 2046;
    bool G1[_code_length];
    bool G2[_code_length];
    bool G1_register[11] = {0,1,0,1,0,1,0,1,0,1,0};
    bool G2_register[11] = {0,1,0,1,0,1,0,1,0,1,0};
    bool feedback1, feedback2;
    bool aux;
    unsigned int lcv, lcv2;
    unsigned int delay;
    signed int prn_idx;
    /* G2 Delays as defined in GPS-ISD-200D */
    const signed int delays[33] = {712 /*PRN1*/, 1581, 1414, 1550, 581, 771, 1311, 1043, 1549, 359, 710, 1579, 1548, 1103, 579, 769, 358, 709, 1411, 1547,
        1102, 578, 357, 1577, 1410, 1546, 1101, 707, 1576, 1409, 1545, 354 /*PRN32*/,
        705};
    const signed int phase1[37] = {1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 8, 8, 8, 9, 9, 10};
    const signed int phase2[37] = {3, 4, 5, 6, 8, 9, 10, 11, 7, 4, 5, 6, 8, 9, 10, 11, 5, 6, 8, 9, 10, 11, 6, 8, 9, 10, 11, 8, 9, 10, 11, 9, 10, 11, 10, 11, 11};

    // compute delay array index for given PRN number
    prn_idx = _prn - 1;

    /* A simple error check */
    if ((prn_idx < 0) || (prn_idx > 51))
        return;

    /*for (lcv = 0; lcv < 11; lcv++)
        {
            G1_register[lcv] = 1;
            G2_register[lcv] = 1;
        }*/

    /* Generate G1 & G2 Register */
    for (lcv = 0; lcv < _code_length; lcv++)
        {
            G1[lcv] = G1_register[0];
            G2[lcv] = G2_register[-(phase1[prn_idx] - 11) ] ^ G2_register[-(phase2[prn_idx] - 11) ];

            feedback1 = (G1_register[0] + G1_register[1] + G1_register[2] + G1_register[3] + G1_register[4] + G1_register[10]) & 0x1;
            feedback2 = (G2_register[0] + G2_register[2] + G2_register[3] + G2_register[6] + G2_register[7] + G2_register[8] + G2_register[9] + G2_register[10]) & 0x1;

            for (lcv2 = 0; lcv2 < 10; lcv2++)
                {
                    G1_register[lcv2] = G1_register[lcv2 + 1];
                    G2_register[lcv2] = G2_register[lcv2 + 1];
                }

            G1_register[10] = feedback1;
            G2_register[10] = feedback2;
        }

    /* Set the delay */
    delay = _code_length - delays[prn_idx]*0; //**********************************
    delay += _chip_shift;
    delay %= _code_length;

    /* Generate PRN from G1 and G2 Registers */
    for (lcv = 0; lcv < _code_length; lcv++)
        {
            aux = G1[(lcv + _chip_shift) % _code_length] ^ G2[delay];
            if (aux == true)
                {
                    _dest[lcv] = 1;
                }
            else
                {
                    _dest[lcv] = -1;
                }

            delay++;
//std::cout  << _dest[lcv] << " ";
            delay %= _code_length;
        }
}


void beidou_b1i_code_gen_float(float* _dest, signed int _prn, unsigned int _chip_shift)
{
    unsigned int _code_length = 2046;
    int b1i_code_int[_code_length];

    beidou_b1i_code_gen_int(b1i_code_int, _prn, _chip_shift);

    for (unsigned int ii = 0; ii < _code_length; ++ii)
        {
            _dest[ii] = static_cast<float>(b1i_code_int[ii]);
        }
}


void beidou_b1i_code_gen_complex(std::complex<float>* _dest, signed int _prn, unsigned int _chip_shift)
{
    unsigned int _code_length = 2046;
    int b1i_code_int[_code_length];

    beidou_b1i_code_gen_int(b1i_code_int, _prn, _chip_shift);

    for (unsigned int ii = 0; ii < _code_length; ++ii)
        {
            _dest[ii] = std::complex<float>(static_cast<float>(b1i_code_int[ii]), 0.0f);
        }
}


/*
 *  Generates complex GPS L1 C/A code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1i_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, int _fs, unsigned int _chip_shift)
{
    // This function is based on the GNU software GPS for MATLAB in the Kay Borre book
    std::complex<float> _code[2046];
    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    float aux;
    const signed int _codeFreqBasis = 2046000;  //Hz
    const signed int _codeLength = 2046;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<signed int>(static_cast<double>(_fs) / static_cast<double>(_codeFreqBasis / _codeLength));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(_codeFreqBasis);        // C/A chip period in sec
    beidou_b1i_code_gen_complex(_code, _prn, _chip_shift);  //generate C/A code 1 sample per chip

    for (signed int i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read C/A code values -------------------------
            // The length of the index array depends on the sampling frequency -
            // number of samples per millisecond (because one C/A code period is one
            // millisecond).

            // _codeValueIndex = ceil((_ts * ((float)i + 1)) / _tc) - 1;
            aux = (_ts * (i + 1)) / _tc;
            _codeValueIndex = auxCeil(aux) - 1;

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
                    _dest[i] = _code[_codeValueIndex];  //repeat the chip -> upsample
                }
        }
}
