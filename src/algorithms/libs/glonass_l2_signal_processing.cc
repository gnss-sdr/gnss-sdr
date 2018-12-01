/*!
 * \file glonass_l2_signal_processing.cc
 * \brief This class implements various functions for GLONASS L2 CA signals
 * \author Damian Miralles, 2018, dmiralles2009(at)gmail.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "glonass_l2_signal_processing.h"

auto auxCeil = [](float x) { return static_cast<int32_t>(static_cast<int64_t>((x) + 1)); };

void glonass_l2_ca_code_gen_complex(std::complex<float>* _dest, /* int32_t _prn,*/ uint32_t _chip_shift)
{
    const uint32_t _code_length = 511;
    bool G1[_code_length];
    bool G1_register[9];
    bool feedback1;
    bool aux;
    uint32_t delay;
    uint32_t lcv, lcv2;

    for (lcv = 0; lcv < 9; lcv++)
        {
            G1_register[lcv] = 1;
        }

    /* Generate G1 Register */
    for (lcv = 0; lcv < _code_length; lcv++)
        {
            G1[lcv] = G1_register[2];

            feedback1 = G1_register[4] ^ G1_register[0];

            for (lcv2 = 0; lcv2 < 8; lcv2++)
                {
                    G1_register[lcv2] = G1_register[lcv2 + 1];
                }

            G1_register[8] = feedback1;
        }

    /* Generate PRN from G1 Register */
    for (lcv = 0; lcv < _code_length; lcv++)
        {
            aux = G1[lcv];
            if (aux == true)
                {
                    _dest[lcv] = std::complex<float>(1, 0);
                }
            else
                {
                    _dest[lcv] = std::complex<float>(-1, 0);
                }
        }

    /* Set the delay */
    delay = _code_length;
    delay += _chip_shift;
    delay %= _code_length;

    /* Generate PRN from G1 and G2 Registers */
    for (lcv = 0; lcv < _code_length; lcv++)
        {
            aux = G1[(lcv + _chip_shift) % _code_length];
            if (aux == true)
                {
                    _dest[lcv] = std::complex<float>(1, 0);
                }
            else
                {
                    _dest[lcv] = std::complex<float>(-1, 0);
                }
            delay++;
            delay %= _code_length;
        }
}


/*
 *  Generates complex GLONASS L2 C/A code for the desired SV ID and sampled to specific sampling frequency
 */
void glonass_l2_ca_code_gen_complex_sampled(std::complex<float>* _dest, /* uint32_t _prn,*/ int32_t _fs, uint32_t _chip_shift)
{
    // This function is based on the GNU software GPS for MATLAB in the Kay Borre book
    std::complex<float> _code[511];
    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    float aux;
    const int32_t _codeFreqBasis = 511000;  //Hz
    const int32_t _codeLength = 511;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int32_t>(static_cast<double>(_fs) / static_cast<double>(_codeFreqBasis / _codeLength));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                 // Sampling period in sec
    _tc = 1.0 / static_cast<float>(_codeFreqBasis);      // C/A chip period in sec
    glonass_l2_ca_code_gen_complex(_code, _chip_shift);  //generate C/A code 1 sample per chip

    for (int32_t i = 0; i < _samplesPerCode; i++)
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
