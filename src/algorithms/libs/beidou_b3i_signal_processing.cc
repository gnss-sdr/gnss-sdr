/*!
 * \file beidou_b3i_signal_processing.cc
 * \brief This class implements various functions for BeiDou B1I signal
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
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

#include "beidou_b3i_signal_processing.h"

auto auxCeil = [](float x) { return static_cast<int>(static_cast<long>((x) + 1)); };

void beidou_b3i_code_gen_int(gsl::span<int> _dest, signed int _prn, unsigned int _chip_shift)
{
    const unsigned int _code_length = 10230;
    bool G1[_code_length];
    bool G2[_code_length];
    std::array<bool, 13> G1_register = {{true, true, true, true, true, true, true, true, true, true, true, true, true}};
    std::array<bool, 13> G2_register = {{true, true, true, true, true, true, true, true, true, true, true, true, true}};
    std::array<bool, 13> G1_register_reset = {{false, false, true, true, true, true, true, true, true, true, true, true, true}};
    bool feedback1, feedback2, aux;
    uint32_t lcv, lcv2, delay;
    int32_t prn_idx = _prn - 1;

    std::array<std::array<bool, 13>, 63> G2_register_shifted =
        {{{{true, false, true, false, true, true, true, true, true, true, true, true, true}},
            {{true, true, true, true, false, false, false, true, false, true, false, true, true}},
            {{true, false, true, true, true, true, false, false, false, true, false, true, false}},
            {{true, true, true, true, true, true, true, true, true, true, false, true, true}},
            {{true, true, false, false, true, false, false, false, true, true, true, true, true}},
            {{true, false, false, true, false, false, true, true, false, false, true, false, false}},
            {{true, true, true, true, true, true, true, false, true, false, false, true, false}},
            {{true, true, true, false, true, true, true, true, true, true, true, false, true}},
            {{true, false, true, false, false, false, false, false, false, false, false, true, false}},
            {{false, false, true, false, false, false, false, false, true, true, false, true, true}},
            {{true, true, true, false, true, false, true, true, true, false, false, false, false}},
            {{false, false, true, false, true, true, false, false, true, true, true, true, false}},
            {{false, true, true, false, false, true, false, false, true, false, true, false, true}},
            {{false, true, true, true, false, false, false, true, false, false, true, true, false}},
            {{true, false, false, false, true, true, false, false, false, true, false, false, true}},
            {{true, true, true, false, false, false, true, true, true, true, true, false, false}},
            {{false, false, true, false, false, true, true, false, false, false, true, false, true}},
            {{false, false, false, false, false, true, true, true, false, true, true, false, false}},
            {{true, false, false, false, true, false, true, false, true, false, true, true, true}},
            {{false, false, false, true, false, true, true, false, true, true, true, true, false}},
            {{false, false, true, false, false, false, false, true, false, true, true, false, true}},
            {{false, false, true, false, true, true, false, false, false, true, false, true, false}},
            {{false, false, false, true, false, true, true, false, false, true, true, true, true}},
            {{false, false, true, true, false, false, true, true, false, false, false, true, false}},
            {{false, false, true, true, true, false, true, false, false, true, false, false, false}},
            {{false, true, false, false, true, false, false, true, false, true, false, false, true}},
            {{true, false, true, true, false, true, true, false, true, false, false, true, true}},
            {{true, false, true, false, true, true, true, true, false, false, false, true, false}},
            {{false, false, false, true, false, true, true, true, true, false, true, false, true}},
            {{false, true, true, true, true, true, true, true, true, true, true, true, true}},
            {{false, true, true, false, true, true, false, false, false, true, true, true, true}},
            {{true, false, true, false, true, true, false, false, false, true, false, false, true}},
            {{true, false, false, true, false, true, false, true, false, true, false, true, true}},
            {{true, true, false, false, true, true, false, true, false, false, true, false, true}},
            {{true, true, false, true, false, false, true, false, true, true, true, false, true}},
            {{true, true, true, true, true, false, true, true, true, false, true, false, false}},
            {{false, false, true, false, true, false, true, true, false, false, true, true, true}},
            {{true, true, true, false, true, false, false, false, true, false, false, false, false}},
            {{true, true, false, true, true, true, false, false, true, false, false, false, false}},
            {{true, true, false, true, false, true, true, false, false, true, true, true, false}},
            {{true, false, false, false, false, false, false, true, true, false, true, false, false}},
            {{false, true, false, true, true, true, true, false, true, true, false, false, true}},
            {{false, true, true, false, true, true, false, true, true, true, true, false, false}},
            {{true, true, false, true, false, false, true, true, true, false, false, false, true}},
            {{false, false, true, true, true, false, false, true, false, false, false, true, false}},
            {{false, true, false, true, false, true, true, false, false, false, true, false, true}},
            {{true, false, false, true, true, true, true, true, false, false, true, true, false}},
            {{true, true, true, true, true, false, true, false, false, true, false, false, false}},
            {{false, false, false, false, true, false, true, false, false, true, false, false, true}},
            {{true, false, false, false, false, true, false, true, false, true, true, false, false}},
            {{true, true, true, true, false, false, true, false, false, true, true, false, false}},
            {{false, true, false, false, true, true, false, false, false, true, true, true, true}},
            {{false, false, false, false, false, false, false, false, true, true, false, false, false}},
            {{true, false, false, false, false, false, false, false, false, false, true, false, false}},
            {{false, false, true, true, false, true, false, true, false, false, true, true, false}},
            {{true, false, true, true, false, false, true, false, false, false, true, true, false}},
            {{false, true, true, true, false, false, true, true, true, true, false, false, false}},
            {{false, false, true, false, true, true, true, false, false, true, false, true, false}},
            {{true, true, false, false, true, true, true, true, true, false, true, true, false}},
            {{true, false, false, true, false, false, true, false, false, false, true, false, true}},
            {{false, true, true, true, false, false, false, true, false, false, false, false, false}},
            {{false, false, true, true, false, false, true, false, false, false, false, true, false}},
            {{false, false, true, false, false, false, true, false, false, true, true, true, false}}}};

    // A simple error check
    if ((prn_idx < 0) || (prn_idx > 63))
        {
            return;
        }

    // Assign shifted G2 register based on prn number
    G2_register = G2_register_shifted[prn_idx];
    std::reverse(G2_register.begin(), G2_register.end());

    // Generate G1 and G2 Register
    for (lcv = 0; lcv < _code_length; lcv++)
        {
            G1[lcv] = G1_register[0];
            G2[lcv] = G2_register[0];

            //feedback1 = (test_G1_register[0]+test_G1_register[2]+test_G1_register[3]+test_G1_register[12]) & 0x1;
            feedback1 = (G1_register[0] + G1_register[9] + G1_register[10] + G1_register[12]) & 0x01;
            feedback2 = (G2_register[0] + G2_register[1] + G2_register[3] + G2_register[4] +
                            G2_register[6] + G2_register[7] + G2_register[8] + G2_register[12]) &
                        0x01;

            for (lcv2 = 0; lcv2 < 12; lcv2++)
                {
                    G1_register[lcv2] = G1_register[lcv2 + 1];
                    G2_register[lcv2] = G2_register[lcv2 + 1];
                }

            G1_register[12] = feedback1;
            G2_register[12] = feedback2;

            // Reset G1 register if sequence found
            if (G1_register == G1_register_reset)
                {
                    G1_register = {{true, true, true, true, true, true, true, true, true, true, true, true, true}};
                }
        }

    delay = _code_length;
    delay += _chip_shift;
    delay %= _code_length;

    /* Generate PRN from G1 and G2 Registers */
    for (lcv = 0; lcv < _code_length; lcv++)
        {
            aux = (G1[(lcv + _chip_shift) % _code_length] + G2[delay]) & 0x01;
            if (aux == true)
                {
                    _dest[lcv] = 1;
                }
            else
                {
                    _dest[lcv] = -1;
                }

            delay++;
            delay %= _code_length;
        }
}


void beidou_b3i_code_gen_float(gsl::span<float> _dest, signed int _prn, unsigned int _chip_shift)
{
    unsigned int _code_length = 10230;
    int b3i_code_int[10230];

    beidou_b3i_code_gen_int(b3i_code_int, _prn, _chip_shift);

    for (unsigned int ii = 0; ii < _code_length; ++ii)
        {
            _dest[ii] = static_cast<float>(b3i_code_int[ii]);
        }
}


void beidou_b3i_code_gen_complex(gsl::span<std::complex<float>> _dest, signed int _prn, unsigned int _chip_shift)
{
    unsigned int _code_length = 10230;
    int b3i_code_int[10230];

    beidou_b3i_code_gen_int(b3i_code_int, _prn, _chip_shift);

    for (unsigned int ii = 0; ii < _code_length; ++ii)
        {
            _dest[ii] = std::complex<float>(static_cast<float>(b3i_code_int[ii]), 0.0F);
        }
}


void beidou_b3i_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, unsigned int _prn, int _fs, unsigned int _chip_shift)
{
    // This function is based on the GNU software GPS for MATLAB in the Kay Borre book
    std::complex<float> _code[10230];
    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    float aux;
    const signed int _codeFreqBasis = 10230000;  //Hz
    const signed int _codeLength = 10230;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<signed int>(static_cast<double>(_fs) / static_cast<double>(_codeFreqBasis / _codeLength));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                    // Sampling period in sec
    _tc = 1.0 / static_cast<float>(_codeFreqBasis);         // C/A chip period in sec
    beidou_b3i_code_gen_complex(_code, _prn, _chip_shift);  //generate C/A code 1 sample per chip

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
