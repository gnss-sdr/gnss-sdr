/*!
 * \file beidou_b1i_signal_processing.cc
 * \brief This class implements various functions for BeiDou B1I signal
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
#include <array>
#include <bitset>
#include <string>

auto auxCeil = [](float x) { return static_cast<int32_t>(static_cast<int64_t>((x) + 1)); };

void beidou_b1i_code_gen_int(gsl::span<int32_t> _dest, int32_t _prn, uint32_t _chip_shift)
{
    const uint32_t _code_length = 2046;
    std::bitset<_code_length> G1{};
    std::bitset<_code_length> G2{};

    std::bitset<11> G1_register(std::string("01010101010"));
    std::bitset<11> G2_register(std::string("01010101010"));

    bool feedback1, feedback2;
    bool aux;
    uint32_t lcv, lcv2;
    uint32_t delay;
    int32_t prn_idx;

    const std::array<int32_t, 33> delays = {712 /*PRN1*/, 1581, 1414, 1550, 581, 771, 1311, 1043, 1549, 359, 710, 1579, 1548, 1103, 579, 769, 358, 709, 1411, 1547,
        1102, 578, 357, 1577, 1410, 1546, 1101, 707, 1576, 1409, 1545, 354 /*PRN32*/,
        705};
    const std::array<int32_t, 37> phase1 = {1, 1, 1, 1, 1, 1, 1, 1, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 8, 8, 8, 9, 9, 10};
    const std::array<int32_t, 37> phase2 = {3, 4, 5, 6, 8, 9, 10, 11, 7, 4, 5, 6, 8, 9, 10, 11, 5, 6, 8, 9, 10, 11, 6, 8, 9, 10, 11, 8, 9, 10, 11, 9, 10, 11, 10, 11, 11};

    // compute delay array index for given PRN number
    prn_idx = _prn - 1;

    // A simple error check
    if ((prn_idx < 0) || (prn_idx > 32))
        {
            return;
        }

    // Generate G1 & G2 Register
    for (lcv = 0; lcv < _code_length; lcv++)
        {
            G1[lcv] = G1_register[0];
            G2[lcv] = G2_register[-(phase1[prn_idx] - 11)] xor G2_register[-(phase2[prn_idx] - 11)];

            feedback1 = G1_register[0] xor G1_register[1] xor G1_register[2] xor G1_register[3] xor G1_register[4] xor G1_register[10];
            feedback2 = G2_register[0] xor G2_register[2] xor G2_register[3] xor G2_register[6] xor G2_register[7] xor G2_register[8] xor G2_register[9] xor G2_register[10];

            for (lcv2 = 0; lcv2 < 10; lcv2++)
                {
                    G1_register[lcv2] = G1_register[lcv2 + 1];
                    G2_register[lcv2] = G2_register[lcv2 + 1];
                }

            G1_register[10] = feedback1;
            G2_register[10] = feedback2;
        }

    // Set the delay
    delay = _code_length - delays[prn_idx] * 0;  //**********************************
    delay += _chip_shift;
    delay %= _code_length;

    // Generate PRN from G1 and G2 Registers
    for (lcv = 0; lcv < _code_length; lcv++)
        {
            aux = G1[(lcv + _chip_shift) % _code_length] xor G2[delay];
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


void beidou_b1i_code_gen_float(gsl::span<float> _dest, int32_t _prn, uint32_t _chip_shift)
{
    const uint32_t _code_length = 2046;
    std::array<int32_t, _code_length> b1i_code_int{};

    beidou_b1i_code_gen_int(gsl::span<int32_t>(b1i_code_int.data(), _code_length), _prn, _chip_shift);

    for (uint32_t ii = 0; ii < _code_length; ++ii)
        {
            _dest[ii] = static_cast<float>(b1i_code_int[ii]);
        }
}


void beidou_b1i_code_gen_complex(gsl::span<std::complex<float>> _dest, int32_t _prn, uint32_t _chip_shift)
{
    const uint32_t _code_length = 2046;
    std::array<int32_t, _code_length> b1i_code_int{};

    beidou_b1i_code_gen_int(gsl::span<int32_t>(b1i_code_int.data(), _code_length), _prn, _chip_shift);

    for (uint32_t ii = 0; ii < _code_length; ++ii)
        {
            _dest[ii] = std::complex<float>(static_cast<float>(b1i_code_int[ii]), 0.0F);
        }
}


/*
 *  Generates complex GPS L1 C/A code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1i_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs, uint32_t _chip_shift)
{
    // This function is based on the GNU software GPS for MATLAB in the Kay Borre book
    std::array<std::complex<float>, 2046> _code{};
    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    float aux;
    const int32_t _codeFreqBasis = 2046000;  // Hz
    const int32_t _codeLength = 2046;

    // --- Find number of samples per spreading code ---------------------------
    _samplesPerCode = static_cast<int32_t>(static_cast<double>(_fs) / static_cast<double>(_codeFreqBasis / _codeLength));

    // --- Find time constants -------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);             // Sampling period in sec
    _tc = 1.0 / static_cast<float>(_codeFreqBasis);  // C/A chip period in sec

    beidou_b1i_code_gen_complex(_code, _prn, _chip_shift);  // generate C/A code 1 sample per chip

    for (int32_t i = 0; i < _samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read C/A code values --------------------
            // The length of the index array depends on the sampling frequency -
            // number of samples per millisecond (because one C/A code period is one
            // millisecond).

            aux = (_ts * (i + 1)) / _tc;
            _codeValueIndex = auxCeil(aux) - 1;

            // --- Make the digitized version of the C/A code ------------------
            // The "upsampled" code is made by selecting values form the CA code
            // chip array (caCode) for the time instances of each sample.
            if (i == _samplesPerCode - 1)
                {
                    // --- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = _code[_codeLength - 1];
                }
            else
                {
                    _dest[i] = _code[_codeValueIndex];  // repeat the chip -> upsample
                }
        }
}
