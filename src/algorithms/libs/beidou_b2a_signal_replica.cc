/*!
 * \file beidou_b2a_signal_replica.cc
 * \brief This class implements signal generators for the BEIDOU B2a signals
 * \author Sara Hrbek, 2018. sara.hrbek(at)gmail.com.
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
 * \note Initial code added as part of GSoC 2018 program
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

#include "beidou_b2a_signal_replica.h"
#include "Beidou_B2a.h"
#include <cinttypes>
#include <cmath>
#include <complex>
#include <deque>
#include <fstream>
#include <iostream>


std::deque<bool> b2ad_g1_shift(std::deque<bool> g1)
{
    // Polynomial: G1 = 1 + x + x^5 + x^11 + x^13;
    std::deque<bool> out(g1.begin(), g1.end() - 1);
    out.push_front(g1[12] xor g1[10] xor g1[4] xor g1[0]);
    return out;
}


std::deque<bool> b2ap_g1_shift(std::deque<bool> g1)
{
    // Polynomial: G1 = 1 + x^3 + x^6 + x^7 + x^13;
    std::deque<bool> out(g1.begin(), g1.end() - 1);
    out.push_front(g1[12] xor g1[6] xor g1[5] xor g1[2]);
    return out;
}


std::deque<bool> b2ad_g2_shift(std::deque<bool> g2)
{
    // Polynomial: G2 = 1 + x^3 + x^5 + x^9 + x^11 +x^12 +x^13;
    std::deque<bool> out(g2.begin(), g2.end() - 1);
    out.push_front(g2[12] xor g2[11] xor g2[10] xor g2[8] xor g2[4] xor g2[2]);
    return out;
}

std::deque<bool> b2ap_g2_shift(std::deque<bool> g2)
{
    //Polynomial: G2 = 1 + x + x^5 + x^7 gsco+ x^8 +x^12 +x^13;
    std::deque<bool> out(g2.begin(), g2.end() - 1);
    out.push_front(g2[12] xor g2[11] xor g2[7] xor g2[6] xor g2[4] xor g2[0]);
    return out;
}

// Make the B2a data G1 sequence
std::deque<bool> make_b2ad_g1()
{
    std::deque<bool> g1 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    std::deque<bool> g1_seq(BEIDOU_B2ad_CODE_LENGTH_CHIPS, 0);

    for (uint32_t i = 0; i < BEIDOU_B2ad_CODE_LENGTH_CHIPS; i++)
        {
            g1_seq[i] = g1[12];
            g1 = b2ad_g1_shift(g1);
            // reset the g1 register
            if (i == 8189)
                {
                    g1 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
                }
        }
    return g1_seq;
}

// Make the B2a data G2 sequence.
std::deque<bool> make_b2ad_g2(std::deque<bool> g2)
{
    std::deque<bool> g2_seq(BEIDOU_B2ad_CODE_LENGTH_CHIPS, 0);

    for (uint32_t i = 0; i < BEIDOU_B2ad_CODE_LENGTH_CHIPS; i++)
        {
            g2_seq[i] = g2[12];
            g2 = b2ad_g2_shift(g2);
        }
    return g2_seq;
}

// Make the B2a pilot G1 sequence
std::deque<bool> make_b2ap_g1()
{
    std::deque<bool> g1 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    std::deque<bool> g1_seq(BEIDOU_B2ap_CODE_LENGTH_CHIPS, 0);

    for (uint32_t i = 0; i < BEIDOU_B2ap_CODE_LENGTH_CHIPS; i++)
        {
            g1_seq[i] = g1[12];
            g1 = b2ap_g1_shift(g1);

            //reset the g1 register
            if (i == 8189)
                {
                    g1 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
                }
        }

    return g1_seq;
}

// Make the B2a pilot G2 sequence
std::deque<bool> make_b2ap_g2(std::deque<bool> g2)
{
    std::deque<bool> g2_seq(BEIDOU_B2ap_CODE_LENGTH_CHIPS, 0);

    for (uint32_t i = 0; i < BEIDOU_B2ap_CODE_LENGTH_CHIPS; i++)
        {
            g2_seq[i] = g2[12];
            g2 = b2ap_g2_shift(g2);
        }
    return g2_seq;
}

// Make legendre sequence for secondary pilot codes
bool make_leg(int32_t k)
{
    bool squaremodp = false;
    int32_t z = 1;

    if (k == 0)
        {
            squaremodp = true;
        }
    else
        {
            while (z <= (BEIDOU_B2ap_SECONDARY_WEIL_CODE_LENGTH - 1) / 2)
                {
                    squaremodp = false;
                    if (((z * z) % BEIDOU_B2ap_SECONDARY_WEIL_CODE_LENGTH) == k)
                        {
                            // k is a square(mod p)
                            squaremodp = true;
                            break;
                        }
                    z = z + 1;
                }
        }

    return squaremodp;
}

// Make B2a Pilot secondary code
std::deque<bool> make_b2ap_secondary_weil_seq(int32_t w, int32_t p)
{
    int32_t n, k = 0;
    int32_t N = BEIDOU_B2ap_SECONDARY_WEIL_CODE_LENGTH;
    std::deque<bool> trunc_weil_seq(BEIDOU_B2ap_SECONDARY_CODE_LENGTH, 0);

    // Generate only the truncated sequence
    for (n = 0; n < BEIDOU_B2ap_SECONDARY_CODE_LENGTH; n++)
        {
            k = (n + p - 1) % N;
            trunc_weil_seq[n] = make_leg(k) xor make_leg((k + w) % N);
        }

    return trunc_weil_seq;
}

// Generate the B2a data PRN codes
void make_b2ad(gsl::span<int32_t> _dest, int32_t prn)
{
    std::deque<bool> g2 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    for (int i = 0; i < 13; i++)
        {
            g2[i] = BEIDOU_B2ad_INIT_REG[prn - 1][i];
        }
    std::deque<bool> g1 = make_b2ad_g1();
    g2 = make_b2ad_g2(g2);

    for (uint32_t n = 0; n < BEIDOU_B2ad_CODE_LENGTH_CHIPS; n++)
        {
            _dest[n] = g1[n] xor g2[n];
        }
}

// Generate a version of the B2a Data code with the secondary code included
void make_b2ad_secondary(gsl::span<int32_t> _dest, int32_t prn)
{
    // This is specific to the Beidou B2a data code
    bool Secondary1 = 0;
    bool Secondary2 = 1;
    std::deque<bool> g2 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    for (int i = 0; i < 13; i++)
        {
            g2[i] = BEIDOU_B2ad_INIT_REG[prn - 1][i];
        }
    std::deque<bool> g1 = make_b2ad_g1();
    g2 = make_b2ad_g2(g2);

    for (uint32_t m = 0; m < BEIDOU_B2ad_SECONDARY_CODE_LENGTH; m++)
        {
            for (uint32_t n = 0; n < BEIDOU_B2ad_CODE_LENGTH_CHIPS; n++)
                {
                    if (m == 3)
                        {
                            _dest[n + m * BEIDOU_B2ad_CODE_LENGTH_CHIPS] = (g1[n] xor g2[n]) xor Secondary2;
                        }
                    else
                        {
                            _dest[n + m * BEIDOU_B2ad_CODE_LENGTH_CHIPS] = (g1[n] xor g2[n]) xor Secondary1;
                        }
                }
        }
}

// Generate the B2a pilot code
void make_b2ap(gsl::span<int32_t> _dest, int32_t prn)
{
    std::deque<bool> g2 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    for (int i = 0; i < 13; i++)
        {
            g2[i] = BEIDOU_B2ap_INIT_REG[prn - 1][i];
        }
    std::deque<bool> g1 = make_b2ap_g1();
    g2 = make_b2ap_g2(g2);

    for (uint32_t n = 0; n < BEIDOU_B2ap_CODE_LENGTH_CHIPS; n++)
        {
            _dest[n] = g1[n] xor g2[n];
        }
}

// Generate a version of the B2a Pilot code with the secondary code included
void make_b2ap_secondary(gsl::span<int32_t> _dest, int32_t prn)
{
    int32_t phase_diff = BEIDOU_B2ap_SECONDARY_PHASE_DIFFERENCE[prn - 1];
    int32_t truncation_point = BEIDOU_B2ap_SECONDARY_TRUNCATION_POINT[prn - 1];

    // Generate primary code
    std::deque<bool> g2 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    for (uint32_t i = 0; i < 13; i++)
        {
            g2[i] = BEIDOU_B2ap_INIT_REG[prn - 1][i];
        }
    std::deque<bool> g1 = make_b2ap_g1();
    g2 = make_b2ap_g2(g2);

    // Generate secondary code
    std::deque<bool> b2ap_sec_code = make_b2ap_secondary_weil_seq(phase_diff, truncation_point);

    for (uint32_t m = 0; m < BEIDOU_B2ap_SECONDARY_CODE_LENGTH; m++)
        {
            for (uint32_t n = 0; n < BEIDOU_B2ap_CODE_LENGTH_CHIPS; n++)
                {
                    _dest[n + m * BEIDOU_B2ap_CODE_LENGTH_CHIPS] = (g1[n] xor g2[n]) xor b2ap_sec_code[m];
                }
        }
}

// Generate a complex version of the B2a data code with the Secondary code
void beidou_b2ad_code_gen_complex_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    uint32_t _code_length = BEIDOU_B2ad_CODE_LENGTH_CHIPS * BEIDOU_B2ad_SECONDARY_CODE_LENGTH;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ad_secondary(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = std::complex<float>(static_cast<float>(1.0 - 2.0 * _code[ii]), 0.0);
        }
}

// Generate a complex version of the B2a data code
void beidou_b2ad_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    uint32_t _code_length = BEIDOU_B2ad_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ad(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = std::complex<float>(static_cast<float>(1.0 - 2.0 * _code[ii]), 0.0);
        }
}

// Generate a float version of the B2a data code
void beidou_b2ad_code_gen_float(gsl::span<float> _dest, uint32_t _prn)
{
    uint32_t _code_length = BEIDOU_B2ad_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ad(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = static_cast<float>(1.0 - 2.0 * static_cast<float>(_code[ii]));
        }
}

/*
 *  Generates complex BEIDOU B2a data code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b2ad_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length = BEIDOU_B2ad_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn <= 63)
        {
            make_b2ad(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B2ad_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B2ad_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B2ad_CODE_RATE_HZ);  // code chip period in sec

    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B2a code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B2ad code -----------------------
            if (i == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeLength - 1], 0);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeValueIndex], 0);  //repeat the chip -> upsample
                }
        }
}

/*
 *  Generates complex BEIDOU B2a data code for the desired SV ID and sampled to specific sampling frequency with the secondary code implemented
 */
void beidou_b2ad_code_gen_complex_sampled_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length = BEIDOU_B2ad_CODE_LENGTH_CHIPS * BEIDOU_B2ad_SECONDARY_CODE_LENGTH;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn <= 63)
        {
            make_b2ad_secondary(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B2ad_CODE_LENGTH_CHIPS * BEIDOU_B2ad_SECONDARY_CODE_LENGTH;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B2ad_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B2ad_CODE_RATE_HZ);  // code chip period in sec

    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B2a code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B2ad code -----------------------
            if (i == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeLength - 1], 0);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeValueIndex], 0);  //repeat the chip -> upsample
                }
        }
}

// Generate a complex version of the B2a pilot code with the secondary code
void beidou_b2ap_code_gen_complex_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    uint32_t _code_length = BEIDOU_B2ap_CODE_LENGTH_CHIPS * BEIDOU_B2ap_SECONDARY_CODE_LENGTH;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ap_secondary(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = std::complex<float>(0.0, static_cast<float>(1.0 - 2.0 * _code[ii]));
        }
}

// Generates a complex version of the B2a pilot primary code
void beidou_b2ap_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    uint32_t _code_length = BEIDOU_B2ap_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ap(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    for (int32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = std::complex<float>(0.0, static_cast<float>(1.0 - 2.0 * _code[ii]));
        }
}

/*
 * Generates a float version of the B2a pilot primary code
 */
void beidou_b2ap_code_gen_float(gsl::span<float> _dest, uint32_t _prn)
{
    uint32_t _code_length = BEIDOU_B2ap_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ap(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = static_cast<float>(1.0 - 2.0 * static_cast<float>(_code[ii]));
        }
}

/*
 * Generates a float version of the B2a pilot primary code
 */
void beidou_b2ap_code_gen_float_secondary(gsl::span<float> _dest, uint32_t _prn)
{
    uint32_t _code_length = BEIDOU_B2ap_CODE_LENGTH_CHIPS * BEIDOU_B2ap_SECONDARY_CODE_LENGTH;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ap_secondary(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = static_cast<float>(1.0 - 2.0 * static_cast<float>(_code[ii]));
        }
}

/*
 *  Generates complex BEIDOU B2a pilot code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b2ap_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length = BEIDOU_B2ap_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ap(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B2ap_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B2ap_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B2ap_CODE_RATE_HZ);  // C/A chip period in sec

    //float aux;
    for (uint32_t ii = 0; ii < _samplesPerCode; ii++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B2a pilot code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B2a code -----------------------
            if (ii == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeLength - 1]);
                }
            else
                {
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeValueIndex]);  //repeat the chip -> upsample
                }
        }
}

/*
 *  Generates complex BEIDOU B2a data code for the desired SV ID and sampled to specific sampling frequency with the secondary code implemented
 */
void beidou_b2ap_code_gen_complex_sampled_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length = BEIDOU_B2ap_CODE_LENGTH_CHIPS * BEIDOU_B2ap_SECONDARY_CODE_LENGTH;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn <= 63)
        {
            make_b2ap_secondary(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B2ap_CODE_LENGTH_CHIPS * BEIDOU_B2ap_SECONDARY_CODE_LENGTH;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B2ap_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B2ap_CODE_RATE_HZ);  // code chip period in sec

    for (uint32_t ii = 0; ii < _samplesPerCode; ii++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B2a code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B2ad code -----------------------
            if (ii == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeLength - 1]);
                }
            else
                {
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeValueIndex]);  //repeat the chip -> upsample
                }
        }
}

void beidou_b2ap_secondary_gen_string(std::string& _dest, uint32_t _prn)
{
    int32_t phase_diff = BEIDOU_B2ap_SECONDARY_PHASE_DIFFERENCE[_prn - 1];
    int32_t truncation_point = BEIDOU_B2ap_SECONDARY_TRUNCATION_POINT[_prn - 1];

    // Generate secondary code
    std::deque<bool> b2ap_sec_code = make_b2ap_secondary_weil_seq(phase_diff, truncation_point);

    for (int32_t i = 0; i < BEIDOU_B2ap_SECONDARY_CODE_LENGTH; i++)
        {
            if (b2ap_sec_code[i] == true)
                _dest[i] = '1';
            else
                _dest[i] = '0';
        }
}

/*
 *  Generates complex BEIDOU B2a data+pilot code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b2a_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length_data = BEIDOU_B2ad_CODE_LENGTH_CHIPS;
    uint32_t _code_length_pilot = BEIDOU_B2ap_CODE_LENGTH_CHIPS;
    int32_t _code_pilot[_code_length_pilot];
    int32_t _code_data[_code_length_data];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ap(gsl::span<int32_t>(_code_pilot, _code_length_pilot), _prn);
            make_b2ad(gsl::span<int32_t>(_code_data, _code_length_data), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B2ap_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B2ap_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B2ap_CODE_RATE_HZ);  // C/A chip period in sec

    //float aux;
    for (uint32_t ii = 0; ii < _samplesPerCode; ii++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B2a pilot code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B2a code -----------------------
            if (ii == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[ii] = std::complex<float>(1.0 - 2.0 * _code_data[_codeLength - 1], 1.0 - 2.0 * _code_pilot[_codeLength - 1]);
                }
            else
                {
                    _dest[ii] = std::complex<float>(1.0 - 2.0 * _code_data[_codeValueIndex], 1.0 - 2.0 * _code_pilot[_codeValueIndex]);  //repeat the chip -> upsample
                }
        }
}