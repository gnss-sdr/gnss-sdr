/*!
 * \file gps_l5_signal.cc
 * \brief This class implements signal generators for the GPS L5 signals
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
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

#include "gps_l5_signal.h"
#include "GPS_L5.h"
#include <cinttypes>
#include <cmath>
#include <complex>
#include <deque>


std::deque<bool> l5i_xa_shift(std::deque<bool> xa)
{
    if (xa == std::deque<bool>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1})
        {
            return std::deque<bool>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
        }
    else
        {
            std::deque<bool> out(xa.begin(), xa.end() - 1);
            out.push_front(xa[12] xor xa[11] xor xa[9] xor xa[8]);
            return out;
        }
}


std::deque<bool> l5q_xa_shift(std::deque<bool> xa)
{
    if (xa == std::deque<bool>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1})
        {
            return std::deque<bool>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
        }
    else
        {
            std::deque<bool> out(xa.begin(), xa.end() - 1);
            out.push_front(xa[12] xor xa[11] xor xa[9] xor xa[8]);
            return out;
        }
}


std::deque<bool> l5i_xb_shift(std::deque<bool> xb)
{
    std::deque<bool> out(xb.begin(), xb.end() - 1);
    out.push_front(xb[12] xor xb[11] xor xb[7] xor xb[6] xor xb[5] xor xb[3] xor xb[2] xor xb[0]);
    return out;
}


std::deque<bool> l5q_xb_shift(std::deque<bool> xb)
{
    std::deque<bool> out(xb.begin(), xb.end() - 1);
    out.push_front(xb[12] xor xb[11] xor xb[7] xor xb[6] xor xb[5] xor xb[3] xor xb[2] xor xb[0]);
    return out;
}


std::deque<bool> make_l5i_xa()
{
    std::deque<bool> xa = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    std::deque<bool> y(GPS_L5i_CODE_LENGTH_CHIPS, 0);

    for (int32_t i = 0; i < GPS_L5i_CODE_LENGTH_CHIPS; i++)
        {
            y[i] = xa[12];
            xa = l5i_xa_shift(xa);
        }
    return y;
}


std::deque<bool> make_l5i_xb()
{
    std::deque<bool> xb = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    std::deque<bool> y(GPS_L5i_CODE_LENGTH_CHIPS, 0);

    for (int32_t i = 0; i < GPS_L5i_CODE_LENGTH_CHIPS; i++)
        {
            y[i] = xb[12];
            xb = l5i_xb_shift(xb);
        }
    return y;
}


std::deque<bool> make_l5q_xa()
{
    std::deque<bool> xa = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    std::deque<bool> y(GPS_L5q_CODE_LENGTH_CHIPS, 0);

    for (int32_t i = 0; i < GPS_L5q_CODE_LENGTH_CHIPS; i++)
        {
            y[i] = xa[12];
            xa = l5q_xa_shift(xa);
        }
    return y;
}


std::deque<bool> make_l5q_xb()
{
    std::deque<bool> xb = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    std::deque<bool> y(GPS_L5q_CODE_LENGTH_CHIPS, 0);

    for (int32_t i = 0; i < GPS_L5q_CODE_LENGTH_CHIPS; i++)
        {
            y[i] = xb[12];
            xb = l5q_xb_shift(xb);
        }
    return y;
}


void make_l5i(int32_t* _dest, int32_t prn)
{
    int32_t xb_offset = GPS_L5i_INIT_REG[prn];

    std::deque<bool> xb = make_l5i_xb();
    std::deque<bool> xa = make_l5i_xa();
    std::deque<bool> xb_shift(GPS_L5i_CODE_LENGTH_CHIPS, 0);

    for (int32_t n = 0; n < GPS_L5i_CODE_LENGTH_CHIPS; n++)
        {
            xb_shift[n] = xb[(xb_offset + n) % GPS_L5i_CODE_LENGTH_CHIPS];
        }
    std::deque<bool> out_code(GPS_L5i_CODE_LENGTH_CHIPS, 0);
    for (int32_t n = 0; n < GPS_L5i_CODE_LENGTH_CHIPS; n++)
        {
            _dest[n] = xa[n] xor xb_shift[n];
        }
}


void make_l5q(int32_t* _dest, int32_t prn)
{
    int32_t xb_offset = GPS_L5q_INIT_REG[prn];

    std::deque<bool> xb = make_l5q_xb();
    std::deque<bool> xa = make_l5q_xa();
    std::deque<bool> xb_shift(GPS_L5q_CODE_LENGTH_CHIPS, 0);

    for (int32_t n = 0; n < GPS_L5q_CODE_LENGTH_CHIPS; n++)
        {
            xb_shift[n] = xb[(xb_offset + n) % GPS_L5q_CODE_LENGTH_CHIPS];
        }
    std::deque<bool> out_code(GPS_L5q_CODE_LENGTH_CHIPS, 0);
    for (int32_t n = 0; n < GPS_L5q_CODE_LENGTH_CHIPS; n++)
        {
            _dest[n] = xa[n] xor xb_shift[n];
        }
}


void gps_l5i_code_gen_complex(std::complex<float>* _dest, uint32_t _prn)
{
    int32_t* _code = new int32_t[GPS_L5i_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 51)
        {
            make_l5i(_code, _prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5i_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * _code[i], 0.0);
        }

    delete[] _code;
}


void gps_l5i_code_gen_float(float* _dest, uint32_t _prn)
{
    int32_t* _code = new int32_t[GPS_L5i_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 51)
        {
            make_l5i(_code, _prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5i_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = 1.0 - 2.0 * static_cast<float>(_code[i]);
        }

    delete[] _code;
}


/*
 *  Generates complex GPS L5i code for the desired SV ID and sampled to specific sampling frequency
 */
void gps_l5i_code_gen_complex_sampled(std::complex<float>* _dest, uint32_t _prn, int32_t _fs)
{
    int32_t* _code = new int32_t[GPS_L5i_CODE_LENGTH_CHIPS];
    if (_prn > 0 and _prn < 51)
        {
            make_l5i(_code, _prn - 1);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = GPS_L5i_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int32_t>(static_cast<double>(_fs) / (static_cast<double>(GPS_L5i_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(GPS_L5i_CODE_RATE_HZ);  // C/A chip period in sec

    //float aux;
    for (int32_t i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read L5 code values -------------------------
            //TODO: Check this formula! Seems to start with an extra sample
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;
            //aux = (_ts * (i + 1)) / _tc;
            //_codeValueIndex = static_cast<int32_t> (static_cast<long>(aux)) - 1;

            //--- Make the digitized version of the L2C code -----------------------
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
    delete[] _code;
}


void gps_l5q_code_gen_complex(std::complex<float>* _dest, uint32_t _prn)
{
    int32_t* _code = new int32_t[GPS_L5q_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 51)
        {
            make_l5q(_code, _prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5q_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * _code[i], 0.0);
        }

    delete[] _code;
}


void gps_l5q_code_gen_float(float* _dest, uint32_t _prn)
{
    int32_t* _code = new int32_t[GPS_L5q_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 51)
        {
            make_l5q(_code, _prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5q_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = 1.0 - 2.0 * static_cast<float>(_code[i]);
        }

    delete[] _code;
}


/*
 *  Generates complex GPS L5i code for the desired SV ID and sampled to specific sampling frequency
 */
void gps_l5q_code_gen_complex_sampled(std::complex<float>* _dest, uint32_t _prn, int32_t _fs)
{
    int32_t* _code = new int32_t[GPS_L5q_CODE_LENGTH_CHIPS];
    if (_prn > 0 and _prn < 51)
        {
            make_l5q(_code, _prn - 1);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = GPS_L5q_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int32_t>(static_cast<double>(_fs) / (static_cast<double>(GPS_L5q_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(GPS_L5q_CODE_RATE_HZ);  // C/A chip period in sec

    //float aux;
    for (int32_t i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read L5 code values -------------------------
            //TODO: Check this formula! Seems to start with an extra sample
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;
            //aux = (_ts * (i + 1)) / _tc;
            //_codeValueIndex = static_cast<int32_t> (static_cast<long>(aux)) - 1;

            //--- Make the digitized version of the L2C code -----------------------
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
    delete[] _code;
}
