/*!
 * \file gps_l2c_signal.cc
 * \brief This class implements signal generators for the GPS L2C signals
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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

#include "gps_l2c_signal.h"
#include "GPS_L2C.h"
#include <cstdint>
#include <cmath>


int32_t gps_l2c_m_shift(int32_t x)
{
    return static_cast<int32_t>((x >> 1)^((x & 1) * 0445112474));
}


void gps_l2c_m_code(int32_t * _dest, unsigned int _prn)
{
    int32_t x;
    x = GPS_L2C_M_INIT_REG[ _prn - 1];
    for (int n = 0; n < GPS_L2_M_CODE_LENGTH_CHIPS; n++)
        {
            _dest[n] = static_cast<int8_t>(x&1);
            x = gps_l2c_m_shift(x);
        }
}


void gps_l2c_m_code_gen_complex(std::complex<float>* _dest, unsigned int _prn)
{
    int32_t* _code = new int32_t[GPS_L2_M_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 51)
        {
            gps_l2c_m_code(_code, _prn);
        }

    for (signed int i = 0; i < GPS_L2_M_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * _code[i], 0.0);
        }

    delete[] _code;
}


/*
 *  Generates complex GPS L2C M code for the desired SV ID and sampled to specific sampling frequency
 */
void gps_l2c_m_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs)
{
    int32_t* _code = new int32_t[GPS_L2_M_CODE_LENGTH_CHIPS];
    if (_prn > 0 and _prn < 51)
        {
            gps_l2c_m_code(_code, _prn);
        }

    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const signed int _codeLength = GPS_L2_M_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(GPS_L2_M_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(GPS_L2_M_CODE_RATE_HZ);  // C/A chip period in sec

    //float aux;
    for (signed int i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read L2C code values -------------------------
            //TODO: Check this formula! Seems to start with an extra sample
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;
            //aux = (_ts * (i + 1)) / _tc;
            //_codeValueIndex = static_cast<int>(static_cast<long>(aux)) - 1;

            //--- Make the digitized version of the L2C code -----------------------
            if (i == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeLength - 1], 0);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeValueIndex], 0); //repeat the chip -> upsample
                }
        }
    delete[] _code;
}




