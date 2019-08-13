/*!
 * \file gps_l2c_signal.cc
 * \brief This class implements signal generators for the GPS L2C signals
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "gps_l2c_signal.h"
#include "GPS_L2C.h"
#include <array>
#include <cmath>
#include <memory>


uint32_t gps_l2c_m_shift(uint32_t x)
{
    return static_cast<uint32_t>((x >> 1U) xor ((x & 1U) * 0445112474U));
}


void gps_l2c_m_code(gsl::span<int32_t> _dest, uint32_t _prn)
{
    uint32_t x;
    x = GPS_L2C_M_INIT_REG[_prn - 1];
    for (int32_t n = 0; n < GPS_L2_M_CODE_LENGTH_CHIPS; n++)
        {
            _dest[n] = static_cast<int8_t>(x & 1U);
            x = gps_l2c_m_shift(x);
        }
}


void gps_l2c_m_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    std::array<int32_t, GPS_L2_M_CODE_LENGTH_CHIPS> _code{};
    if (_prn > 0 and _prn < 51)
        {
            gps_l2c_m_code(_code, _prn);
        }

    for (int32_t i = 0; i < GPS_L2_M_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * _code[i], 0.0);
        }
}


void gps_l2c_m_code_gen_float(gsl::span<float> _dest, uint32_t _prn)
{
    std::array<int32_t, GPS_L2_M_CODE_LENGTH_CHIPS> _code{};
    if (_prn > 0 and _prn < 51)
        {
            gps_l2c_m_code(_code, _prn);
        }

    for (int32_t i = 0; i < GPS_L2_M_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = 1.0 - 2.0 * static_cast<float>(_code[i]);
        }
}


/*
 *  Generates complex GPS L2C M code for the desired SV ID and sampled to specific sampling frequency
 */
void gps_l2c_m_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    std::array<int32_t, GPS_L2_M_CODE_LENGTH_CHIPS> _code{};
    if (_prn > 0 and _prn < 51)
        {
            gps_l2c_m_code(_code, _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = GPS_L2_M_CODE_LENGTH_CHIPS;

    // --- Find number of samples per spreading code ---------------------------
    _samplesPerCode = static_cast<int32_t>(static_cast<double>(_fs) / (static_cast<double>(GPS_L2_M_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    // --- Find time constants -------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                    // Sampling period in sec
    _tc = 1.0 / static_cast<float>(GPS_L2_M_CODE_RATE_HZ);  // L2C chip period in sec

    for (int32_t i = 0; i < _samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read L2C code values --------------------
            _codeValueIndex = std::ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            // --- Make the digitized version of the L2C code ------------------
            if (i == _samplesPerCode - 1)
                {
                    // --- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeLength - 1], 0);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeValueIndex], 0);  // repeat the chip -> upsample
                }
        }
}
