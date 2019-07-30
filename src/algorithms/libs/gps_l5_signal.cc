/*!
 * \file gps_l5_signal.cc
 * \brief This class implements signal generators for the GPS L5 signals
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
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

#include "gps_l5_signal.h"
#include "GPS_L5.h"
#include <array>
#include <cmath>
#include <deque>

std::deque<bool> l5i_xa_shift(std::deque<bool> xa)  // GPS-IS-705E Figure 3-4 pp. 15
{
    if (xa == std::deque<bool>{true, true, true, true, true, true, true, true, true, true, true, false, true})
        {
            return std::deque<bool>{true, true, true, true, true, true, true, true, true, true, true, true, true};
        }
    std::deque<bool> out(xa.begin(), xa.end() - 1);
    out.push_front(xa[12] xor xa[11] xor xa[9] xor xa[8]);
    return out;
}


std::deque<bool> l5q_xa_shift(std::deque<bool> xa)
{
    if (xa == std::deque<bool>{true, true, true, true, true, true, true, true, true, true, true, false, true})
        {
            return std::deque<bool>{true, true, true, true, true, true, true, true, true, true, true, true, true};
        }
    std::deque<bool> out(xa.begin(), xa.end() - 1);
    out.push_front(xa[12] xor xa[11] xor xa[9] xor xa[8]);
    return out;
}


std::deque<bool> l5i_xb_shift(std::deque<bool> xb)  // GPS-IS-705E Figure 3-5 pp. 16
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
    std::deque<bool> xa = {true, true, true, true, true, true, true, true, true, true, true, true, true};
    std::deque<bool> y(GPS_L5I_CODE_LENGTH_CHIPS, false);

    for (int32_t i = 0; i < GPS_L5I_CODE_LENGTH_CHIPS; i++)
        {
            y[i] = xa[12];
            xa = l5i_xa_shift(xa);
        }
    return y;
}


std::deque<bool> make_l5i_xb()
{
    std::deque<bool> xb = {true, true, true, true, true, true, true, true, true, true, true, true, true};
    std::deque<bool> y(GPS_L5I_CODE_LENGTH_CHIPS, false);

    for (int32_t i = 0; i < GPS_L5I_CODE_LENGTH_CHIPS; i++)
        {
            y[i] = xb[12];
            xb = l5i_xb_shift(xb);
        }
    return y;
}


std::deque<bool> make_l5q_xa()
{
    std::deque<bool> xa = {true, true, true, true, true, true, true, true, true, true, true, true, true};
    std::deque<bool> y(GPS_L5Q_CODE_LENGTH_CHIPS, false);

    for (int32_t i = 0; i < GPS_L5Q_CODE_LENGTH_CHIPS; i++)
        {
            y[i] = xa[12];
            xa = l5q_xa_shift(xa);
        }
    return y;
}


std::deque<bool> make_l5q_xb()
{
    std::deque<bool> xb = {true, true, true, true, true, true, true, true, true, true, true, true, true};
    std::deque<bool> y(GPS_L5Q_CODE_LENGTH_CHIPS, false);

    for (int32_t i = 0; i < GPS_L5Q_CODE_LENGTH_CHIPS; i++)
        {
            y[i] = xb[12];
            xb = l5q_xb_shift(xb);
        }
    return y;
}


void make_l5i(gsl::span<int32_t> _dest, int32_t prn)
{
    int32_t xb_offset = GPS_L5I_INIT_REG[prn];

    std::deque<bool> xb = make_l5i_xb();
    std::deque<bool> xa = make_l5i_xa();
    std::deque<bool> xb_shift(GPS_L5I_CODE_LENGTH_CHIPS, false);

    for (int32_t n = 0; n < GPS_L5I_CODE_LENGTH_CHIPS; n++)
        {
            xb_shift[n] = xb[(xb_offset + n) % GPS_L5I_CODE_LENGTH_CHIPS];
        }

    for (int32_t n = 0; n < GPS_L5I_CODE_LENGTH_CHIPS; n++)
        {
            _dest[n] = xa[n] xor xb_shift[n];
        }
}


void make_l5q(gsl::span<int32_t> _dest, int32_t prn)
{
    int32_t xb_offset = GPS_L5Q_INIT_REG[prn];

    std::deque<bool> xb = make_l5q_xb();
    std::deque<bool> xa = make_l5q_xa();
    std::deque<bool> xb_shift(GPS_L5Q_CODE_LENGTH_CHIPS, false);

    for (int32_t n = 0; n < GPS_L5Q_CODE_LENGTH_CHIPS; n++)
        {
            xb_shift[n] = xb[(xb_offset + n) % GPS_L5Q_CODE_LENGTH_CHIPS];
        }

    for (int32_t n = 0; n < GPS_L5Q_CODE_LENGTH_CHIPS; n++)
        {
            _dest[n] = xa[n] xor xb_shift[n];
        }
}


void gps_l5i_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    std::array<int32_t, GPS_L5I_CODE_LENGTH_CHIPS> _code{};
    if (_prn > 0 and _prn < 51)
        {
            make_l5i(_code, _prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5I_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * static_cast<float>(_code[i]), 0.0);
        }
}


void gps_l5i_code_gen_float(gsl::span<float> _dest, uint32_t _prn)
{
    std::array<int32_t, GPS_L5I_CODE_LENGTH_CHIPS> _code{};
    if (_prn > 0 and _prn < 51)
        {
            make_l5i(_code, _prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5I_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = 1.0 - 2.0 * static_cast<float>(_code[i]);
        }
}


/*
 *  Generates complex GPS L5i code for the desired SV ID and sampled to specific sampling frequency
 */
void gps_l5i_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    std::array<int32_t, GPS_L5I_CODE_LENGTH_CHIPS> _code{};
    if (_prn > 0 and _prn < 51)
        {
            make_l5i(_code, _prn - 1);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = GPS_L5I_CODE_LENGTH_CHIPS;

    // --- Find number of samples per spreading code ---------------------------
    _samplesPerCode = static_cast<int32_t>(static_cast<double>(_fs) / (static_cast<double>(GPS_L5I_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    // --- Find time constants -------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(GPS_L5I_CODE_RATE_HZ);  // L5I primary chip period in sec

    for (int32_t i = 0; i < _samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read L5 code values ---------------------
            _codeValueIndex = static_cast<int32_t>(std::ceil(_ts * static_cast<float>(i + 1) / _tc)) - 1;

            // --- Make the digitized version of the L5I code ------------------
            if (i == _samplesPerCode - 1)
                {
                    // --- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeLength - 1], 0.0);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeValueIndex], 0.0);  // repeat the chip -> upsample
                }
        }
}


void gps_l5q_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    std::array<int32_t, GPS_L5Q_CODE_LENGTH_CHIPS> _code{};
    if (_prn > 0 and _prn < 51)
        {
            make_l5q(_code, _prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5Q_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * static_cast<float>(_code[i]), 0.0);
        }
}


void gps_l5q_code_gen_float(gsl::span<float> _dest, uint32_t _prn)
{
    std::array<int32_t, GPS_L5Q_CODE_LENGTH_CHIPS> _code{};
    if (_prn > 0 and _prn < 51)
        {
            make_l5q(_code, _prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5Q_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = 1.0 - 2.0 * static_cast<float>(_code[i]);
        }
}


/*
 *  Generates complex GPS L5Q code for the desired SV ID and sampled to specific sampling frequency
 */
void gps_l5q_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    std::array<int32_t, GPS_L5Q_CODE_LENGTH_CHIPS> _code{};
    if (_prn > 0 and _prn < 51)
        {
            make_l5q(_code, _prn - 1);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = GPS_L5Q_CODE_LENGTH_CHIPS;

    // --- Find number of samples per spreading code ---------------------------
    _samplesPerCode = static_cast<int32_t>(static_cast<double>(_fs) / (static_cast<double>(GPS_L5Q_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    // --- Find time constants -------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(GPS_L5Q_CODE_RATE_HZ);  // L5Q chip period in sec

    for (int32_t i = 0; i < _samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read L5 code values ---------------------
            _codeValueIndex = static_cast<int32_t>(std::ceil(_ts * static_cast<float>(i + 1) / _tc)) - 1;

            // --- Make the digitized version of the L5Q code ------------------
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
