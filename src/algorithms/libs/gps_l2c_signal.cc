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
 * SPDX-License-Identifier: GPL-3.0-or-later
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


void gps_l2c_m_code(own::span<int32_t> _dest, uint32_t _prn)
{
    uint32_t x = GPS_L2C_M_INIT_REG[_prn - 1];
    for (int32_t n = 0; n < GPS_L2_M_CODE_LENGTH_CHIPS; n++)
        {
            _dest[n] = static_cast<int8_t>(x & 1U);
            x = gps_l2c_m_shift(x);
        }
}


void gps_l2c_m_code_gen_complex(own::span<std::complex<float>> _dest, uint32_t _prn)
{
    std::array<int32_t, GPS_L2_M_CODE_LENGTH_CHIPS> _code{};
    if (_prn > 0 and _prn < 51)
        {
            gps_l2c_m_code(_code, _prn);
        }

    for (int32_t i = 0; i < GPS_L2_M_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(0.0, 1.0F - 2.0F * _code[i]);
        }
}


void gps_l2c_m_code_gen_float(own::span<float> _dest, uint32_t _prn)
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
void gps_l2c_m_code_gen_complex_sampled(own::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    constexpr int32_t _codeLength = GPS_L2_M_CODE_LENGTH_CHIPS;
    constexpr float _tc = 1.0F / static_cast<float>(GPS_L2_M_CODE_RATE_CPS);  // L2C chip period in sec

    const auto _samplesPerCode = static_cast<int32_t>(static_cast<double>(_fs) / (static_cast<double>(GPS_L2_M_CODE_RATE_CPS) / static_cast<double>(_codeLength)));
    const float _ts = 1.0F / static_cast<float>(_fs);  // Sampling period in sec
    int32_t _codeValueIndex;

    std::array<int32_t, GPS_L2_M_CODE_LENGTH_CHIPS> _code{};
    if (_prn > 0 and _prn < 51)
        {
            gps_l2c_m_code(_code, _prn);
        }

    for (int32_t i = 0; i < _samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read L2C code values --------------------
            _codeValueIndex = std::ceil((_ts * (static_cast<float>(i) + 1.0F)) / _tc) - 1;

            // --- Make the digitized version of the L2C code ------------------
            if (i == _samplesPerCode - 1)
                {
                    // --- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(0.0, 1.0F - 2.0F * _code[_codeLength - 1]);
                }
            else
                {
                    _dest[i] = std::complex<float>(0.0, 1.0F - 2.0F * _code[_codeValueIndex]);  // repeat the chip -> upsample
                }
        }
}
