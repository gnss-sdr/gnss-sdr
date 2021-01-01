/*!
 * \file gps_l2c_signal_replica.cc
 * \brief This file implements signal generators for GPS L2C signals
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "gps_l2c_signal_replica.h"
#include "GPS_L2C.h"
#include <array>
#include <cmath>
#include <memory>


uint32_t gps_l2c_m_shift(uint32_t x)
{
    return static_cast<uint32_t>((x >> 1U) xor ((x & 1U) * 0445112474U));
}


void gps_l2c_m_code(own::span<int32_t> dest, uint32_t prn)
{
    uint32_t x = GPS_L2C_M_INIT_REG[prn - 1];
    for (int32_t n = 0; n < GPS_L2_M_CODE_LENGTH_CHIPS; n++)
        {
            dest[n] = static_cast<int32_t>(x & 1U);
            x = gps_l2c_m_shift(x);
        }
}


void gps_l2c_m_code_gen_complex(own::span<std::complex<float>> dest, uint32_t prn)
{
    std::array<int32_t, GPS_L2_M_CODE_LENGTH_CHIPS> code_aux{};
    if (prn > 0 and prn < 51)
        {
            gps_l2c_m_code(code_aux, prn);
        }

    for (int32_t i = 0; i < GPS_L2_M_CODE_LENGTH_CHIPS; i++)
        {
            dest[i] = std::complex<float>(0.0, 1.0F - 2.0F * code_aux[i]);
        }
}


void gps_l2c_m_code_gen_float(own::span<float> dest, uint32_t prn)
{
    std::array<int32_t, GPS_L2_M_CODE_LENGTH_CHIPS> code_aux{};
    if (prn > 0 and prn < 51)
        {
            gps_l2c_m_code(code_aux, prn);
        }

    for (int32_t i = 0; i < GPS_L2_M_CODE_LENGTH_CHIPS; i++)
        {
            dest[i] = 1.0 - 2.0 * static_cast<float>(code_aux[i]);
        }
}


/*
 *  Generates complex GPS L2C M code for the desired SV ID and sampled to specific sampling frequency
 */
void gps_l2c_m_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    constexpr int32_t codeLength = GPS_L2_M_CODE_LENGTH_CHIPS;
    constexpr float tc = 1.0F / static_cast<float>(GPS_L2_M_CODE_RATE_CPS);  // L2C chip period in sec

    const auto samplesPerCode = static_cast<int32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(GPS_L2_M_CODE_RATE_CPS) / static_cast<double>(codeLength)));
    const float ts = 1.0F / static_cast<float>(sampling_freq);  // Sampling period in sec
    int32_t codeValueIndex;

    std::array<int32_t, GPS_L2_M_CODE_LENGTH_CHIPS> code_aux{};
    if (prn > 0 and prn < 51)
        {
            gps_l2c_m_code(code_aux, prn);
        }

    for (int32_t i = 0; i < samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read L2C code values --------------------
            codeValueIndex = std::ceil((ts * (static_cast<float>(i) + 1.0F)) / tc) - 1;

            // --- Make the digitized version of the L2C code ------------------
            if (i == samplesPerCode - 1)
                {
                    // Correct the last index (due to number rounding issues)
                    dest[i] = std::complex<float>(0.0, 1.0F - 2.0F * code_aux[codeLength - 1]);
                }
            else
                {
                    dest[i] = std::complex<float>(0.0, 1.0F - 2.0F * code_aux[codeValueIndex]);  // repeat the chip -> upsample
                }
        }
}
