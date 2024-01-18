/*!
 * \file gps_l5_signal_replica.cc
 * \brief This file implements signal generators for GPS L5 signals
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
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

#include "gps_l5_signal_replica.h"
#include "GPS_L5.h"
#include <array>
#include <cmath>
#include <deque>

std::deque<bool> l5i_xa_shift(const std::deque<bool>& xa)  // GPS-IS-705E Figure 3-4 pp. 15
{
    if (xa == std::deque<bool>{true, true, true, true, true, true, true, true, true, true, true, false, true})
        {
            return std::deque<bool>{true, true, true, true, true, true, true, true, true, true, true, true, true};
        }
    std::deque<bool> out(xa.cbegin(), xa.cend() - 1);
    out.push_front(xa[12] xor xa[11] xor xa[9] xor xa[8]);
    return out;
}


std::deque<bool> l5q_xa_shift(const std::deque<bool>& xa)
{
    if (xa == std::deque<bool>{true, true, true, true, true, true, true, true, true, true, true, false, true})
        {
            return std::deque<bool>{true, true, true, true, true, true, true, true, true, true, true, true, true};
        }
    std::deque<bool> out(xa.cbegin(), xa.cend() - 1);
    out.push_front(xa[12] xor xa[11] xor xa[9] xor xa[8]);
    return out;
}


std::deque<bool> l5i_xb_shift(const std::deque<bool>& xb)  // GPS-IS-705E Figure 3-5 pp. 16
{
    std::deque<bool> out(xb.cbegin(), xb.cend() - 1);
    out.push_front(xb[12] xor xb[11] xor xb[7] xor xb[6] xor xb[5] xor xb[3] xor xb[2] xor xb[0]);
    return out;
}


std::deque<bool> l5q_xb_shift(const std::deque<bool>& xb)
{
    std::deque<bool> out(xb.cbegin(), xb.cend() - 1);
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


void make_l5i(own::span<int32_t> dest, int32_t prn)
{
    const int32_t xb_offset = GPS_L5I_INIT_REG[prn];

    const std::deque<bool> xb = make_l5i_xb();
    const std::deque<bool> xa = make_l5i_xa();
    std::deque<bool> xb_shift(GPS_L5I_CODE_LENGTH_CHIPS, false);

    for (int32_t n = 0; n < GPS_L5I_CODE_LENGTH_CHIPS; n++)
        {
            xb_shift[n] = xb[(xb_offset + n) % GPS_L5I_CODE_LENGTH_CHIPS];
        }

    for (int32_t n = 0; n < GPS_L5I_CODE_LENGTH_CHIPS; n++)
        {
            dest[n] = xa[n] xor xb_shift[n];
        }
}


void make_l5q(own::span<int32_t> dest, int32_t prn)
{
    const int32_t xb_offset = GPS_L5Q_INIT_REG[prn];

    const std::deque<bool> xb = make_l5q_xb();
    const std::deque<bool> xa = make_l5q_xa();
    std::deque<bool> xb_shift(GPS_L5Q_CODE_LENGTH_CHIPS, false);

    for (int32_t n = 0; n < GPS_L5Q_CODE_LENGTH_CHIPS; n++)
        {
            xb_shift[n] = xb[(xb_offset + n) % GPS_L5Q_CODE_LENGTH_CHIPS];
        }

    for (int32_t n = 0; n < GPS_L5Q_CODE_LENGTH_CHIPS; n++)
        {
            dest[n] = xa[n] xor xb_shift[n];
        }
}


void gps_l5i_code_gen_complex(own::span<std::complex<float>> dest, uint32_t prn)
{
    std::array<int32_t, GPS_L5I_CODE_LENGTH_CHIPS> code_aux{};
    if (prn > 0 and prn < 51)
        {
            make_l5i(code_aux, prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5I_CODE_LENGTH_CHIPS; i++)
        {
            dest[i] = std::complex<float>(1.0F - 2.0F * static_cast<float>(code_aux[i]), 0.0);
        }
}


void gps_l5i_code_gen_float(own::span<float> dest, uint32_t prn)
{
    std::array<int32_t, GPS_L5I_CODE_LENGTH_CHIPS> code_aux{};
    if (prn > 0 and prn < 51)
        {
            make_l5i(code_aux, prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5I_CODE_LENGTH_CHIPS; i++)
        {
            dest[i] = 1.0F - 2.0F * static_cast<float>(code_aux[i]);
        }
}


/*
 *  Generates complex GPS L5i code for the desired SV ID and sampled to specific sampling frequency
 */
void gps_l5i_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    constexpr int32_t codeLength = GPS_L5I_CODE_LENGTH_CHIPS;
    constexpr float tc = 1.0 / static_cast<float>(GPS_L5I_CODE_RATE_CPS);  // L5I primary chip period in sec

    const auto samplesPerCode = static_cast<int32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(GPS_L5I_CODE_RATE_CPS) / static_cast<double>(codeLength)));
    const float ts = 1.0F / static_cast<float>(sampling_freq);  // Sampling period in sec
    int32_t codeValueIndex;

    std::array<int32_t, GPS_L5I_CODE_LENGTH_CHIPS> code_aux{};
    if (prn > 0 and prn < 51)
        {
            make_l5i(code_aux, prn - 1);
        }

    for (int32_t i = 0; i < samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read L5 code values ---------------------
            codeValueIndex = static_cast<int32_t>(std::ceil(ts * (i + 1.0F) / tc)) - 1;

            // --- Make the digitized version of the L5I code ------------------
            if (i == samplesPerCode - 1)
                {
                    // --- Correct the last index (due to number rounding issues) -----------
                    dest[i] = std::complex<float>(1.0F - 2.0F * code_aux[codeLength - 1], 0.0);
                }
            else
                {
                    dest[i] = std::complex<float>(1.0F - 2.0F * code_aux[codeValueIndex], 0.0);  // repeat the chip -> upsample
                }
        }
}


void gps_l5q_code_gen_complex(own::span<std::complex<float>> dest, uint32_t prn)
{
    std::array<int32_t, GPS_L5Q_CODE_LENGTH_CHIPS> code_aux{};
    if (prn > 0 and prn < 51)
        {
            make_l5q(code_aux, prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5Q_CODE_LENGTH_CHIPS; i++)
        {
            dest[i] = std::complex<float>(0.0, 1.0F - 2.0F * static_cast<float>(code_aux[i]));
        }
}


void gps_l5q_code_gen_float(own::span<float> dest, uint32_t prn)
{
    std::array<int32_t, GPS_L5Q_CODE_LENGTH_CHIPS> code_aux{};
    if (prn > 0 and prn < 51)
        {
            make_l5q(code_aux, prn - 1);
        }

    for (int32_t i = 0; i < GPS_L5Q_CODE_LENGTH_CHIPS; i++)
        {
            dest[i] = 1.0 - 2.0 * static_cast<float>(code_aux[i]);
        }
}


/*
 *  Generates complex GPS L5Q code for the desired SV ID and sampled to specific sampling frequency
 */
void gps_l5q_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq)
{
    std::array<int32_t, GPS_L5Q_CODE_LENGTH_CHIPS> code_aux{};
    if (prn > 0 and prn < 51)
        {
            make_l5q(code_aux, prn - 1);
        }

    int32_t codeValueIndex;
    constexpr int32_t codeLength = GPS_L5Q_CODE_LENGTH_CHIPS;

    // --- Find number of samples per spreading code ---------------------------
    const auto samplesPerCode = static_cast<int32_t>(static_cast<double>(sampling_freq) / (static_cast<double>(GPS_L5Q_CODE_RATE_CPS) / static_cast<double>(codeLength)));

    // --- Find time constants -------------------------------------------------
    const float ts = 1.0F / static_cast<float>(sampling_freq);              // Sampling period in sec
    constexpr float tc = 1.0F / static_cast<float>(GPS_L5Q_CODE_RATE_CPS);  // L5Q chip period in sec

    for (int32_t i = 0; i < samplesPerCode; i++)
        {
            // === Digitizing ==================================================

            // --- Make index array to read L5 code values ---------------------
            codeValueIndex = static_cast<int32_t>(std::ceil(ts * (i + 1.0F) / tc)) - 1;

            // --- Make the digitized version of the L5Q code ------------------
            if (i == samplesPerCode - 1)
                {
                    // --- Correct the last index (due to number rounding issues) -----------
                    dest[i] = std::complex<float>(0.0, 1.0F - 2.0F * code_aux[codeLength - 1]);
                }
            else
                {
                    dest[i] = std::complex<float>(0.0, 1.0F - 2.0F * code_aux[codeValueIndex]);  // repeat the chip -> upsample
                }
        }
}
