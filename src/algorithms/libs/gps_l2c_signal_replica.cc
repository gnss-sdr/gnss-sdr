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
    return ((x >> 1U) xor ((x & 1U) * 0445112474U));
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

void gps_l2c_m_code_gen_float_cl_zeroed(own::span<float> dest, uint32_t prn)
{
    // dest must be at least GPS_L2_M_CODE_LENGTH_CHIPS_L_ZEROED
    // Layout: [ CM0, 0, CM1, 0, ..., CM10229, 0 ]  -> 20460 "chips"

    std::array<int32_t, GPS_L2_M_CODE_LENGTH_CHIPS> code_aux{};

    if (prn > 0U && prn < 51U)
        {
            gps_l2c_m_code(code_aux, prn);
        }
    else
        {
            // out of range -> output zeros
            const int32_t n = std::min<int32_t>(static_cast<int32_t>(dest.size()),
                GPS_L2_M_CODE_LENGTH_CHIPS_L_ZEROED);
            for (int32_t i = 0; i < n; i++)
                {
                    dest[i] = 0.0F;
                }
            return;
        }

    const int32_t n = std::min<int32_t>(static_cast<int32_t>(dest.size()),
        GPS_L2_M_CODE_LENGTH_CHIPS_L_ZEROED);

    for (int32_t i = 0; i < n; i++)
        {
            const bool is_cm_slot = ((i & 1) == 0);  // even indices: CM, odd: CL (zeroed)

            if (is_cm_slot)
                {
                    const int32_t cm_idx = (i >> 1);  // i/2 in [0..10229]
                    dest[i] = 1.0F - 2.0F * static_cast<float>(code_aux[cm_idx]);
                }
            else
                {
                    dest[i] = 0.0F;  // CL slot zeroed
                }
        }
}


void gps_l2c_m_code_gen_complex_sampled(
    own::span<std::complex<float>> dest,
    uint32_t prn,
    int32_t sampling_freq,
    bool cl_zeroed)
{
    constexpr int32_t codeLength = GPS_L2_M_CODE_LENGTH_CHIPS;                  // 10230
    constexpr float tc_cm = 1.0F / static_cast<float>(GPS_L2_M_CODE_RATE_CPS);  // 511.5 kcps
    constexpr float tc_l2c = tc_cm * 0.5F;                                      // 1.023 Mcps (half-chip)

    const float ts = 1.0F / static_cast<float>(sampling_freq);

    // --- Load CM (M-code) ---
    std::array<int32_t, GPS_L2_M_CODE_LENGTH_CHIPS> code_aux{};
    if (prn > 0U && prn < 51U)
        {
            gps_l2c_m_code(code_aux, prn);
        }
    else
        {
            for (auto &v : dest)
                {
                    v = std::complex<float>(0.0F, 0.0F);
                }
            return;
        }

    // --- Effective chip configuration ---
    const int32_t chips_per_epoch = cl_zeroed ? (2 * codeLength) : codeLength;
    const float tc_used = cl_zeroed ? tc_l2c : tc_cm;

    const auto samplesPerCode = static_cast<int32_t>(
        static_cast<double>(sampling_freq) /
        (static_cast<double>(1.0F / tc_used) / static_cast<double>(chips_per_epoch)));

    const int32_t nSamples =
        std::min<int32_t>(static_cast<int32_t>(dest.size()), samplesPerCode);

    for (int32_t i = 0; i < nSamples; i++)
        {
            auto slotIndex =
                static_cast<int32_t>(std::floor(ts * static_cast<float>(i) / tc_used));

            // Avoid rounding overflow at last sample
            if (i == nSamples - 1)
                {
                    slotIndex = chips_per_epoch - 1;
                }

            if (!cl_zeroed)
                {
                    // --- Classic CM-only (511.5 kcps) ---
                    const int32_t cm_idx = std::min<int32_t>(slotIndex, codeLength - 1);
                    dest[i] = std::complex<float>(0.0F,
                        1.0F - 2.0F * code_aux[cm_idx]);
                }
            else
                {
                    // --- L2C effective 1.023 Mcps, CL zeroed ---
                    const bool is_cm_slot = ((slotIndex & 1) == 0);

                    if (is_cm_slot)
                        {
                            const int32_t cm_idx =
                                std::min<int32_t>(slotIndex >> 1, codeLength - 1);

                            dest[i] = std::complex<float>(0.0F,
                                1.0F - 2.0F * code_aux[cm_idx]);
                        }
                    else
                        {
                            // CL slot → zeroed
                            dest[i] = std::complex<float>(0.0F, 0.0F);
                        }
                }
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
            codeValueIndex = static_cast<int32_t>(std::floor(ts * static_cast<float>(i) / tc));

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
