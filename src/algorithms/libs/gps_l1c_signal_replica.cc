/*!
 * \file gps_l1c_signal_replica.cc
 * \brief This library implements various functions for GPS L1C signal
 * replica generation
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "gps_l1c_signal_replica.h"
#include "GPS_L1C.h"
#include "gnss_signal_replica.h"
#include <algorithm>
#include <array>
#include <utility>
#include <vector>

namespace
{
// Legendre sequence L(t), t = 0, ..., 10222 (IS-GPS-800J, 3.2.2.1.1)
const std::vector<uint8_t>& legendre_sequence()
{
    static const std::vector<uint8_t> sequence = []() {
        std::vector<uint8_t> l(GPS_L1C_WEIL_LENGTH, 0);
        for (int64_t x = 1; x < GPS_L1C_WEIL_LENGTH; x++)
            {
                l[static_cast<size_t>((x * x) % GPS_L1C_WEIL_LENGTH)] = 1;
            }
        l[0] = 0;
        return l;
    }();
    return sequence;
}


bool is_valid_prn(uint32_t prn)
{
    return prn >= 1 && prn <= static_cast<uint32_t>(GPS_L1C_NUMBER_OF_PRNS);
}


bool is_tmboc_boc61_slot(int32_t chip)
{
    const int32_t u = chip % 33;
    return std::any_of(GPS_L1C_TMBOC_BOC61_SLOTS.cbegin(), GPS_L1C_TMBOC_BOC61_SLOTS.cend(),
        [u](int32_t slot) { return slot == u; });
}
}  // namespace


void gps_l1c_code_gen_int(own::span<int32_t> dest, bool pilot, uint32_t prn)
{
    if (!is_valid_prn(prn) || dest.size() < static_cast<size_t>(GPS_L1C_CODE_LENGTH_CHIPS))
        {
            return;
        }
    const auto& l = legendre_sequence();
    const int32_t w = pilot ? GPS_L1CP_WEIL_INDEX[prn - 1] : GPS_L1CD_WEIL_INDEX[prn - 1];
    const int32_t p = pilot ? GPS_L1CP_INSERTION_INDEX[prn - 1] : GPS_L1CD_INSERTION_INDEX[prn - 1];
    const std::array<int32_t, 7> expansion{{0, 1, 1, 0, 1, 0, 0}};

    // The expansion sequence is inserted before the p-th value of the Weil code,
    // so it occupies chips t = p - 1, ..., p + 5
    const auto code_length = static_cast<int32_t>(GPS_L1C_CODE_LENGTH_CHIPS);
    for (int32_t t = 0; t < code_length; t++)
        {
            if (t < p - 1)
                {
                    dest[t] = l[t] ^ l[(t + w) % GPS_L1C_WEIL_LENGTH];
                }
            else if (t < p + 6)
                {
                    dest[t] = expansion[t - p + 1];
                }
            else
                {
                    dest[t] = l[t - 7] ^ l[(t - 7 + w) % GPS_L1C_WEIL_LENGTH];
                }
        }
}


std::string gps_l1c_overlay_code(uint32_t prn)
{
    if (!is_valid_prn(prn))
        {
            return {};
        }
    // 11-stage S1 generator (IS-GPS-800J Figure 3.2-2): the output is stage 11,
    // and the feedback into stage 1 is the sum of stages j with m_j = 1
    const uint32_t polynomial = GPS_L1CO_S1_POLYNOMIAL[prn - 1];
    uint32_t state = GPS_L1CO_INITIAL_BITS[prn - 1];  // bit j - 1 holds stage j
    std::string code(GPS_L1C_OVERLAY_CODE_LENGTH, '0');
    for (int32_t i = 0; i < GPS_L1C_OVERLAY_CODE_LENGTH; i++)
        {
            code[i] = ((state >> 10U) & 1U) ? '1' : '0';
            uint32_t feedback = 0;
            for (uint32_t j = 1; j <= 11; j++)
                {
                    feedback ^= ((polynomial >> j) & (state >> (j - 1U))) & 1U;
                }
            state = ((state << 1U) | feedback) & 0x7FFU;
        }
    return code;
}


void gps_l1c_code_gen_float(own::span<float> dest, bool pilot, bool tmboc, uint32_t prn)
{
    const int32_t samples_per_chip = tmboc ? GPS_L1C_TMBOC_SAMPLES_PER_CHIP : GPS_L1C_BOC_SAMPLES_PER_CHIP;
    const auto code_length = static_cast<int32_t>(GPS_L1C_CODE_LENGTH_CHIPS);
    if (dest.size() < static_cast<size_t>(samples_per_chip) * static_cast<size_t>(code_length))
        {
            return;
        }
    std::vector<int32_t> chips(code_length, 0);
    gps_l1c_code_gen_int(chips, pilot, prn);
    for (int32_t t = 0; t < code_length; t++)
        {
            // A "0" is in-phase with the carrier (IS-GPS-800J, 3.3)
            const float chip = chips[t] == 0 ? 1.0F : -1.0F;
            const bool boc61 = pilot && tmboc && is_tmboc_boc61_slot(t);
            for (int32_t k = 0; k < samples_per_chip; k++)
                {
                    float sub = 0.0F;
                    if (boc61)
                        {
                            // 6 cycles of a sine-phased square wave
                            sub = (k % 2 == 0) ? 1.0F : -1.0F;
                        }
                    else
                        {
                            // 1 cycle of a sine-phased square wave
                            sub = (k < samples_per_chip / 2) ? 1.0F : -1.0F;
                        }
                    dest[t * samples_per_chip + k] = chip * sub;
                }
        }
}


void gps_l1c_code_gen_complex_sampled(own::span<std::complex<float>> dest,
    bool pilot,
    bool tmboc,
    uint32_t prn,
    int32_t sampling_freq,
    uint32_t chip_shift)
{
    const int32_t samples_per_chip = tmboc ? GPS_L1C_TMBOC_SAMPLES_PER_CHIP : GPS_L1C_BOC_SAMPLES_PER_CHIP;
    const auto code_length = static_cast<int32_t>(GPS_L1C_CODE_LENGTH_CHIPS);
    const auto samples_per_code = static_cast<uint32_t>(static_cast<double>(sampling_freq) * GPS_L1C_CODE_PERIOD_S);
    if (samples_per_code == 0 || dest.size() < samples_per_code)
        {
            return;
        }
    const auto delay = static_cast<uint32_t>(((code_length - static_cast<int32_t>(chip_shift % static_cast<uint32_t>(code_length))) % code_length) *
                                             static_cast<int64_t>(samples_per_code) / code_length);

    std::vector<float> signal(static_cast<size_t>(samples_per_chip) * code_length);
    gps_l1c_code_gen_float(signal, pilot, tmboc, prn);

    const float fs_code = static_cast<float>(samples_per_chip) * static_cast<float>(GPS_L1C_CODE_RATE_CPS);
    if (static_cast<uint32_t>(signal.size()) != samples_per_code)
        {
            std::vector<float> resampled_signal(samples_per_code);
            resampler(signal, resampled_signal, fs_code, static_cast<float>(sampling_freq));
            signal = std::move(resampled_signal);
        }

    for (uint32_t i = 0; i < samples_per_code; i++)
        {
            dest[(i + delay) % samples_per_code] = std::complex<float>(signal[i], 0.0F);
        }
}
