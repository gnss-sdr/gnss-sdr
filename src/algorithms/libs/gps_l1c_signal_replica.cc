/*!
 * \file gps_l1c_signal_replica.h
 * \brief This library implements various functions for GPS L1C signal
 * replica generation
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
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

#include "GPS_L1C.h"
#include "gnss_signal_replica.h"
#include "gps_l1c_signal_replica.h"
#include <cassert>
#include <cmath>
#include <cstddef>  // for size_t
#include <utility>
#include <vector>

std::string generate_gps_l1c_overlay_code(uint32_t prn)
{
    const int32_t prn_ = prn - 1;
    int32_t index = 0;

    if ((prn < 1) || (prn > GPS_L1C_NUMBER_OF_PRN))
        {
            return "";
        }

    std::string out{};
    out.reserve(GPS_L1C_O_CODE_LENGTH_CHIPS);

    for (size_t i = 0; i < GPS_L1C_OVERLAY_CODE_STR_LENGTH; i++)
        {
            std::string aux = hex_to_binary_string(GPS_L1C_OVERLAY_CODE[prn_][i]);
            out[index] = aux[0];
            out[index + 1] = aux[1];
            out[index + 2] = aux[2];
            out[index + 3] = aux[3];
            index = index + 4;
        }

    return out;
}

// The last 2 bits of the 10232 bits of "dest" are zero-padding
void gps_l1c_range_code_gen_int(own::span<int> dest, bool p_code, uint32_t prn)
{
    assert(dest.size() >= 10232);

    const int32_t prn_ = prn - 1;
    int32_t index = 0;

    if ((prn < 1) || (prn > GPS_L1C_NUMBER_OF_PRN))
        {
            return;
        }

    const auto* source = &GPS_L1C_D_RANGE_CODE;

    if (p_code)
        {
            source = &GPS_L1C_P_RANGE_CODE;
        }

    for (size_t i = 0; i < GPS_L1C_RANGE_CODE_STR_LENGTH; i++)
        {
            hex_to_binary_converter(dest.subspan(index, 4), (*source)[prn_][i]);

            index += 4;
        }
}

void gps_l1c_overlay_code_gen_int(own::span<int> dest, uint32_t prn)
{
    assert(dest.size() >= 1800);

    const int32_t prn_ = prn - 1;
    int32_t index = 0;

    if ((prn < 1) || (prn > GPS_L1C_NUMBER_OF_PRN))
        {
            return;
        }

    for (size_t i = 0; i < GPS_L1C_OVERLAY_CODE_STR_LENGTH; i++)
        {
            hex_to_binary_converter(dest.subspan(index, 4), GPS_L1C_OVERLAY_CODE[prn_][i]);
            index += 4;
        }
}

void gps_l1c_sinboc(own::span<float> dest, bool p_code, uint32_t prn)
{
    // Short names
    constexpr size_t nr = GPS_L1C_CODE_LENGTH_CHIPS;

    // +2 due to padding
    std::array<int32_t, nr + 2> range_code_chips{};
    gps_l1c_range_code_gen_int(range_code_chips, p_code, prn);

    assert(dest.size() >= nr * 2);

    for (size_t r = 0; r < nr; r++)
        {
            int32_t value = range_code_chips[r];
            dest[r * 2 + 0] = static_cast<float>(value);
            dest[r * 2 + 1] = -static_cast<float>(value);
        }
}

void gps_l1c_p_sinboc_with_overlay(own::span<float> dest, uint32_t prn)
{
    // Short names
    const size_t nr = GPS_L1C_CODE_LENGTH_CHIPS;
    const size_t no = GPS_L1C_O_CODE_LENGTH_CHIPS;

    // +2 due to padding
    std::array<int32_t, nr + 2> range_code_chips{};
    gps_l1c_range_code_gen_int(range_code_chips, true, prn);

    std::array<int32_t, no> overlay_code_chips{};
    gps_l1c_overlay_code_gen_int(overlay_code_chips, prn);

    assert(dest.size() >= nr * no * 2);

    for (size_t o = 0; o < no; o++)
        {
            for (size_t r = 0; r < nr; r++)
                {
                    int32_t value = range_code_chips[r] * overlay_code_chips[o];
                    dest[(o * nr + r) * 2 + 0] = static_cast<float>(value);
                    dest[(o * nr + r) * 2 + 1] = -static_cast<float>(value);
                }
        }
}

void gps_l1c_p_sinboc_no_overlay(own::span<float> dest, uint32_t prn)
{
    // Short names
    constexpr size_t nr = GPS_L1C_CODE_LENGTH_CHIPS;

    // +2 due to padding
    std::array<int32_t, nr + 2> range_code_chips{};
    gps_l1c_range_code_gen_int(range_code_chips, true, prn);

    assert(dest.size() >= nr * 2);

    for (size_t r = 0; r < nr; r++)
        {
            int32_t value = range_code_chips[r];
            dest[r * 2 + 0] = static_cast<float>(value);
            dest[r * 2 + 1] = -static_cast<float>(value);
        }
}


void gps_l1c_code_gen_float_sampled(own::span<float> dest, bool p_code, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift)
{
    // Short names
    const size_t nr = GPS_L1C_CODE_LENGTH_CHIPS;

    const int32_t codeFreqBasis = GPS_L1C_CODE_CHIP_RATE_CPS;
    const int32_t samplesPerChip = 2;

    const auto samplesPerCode = static_cast<uint32_t>(static_cast<double>(sampling_freq) * GPS_L1C_CODE_PERIOD_S);

    const uint32_t delay = ((static_cast<int32_t>(GPS_L1C_CODE_LENGTH_CHIPS) - chip_shift) % static_cast<int32_t>(GPS_L1C_CODE_LENGTH_CHIPS)) * samplesPerCode / GPS_L1C_CODE_LENGTH_CHIPS;

    // +2 due to padding
    std::array<int32_t, nr + 2> range_code_chips{};
    gps_l1c_range_code_gen_int(range_code_chips, p_code, prn);

    // Signal sampled at the ideal samplesPerChip * chip rate
    const uint32_t codeLength = samplesPerChip * GPS_L1C_CODE_LENGTH_CHIPS;
    std::vector<float> signal(codeLength);

    for (uint32_t i = 0; i < GPS_L1C_CODE_LENGTH_CHIPS; i++)
        {
            // BOC(1, 1)
            signal[i * 2 + 0] = range_code_chips[i];
            signal[i * 2 + 1] = -range_code_chips[i];
        }

    if (sampling_freq != samplesPerChip * codeFreqBasis)
        {
            std::vector<float> resampled_signal(samplesPerCode);

            resampler(signal, resampled_signal, static_cast<float>(samplesPerChip * codeFreqBasis), sampling_freq);

            signal = std::move(resampled_signal);
        }

    for (uint32_t i = 0; i < samplesPerCode; i++)
        {
            dest[(i + delay) % samplesPerCode] = signal[i];
        }
}

void gps_l1c_code_gen_complex_sampled(own::span<std::complex<float>> dest, bool p_code, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift)
{
    const auto samplesPerCode = static_cast<uint32_t>(static_cast<double>(sampling_freq) * GPS_L1C_CODE_PERIOD_S);
    std::vector<float> real_code(samplesPerCode);

    gps_l1c_code_gen_float_sampled(real_code, p_code, prn, sampling_freq, chip_shift);
    for (uint32_t i = 0; i < samplesPerCode; ++i)
        {
            dest[i] = std::complex<float>(real_code[i], 0.0F);
        }
}
