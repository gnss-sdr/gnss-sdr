/*!
 * \file beidou_b1c_signal_replica.cc
 * \brief Library for BeiDou B1C signal replica generation
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
 *
 * Primary codes loaded from precomputed hex tables (Galileo E1 style).
 * Pilot secondary codes from per-PRN binary strings (Galileo E5a-Q style).
 * BDS-SIS-ICD-B1C-1.0 §5.2.
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
#include "beidou_b1c_signal_replica.h"
#include "Beidou_B1C.h"
#include "Beidou_B1C_codes.h"
#include "gnss_signal_replica.h"
#include <cmath>
#include <string>
#include <vector>

namespace
{
void hex_primary_to_chip_int(own::span<int32_t> dest, const char* hex_code)
{
    int32_t index = 0;
    for (size_t i = 0; i < BEIDOU_B1C_PRIMARY_CODE_STR_LENGTH - 1; i++)
        {
            std::array<int32_t, 4> bipolar{};
            hex_to_binary_converter(bipolar, hex_code[i]);
            for (int32_t j = 0; j < 4; j++)
                {
                    dest[index++] = (bipolar[j] == 1) ? 0 : 1;
                }
        }
    std::array<int32_t, 4> last{};
    hex_to_binary_converter(last, hex_code[BEIDOU_B1C_PRIMARY_CODE_STR_LENGTH - 1]);
    dest[index++] = (last[0] == 1) ? 0 : 1;
    dest[index++] = (last[1] == 1) ? 0 : 1;
}
}  // namespace


void beidou_b1c_code_gen_int(own::span<int32_t> dest, const std::array<char, 3>& signal_id, int32_t prn)
{
    if (prn < 1 || prn > BEIDOU_B1C_NUMBER_OF_PRNS)
        {
            return;
        }
    const int32_t prn_index = prn - 1;
    const std::string sig = signal_id.data();
    const char* hex_code = nullptr;
    if (sig.rfind("1P") != std::string::npos)
        {
            hex_code = BEIDOU_B1C_PILOT_PRIMARY_CODE[prn_index];
        }
    else
        {
            hex_code = BEIDOU_B1C_DATA_PRIMARY_CODE[prn_index];
        }
    hex_primary_to_chip_int(dest, hex_code);
}


void beidou_b1c_sinboc_11_gen_int(own::span<int32_t> dest, own::span<const int32_t> prn)
{
    constexpr auto length_in = static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS);
    const auto period = static_cast<uint32_t>(dest.size() / length_in);
    for (uint32_t i = 0; i < length_in; i++)
        {
            for (uint32_t j = 0; j < (period / 2); j++)
                {
                    dest[i * period + j] = prn[i];
                }
            for (uint32_t j = (period / 2); j < period; j++)
                {
                    dest[i * period + j] = -prn[i];
                }
        }
}


void beidou_b1c_sinboc_61_gen_int(own::span<int32_t> dest, own::span<const int32_t> prn)
{
    constexpr auto length_in = static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS);
    const auto period = static_cast<uint32_t>(dest.size() / length_in);
    for (uint32_t i = 0; i < length_in; i++)
        {
            for (uint32_t j = 0; j < period; j += 2)
                {
                    dest[i * period + j] = prn[i];
                }
            for (uint32_t j = 1; j < period; j += 2)
                {
                    dest[i * period + j] = -prn[i];
                }
        }
}


namespace
{
/*!
 * QMBOC(6,1,4/33) pilot subcarrier (ICD eqs. 4-10/4-11):
 * Re = BOC(6,1), Im = BOC(1,1), power ratio 4:29.
 */
void beidou_b1c_qmboc_gen_complex(own::span<std::complex<float>> dest, own::span<const int32_t> prn)
{
    const auto code_length = dest.size();
    const float alpha = std::sqrt(29.0F / 33.0F);  // BOC(1,1) weight in sc_B1C_pilot
    const float beta = std::sqrt(4.0F / 33.0F);    // BOC(6,1) weight in sc_B1C_pilot
    std::vector<int32_t> sinboc_11(code_length);
    std::vector<int32_t> sinboc_61(code_length);
    beidou_b1c_sinboc_11_gen_int(sinboc_11, prn);
    beidou_b1c_sinboc_61_gen_int(sinboc_61, prn);
    for (size_t i = 0; i < code_length; i++)
        {
            dest[i] = std::complex<float>(
                beta * static_cast<float>(sinboc_61[i]),
                alpha * static_cast<float>(sinboc_11[i]));
        }
}
}  // namespace


void beidou_b1c_code_gen_sinboc11_float(own::span<float> dest, const std::array<char, 3>& signal_id, uint32_t prn)
{
    std::array<int32_t, 10230> primary_code{};
    beidou_b1c_code_gen_int(primary_code, signal_id, static_cast<int32_t>(prn));
    const auto code_length = static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS);
    for (uint32_t i = 0; i < code_length; i++)
        {
            const float chip = primary_code[i] == 0 ? 1.0F : -1.0F;
            dest[2 * i] = chip;
            dest[2 * i + 1] = -chip;
        }
}


void beidou_b1c_code_gen_float_sampled(own::span<float> dest, const std::array<char, 3>& signal_id,
    bool qmboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift, bool secondary_flag)
{
    constexpr auto code_freq_basis = static_cast<int32_t>(BEIDOU_B1C_CODE_RATE_CPS);
    const int32_t samples_per_chip = qmboc ? 12 : 2;
    const uint32_t code_length = samples_per_chip * static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS);
    const std::string b1c_signal = signal_id.data();
    const bool is_pilot = b1c_signal.rfind("1P") != std::string::npos;
    auto samples_per_code = static_cast<uint32_t>(static_cast<double>(sampling_freq) /
                                                  (static_cast<double>(code_freq_basis) / BEIDOU_B1C_CODE_LENGTH_CHIPS));
    const uint32_t delay = ((static_cast<int32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS) - static_cast<int32_t>(chip_shift)) %
                               static_cast<int32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS)) *
                           samples_per_code / static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS);

    std::array<int32_t, 10230> primary_code{};
    beidou_b1c_code_gen_int(primary_code, signal_id, static_cast<int32_t>(prn));

    std::array<int32_t, 10230> primary_bipolar{};
    for (uint32_t i = 0; i < static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS); i++)
        {
            primary_bipolar[i] = primary_code[i] == 0 ? 1 : -1;
        }

    // Tracking path: real BOC(1,1) only (full QMBOC is complex).
    std::vector<float> signal_b1c(code_length);
    std::vector<int32_t> signal_b1c_int(code_length);
    beidou_b1c_sinboc_11_gen_int(signal_b1c_int, primary_bipolar);
    for (uint32_t ii = 0; ii < code_length; ++ii)
        {
            signal_b1c[ii] = static_cast<float>(signal_b1c_int[ii]);
        }

    if (sampling_freq != samples_per_chip * code_freq_basis)
        {
            std::vector<float> resampled_signal(samples_per_code);
            resampler(signal_b1c, resampled_signal, static_cast<float>(samples_per_chip * code_freq_basis), sampling_freq);
            signal_b1c = std::move(resampled_signal);
        }
    else
        {
            samples_per_code = code_length;
        }

    if (is_pilot && secondary_flag)
        {
            if (prn >= 1 && prn <= static_cast<uint32_t>(BEIDOU_B1C_NUMBER_OF_PRNS))
                {
                    const int32_t prn_index = static_cast<int32_t>(prn) - 1;
                    const char* sec_code = BEIDOU_B1C_PILOT_SECONDARY_CODE[prn_index];

                    std::vector<float> signal_with_secondary(BEIDOU_B1C_PILOT_SECONDARY_CODE_LENGTH * samples_per_code);
                    for (int32_t i = 0; i < BEIDOU_B1C_PILOT_SECONDARY_CODE_LENGTH; i++)
                        {
                            const float sec_chip = sec_code[i] == '0' ? 1.0F : -1.0F;
                            for (uint32_t k = 0; k < samples_per_code; k++)
                                {
                                    signal_with_secondary[static_cast<size_t>(i) * samples_per_code + k] = signal_b1c[k] * sec_chip;
                                }
                        }
                    samples_per_code *= static_cast<uint32_t>(BEIDOU_B1C_PILOT_SECONDARY_CODE_LENGTH);
                    signal_b1c = std::move(signal_with_secondary);
                }
        }

    for (uint32_t i = 0; i < samples_per_code; i++)
        {
            dest[(i + delay) % samples_per_code] = signal_b1c[i];
        }
}


void beidou_b1c_code_gen_complex_sampled(own::span<std::complex<float>> dest, const std::array<char, 3>& signal_id,
    bool qmboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift, bool secondary_flag)
{
    constexpr auto code_freq_basis = static_cast<int32_t>(BEIDOU_B1C_CODE_RATE_CPS);
    const int32_t samples_per_chip = qmboc ? 12 : 2;
    const uint32_t code_length = samples_per_chip * static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS);
    const std::string b1c_signal = signal_id.data();
    const bool is_pilot = b1c_signal.rfind("1P") != std::string::npos;
    auto samples_per_code = static_cast<uint32_t>(static_cast<double>(sampling_freq) /
                                                  (static_cast<double>(code_freq_basis) / BEIDOU_B1C_CODE_LENGTH_CHIPS));
    const uint32_t delay = ((static_cast<int32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS) - static_cast<int32_t>(chip_shift)) %
                               static_cast<int32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS)) *
                           samples_per_code / static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS);

    std::array<int32_t, 10230> primary_code{};
    beidou_b1c_code_gen_int(primary_code, signal_id, static_cast<int32_t>(prn));

    std::array<int32_t, 10230> primary_bipolar{};
    for (uint32_t i = 0; i < static_cast<uint32_t>(BEIDOU_B1C_CODE_LENGTH_CHIPS); i++)
        {
            primary_bipolar[i] = primary_code[i] == 0 ? 1 : -1;
        }

    std::vector<std::complex<float>> signal_b1c(code_length);
    if (qmboc && is_pilot)
        {
            beidou_b1c_qmboc_gen_complex(signal_b1c, primary_bipolar);
        }
    else
        {
            std::vector<int32_t> signal_b1c_int(code_length);
            beidou_b1c_sinboc_11_gen_int(signal_b1c_int, primary_bipolar);
            for (uint32_t ii = 0; ii < code_length; ++ii)
                {
                    signal_b1c[ii] = std::complex<float>(static_cast<float>(signal_b1c_int[ii]), 0.0F);
                }
        }

    if (sampling_freq != samples_per_chip * code_freq_basis)
        {
            std::vector<std::complex<float>> resampled_signal(samples_per_code);
            resampler(signal_b1c, resampled_signal, static_cast<float>(samples_per_chip * code_freq_basis),
                static_cast<float>(sampling_freq));
            signal_b1c = std::move(resampled_signal);
        }
    else
        {
            samples_per_code = code_length;
        }

    if (is_pilot && secondary_flag)
        {
            if (prn >= 1 && prn <= static_cast<uint32_t>(BEIDOU_B1C_NUMBER_OF_PRNS))
                {
                    const int32_t prn_index = static_cast<int32_t>(prn) - 1;
                    const char* sec_code = BEIDOU_B1C_PILOT_SECONDARY_CODE[prn_index];

                    std::vector<std::complex<float>> signal_with_secondary(
                        static_cast<size_t>(BEIDOU_B1C_PILOT_SECONDARY_CODE_LENGTH) * samples_per_code);
                    for (int32_t i = 0; i < BEIDOU_B1C_PILOT_SECONDARY_CODE_LENGTH; i++)
                        {
                            const float sec_chip = sec_code[i] == '0' ? 1.0F : -1.0F;
                            for (uint32_t k = 0; k < samples_per_code; k++)
                                {
                                    signal_with_secondary[static_cast<size_t>(i) * samples_per_code + k] =
                                        signal_b1c[k] * sec_chip;
                                }
                        }
                    samples_per_code *= static_cast<uint32_t>(BEIDOU_B1C_PILOT_SECONDARY_CODE_LENGTH);
                    signal_b1c = std::move(signal_with_secondary);
                }
        }

    for (uint32_t i = 0; i < samples_per_code; i++)
        {
            dest[(i + delay) % samples_per_code] = signal_b1c[i];
        }
}


void beidou_b1c_code_gen_complex_sampled(own::span<std::complex<float>> dest, const std::array<char, 3>& signal_id,
    bool qmboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift)
{
    beidou_b1c_code_gen_complex_sampled(dest, signal_id, qmboc, prn, sampling_freq, chip_shift, false);
}
