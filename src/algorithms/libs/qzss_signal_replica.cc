/*!
 * \file qzss_signal_replica.cc
 * \brief This file implements signal generators for QZSS signals
 * \author  Carles Fernández-Prades, 2026. cfernandez (at) cttc.es
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

#include "qzss_signal_replica.h"
#include "qzss.h"
#include <array>
#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

// -----------------------------
// QZSS L1 C/A utils
// -----------------------------

struct QzssL1Entry
{
    uint16_t g2_delay_chips;  // Provided by ICD, but not applied if using init_g2_octal
    uint16_t init_g2_octal;   // 10-bit value expressed in octal digits in ICD
};


// IS-QZSS-PNT-006 Table 3.2.2-1 (PRN 193..206)
static QzssL1Entry qzss_l1_table(uint32_t prn)
{
    switch (prn)
        {
        case 193:
            return {339, 01050};
        case 194:
            return {208, 01607};
        case 195:
            return {711, 01747};
        case 196:
            return {189, 01305};
        case 197:
            return {263, 00540};
        case 198:
            return {537, 01363};  // non-standard
        case 199:
            return {663, 00727};
        case 200:
            return {942, 00147};
        case 201:
            return {173, 01206};
        case 202:
            return {900, 01045};  // non-standard
        case 203:
            return {30, 00476};  // L1C/B
        case 204:
            return {500, 00604};  // L1C/B
        case 205:
            return {935, 01757};  // L1C/B
        case 206:
            return {556, 01330};  // L1C/B
        default:
            LOG(WARNING) << "Unsupported QZSS L1 PRN";
            return {0, 0};
        }
}

// -----------------------------
// QZSS L5 utils
// -----------------------------

struct QzssL5Entry
{
    uint16_t init_i5;
    uint16_t init_q5;
    int advance_i5;
    int advance_q5;
};


// IS-QZSS-PNT-006 Table 3.2.5-1
static QzssL5Entry qzss_l5_table(uint32_t prn)
{
    switch (prn)
        {
        case 193:
            return {0b0110000101110, 0b1001110000111, 5836, 4757};
        case 194:
            return {0b0110010011111, 0b0110100111010, 926, 427};
        case 195:
            return {0b1000111001100, 0b0110001100110, 6086, 5452};
        case 196:
            return {0b111101110001, 0b0000100001100, 950, 5182};
        case 197:
            return {0b0011111100001, 0b0101000101101, 5905, 6606};
        case 198:
            return {0b0000001110001, 0b1000001010111, 3240, 6531};
        case 199:
            return {0b1010110100100, 0b0011001110001, 6675, 4268};
        case 200:
            return {0b0100001110110, 0b0100011100110, 3197, 3115};
        case 201:
            return {0b0111110100011, 0b0100101100101, 1555, 6835};
        case 202:
            return {0b0001111001011, 0b1110001010111, 3589, 862};
        default:
            LOG(WARNING) << "Unsupported QZSS L5 PRN";
            return {0, 0, 0, 0};
        }
}


// -----------------------------------
// QZSS L1 C/A code generation
// -----------------------------------

// Generates real QZSS L1 C/A code (1023 chips, +/-1)
void qzss_l1_code_gen_float(own::span<float> dest, uint32_t prn)
{
    if (dest.size() != QZSS_L1_CODE_LENGTH)
        {
            LOG(WARNING) << "QZSS L1 code must be 1023 chips";
            return;
        }

    const auto entry = qzss_l1_table(prn);

    std::array<uint8_t, 10> g1{};
    std::array<uint8_t, 10> g2{};

    g1.fill(1);
    const uint16_t init = entry.init_g2_octal;
    for (int s = 0; s < 10; ++s)
        {
            g2[s] = static_cast<uint8_t>((init >> s) & 0x1);
        }

    // Generate 1023 chips
    for (int i = 0; i < QZSS_L1_CODE_LENGTH; ++i)
        {
            const uint8_t g1_out = g1[9];
            const uint8_t g2_out = g2[9];
            const auto prn_bit = static_cast<uint8_t>(g1_out ^ g2_out);

            // Map 0 -> -1, 1 -> +1
            dest[i] = prn_bit ? 1.0F : -1.0F;

            // Step G1
            const auto g1_fb = static_cast<uint8_t>(g1[2] ^ g1[9]);
            for (int k = 9; k > 0; --k)
                {
                    g1[k] = g1[k - 1];
                }
            g1[0] = g1_fb;

            // Step G2
            const auto g2_fb = static_cast<uint8_t>(
                g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]);
            for (int k = 9; k > 0; --k)
                {
                    g2[k] = g2[k - 1];
                }
            g2[0] = g2_fb;
        }
}


void qzss_l1_code_gen_complex_sampled(
    own::span<std::complex<float>> dest,
    uint32_t prn,
    int32_t sampling_freq)
{
    if (sampling_freq <= 0)
        {
            LOG(WARNING) << "Invalid sampling frequency";
            return;
        }

    std::array<float, QZSS_L1_CODE_LENGTH> code{};
    qzss_l1_code_gen_float(own::span<float>(code.data(), code.size()), prn);

    const double phase_step = QZSS_L1_CHIP_RATE / static_cast<double>(sampling_freq);
    double code_phase = 0.0;

    for (auto & d : dest)
        {
            int chip = static_cast<int>(code_phase);
            chip %= QZSS_L1_CODE_LENGTH;
            d = {code[static_cast<size_t>(chip)], 0.0F};

            code_phase += phase_step;
            if (code_phase >= QZSS_L1_CODE_LENGTH)
                {
                    code_phase -= QZSS_L1_CODE_LENGTH;
                }
        }
}


// -----------------------------------
// QZSS L5 (I&Q) code generation
// -----------------------------------

static uint8_t xa_step(uint16_t& state)
{
    const auto out = static_cast<uint8_t>(state & 0x1);
    const auto fb = static_cast<uint8_t>(
        ((state >> 4) ^ (state >> 3) ^ (state >> 1) ^ (state >> 0)) & 0x1);

    auto next = static_cast<uint16_t>((state >> 1) | (static_cast<uint16_t>(fb) << 12));

    if (state == XA_SHORT_DECODE)
        {
            next = XA_ALL_ONES;
        }

    state = next;
    return out;
}


static uint8_t xb_step(uint16_t& state)
{
    const auto out = static_cast<uint8_t>(state & 0x1);
    const auto fb = static_cast<uint8_t>(
        ((state >> 12) ^ (state >> 10) ^ (state >> 9) ^ (state >> 7) ^
            (state >> 6) ^ (state >> 5) ^ (state >> 1) ^ (state >> 0)) &
        0x1);

    state = static_cast<uint16_t>((state >> 1) | (static_cast<uint16_t>(fb) << 12));
    return out;
}


void qzss_l5i_code_gen_float(own::span<float> dest, uint32_t prn)
{
    if (dest.size() != QZSS_L5_CODE_LENGTH)
        {
            LOG(WARNING) << "L5I code must be 10230 chips";
            return;
        }

    const auto entry = qzss_l5_table(prn);
    uint16_t xa_state = XA_ALL_ONES;
    uint16_t xb_state = entry.init_i5;

    for (int i = 0; i < QZSS_L5_CODE_LENGTH; ++i)
        {
            const uint8_t xa = xa_step(xa_state);
            const uint8_t xb = xb_step(xb_state);
            const auto bit = static_cast<uint8_t>(xa ^ xb);

            dest[i] = bit ? 1.0F : -1.0F;
        }
}


void qzss_l5q_code_gen_float(own::span<float> dest, uint32_t prn)
{
    if (dest.size() != QZSS_L5_CODE_LENGTH)
        {
            LOG(WARNING) << "L5Q code must be 10230 chips";
            return;
        }

    const auto entry = qzss_l5_table(prn);
    uint16_t xa_state = XA_ALL_ONES;
    uint16_t xb_state = entry.init_q5;

    for (int i = 0; i < QZSS_L5_CODE_LENGTH; ++i)
        {
            const uint8_t xa = xa_step(xa_state);
            const uint8_t xb = xb_step(xb_state);
            const auto bit = static_cast<uint8_t>(xa ^ xb);

            dest[i] = bit ? 1.0F : -1.0F;
        }
}


void qzss_l5i_code_gen_complex_sampled(
    own::span<std::complex<float>> dest,
    uint32_t prn,
    int32_t sampling_freq)
{
    if (sampling_freq <= 0)
        {
            LOG(WARNING) << "Invalid sampling frequency";
            return;
        }

    std::array<float, QZSS_L5_CODE_LENGTH> code{};
    qzss_l5i_code_gen_float(own::span<float>(code.data(), code.size()), prn);

    const double phase_step = QZSS_L5_CHIP_RATE / static_cast<double>(sampling_freq);
    double code_phase = 0.0;

    for (auto & d : dest)
        {
            int chip = static_cast<int>(code_phase);
            chip %= QZSS_L5_CODE_LENGTH;

            d = {code[static_cast<size_t>(chip)], 0.0F};

            code_phase += phase_step;
            if (code_phase >= QZSS_L5_CODE_LENGTH)
                {
                    code_phase -= QZSS_L5_CODE_LENGTH;
                }
        }
}


void qzss_l5q_code_gen_complex_sampled(
    own::span<std::complex<float>> dest,
    uint32_t prn,
    int32_t sampling_freq)
{
    if (sampling_freq <= 0)
        {
            LOG(WARNING) << "Invalid sampling frequency";
            return;
        }

    std::array<float, QZSS_L5_CODE_LENGTH> code{};
    qzss_l5q_code_gen_float(own::span<float>(code.data(), code.size()), prn);

    const double phase_step = QZSS_L5_CHIP_RATE / static_cast<double>(sampling_freq);
    double code_phase = 0.0;

    for (auto & d : dest)
        {
            int chip = static_cast<int>(code_phase);
            chip %= QZSS_L5_CODE_LENGTH;

            d = {0.0F, code[static_cast<size_t>(chip)]};

            code_phase += phase_step;
            if (code_phase >= QZSS_L5_CODE_LENGTH)
                {
                    code_phase -= QZSS_L5_CODE_LENGTH;
                }
        }
}