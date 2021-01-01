/*!
 * \file gnss_signal_replica.cc
 * \brief This library gathers a few functions used for GNSS signal replica
 * generation regardless of system used
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "gnss_signal_replica.h"
#include "MATH_CONSTANTS.h"
#include <gnuradio/fxpt_nco.h>
#include <cstddef>  // for size_t


const auto AUX_CEIL2 = [](float x) { return static_cast<int32_t>(static_cast<int64_t>((x) + 1)); };

void complex_exp_gen(own::span<std::complex<float>> dest, double freq, double sampling_freq)
{
    gr::fxpt_nco d_nco;
    d_nco.set_freq(static_cast<float>((TWO_PI * freq) / sampling_freq));
    d_nco.sincos(dest.data(), dest.size(), 1);
}


void complex_exp_gen_conj(own::span<std::complex<float>> dest, double freq, double sampling_freq)
{
    gr::fxpt_nco d_nco;
    d_nco.set_freq(-static_cast<float>((TWO_PI * freq) / sampling_freq));
    d_nco.sincos(dest.data(), dest.size(), 1);
}


void hex_to_binary_converter(own::span<int32_t> dest, char from)
{
    switch (from)
        {
        case '0':
            dest[0] = 1;
            dest[1] = 1;
            dest[2] = 1;
            dest[3] = 1;
            break;
        case '1':
            dest[0] = 1;
            dest[1] = 1;
            dest[2] = 1;
            dest[3] = -1;
            break;
        case '2':
            dest[0] = 1;
            dest[1] = 1;
            dest[2] = -1;
            dest[3] = 1;
            break;
        case '3':
            dest[0] = 1;
            dest[1] = 1;
            dest[2] = -1;
            dest[3] = -1;
            break;
        case '4':
            dest[0] = 1;
            dest[1] = -1;
            dest[2] = 1;
            dest[3] = 1;
            break;
        case '5':
            dest[0] = 1;
            dest[1] = -1;
            dest[2] = 1;
            dest[3] = -1;
            break;
        case '6':
            dest[0] = 1;
            dest[1] = -1;
            dest[2] = -1;
            dest[3] = 1;
            break;
        case '7':
            dest[0] = 1;
            dest[1] = -1;
            dest[2] = -1;
            dest[3] = -1;
            break;
        case '8':
            dest[0] = -1;
            dest[1] = 1;
            dest[2] = 1;
            dest[3] = 1;
            break;
        case '9':
            dest[0] = -1;
            dest[1] = 1;
            dest[2] = 1;
            dest[3] = -1;
            break;
        case 'A':
            dest[0] = -1;
            dest[1] = 1;
            dest[2] = -1;
            dest[3] = 1;
            break;
        case 'B':
            dest[0] = -1;
            dest[1] = 1;
            dest[2] = -1;
            dest[3] = -1;
            break;
        case 'C':
            dest[0] = -1;
            dest[1] = -1;
            dest[2] = 1;
            dest[3] = 1;
            break;
        case 'D':
            dest[0] = -1;
            dest[1] = -1;
            dest[2] = 1;
            dest[3] = -1;
            break;
        case 'E':
            dest[0] = -1;
            dest[1] = -1;
            dest[2] = -1;
            dest[3] = 1;
            break;
        case 'F':
            dest[0] = -1;
            dest[1] = -1;
            dest[2] = -1;
            dest[3] = -1;
            break;
        default:
            break;
        }
}


std::string hex_to_binary_string(char from)
{
    std::string dest("0000");
    switch (from)
        {
        case '0':
            dest[0] = '0';
            dest[1] = '0';
            dest[2] = '0';
            dest[3] = '0';
            break;
        case '1':
            dest[0] = '0';
            dest[1] = '0';
            dest[2] = '0';
            dest[3] = '1';
            break;
        case '2':
            dest[0] = '0';
            dest[1] = '0';
            dest[2] = '1';
            dest[3] = '0';
            break;
        case '3':
            dest[0] = '0';
            dest[1] = '0';
            dest[2] = '1';
            dest[3] = '1';
            break;
        case '4':
            dest[0] = '0';
            dest[1] = '1';
            dest[2] = '0';
            dest[3] = '0';
            break;
        case '5':
            dest[0] = '0';
            dest[1] = '1';
            dest[2] = '0';
            dest[3] = '1';
            break;
        case '6':
            dest[0] = '0';
            dest[1] = '1';
            dest[2] = '1';
            dest[3] = '0';
            break;
        case '7':
            dest[0] = '0';
            dest[1] = '1';
            dest[2] = '1';
            dest[3] = '1';
            break;
        case '8':
            dest[0] = '1';
            dest[1] = '0';
            dest[2] = '0';
            dest[3] = '0';
            break;
        case '9':
            dest[0] = '1';
            dest[1] = '0';
            dest[2] = '0';
            dest[3] = '1';
            break;
        case 'A':
            dest[0] = '1';
            dest[1] = '0';
            dest[2] = '1';
            dest[3] = '0';
            break;
        case 'B':
            dest[0] = '1';
            dest[1] = '0';
            dest[2] = '1';
            dest[3] = '1';
            break;
        case 'C':
            dest[0] = '1';
            dest[1] = '1';
            dest[2] = '0';
            dest[3] = '0';
            break;
        case 'D':
            dest[0] = '1';
            dest[1] = '1';
            dest[2] = '0';
            dest[3] = '1';
            break;
        case 'E':
            dest[0] = '1';
            dest[1] = '1';
            dest[2] = '1';
            dest[3] = '0';
            break;
        case 'F':
            dest[0] = '1';
            dest[1] = '1';
            dest[2] = '1';
            dest[3] = '1';
            break;
        default:
            break;
        }
    return dest;
}


void resampler(const own::span<float> from, own::span<float> dest, float fs_in,
    float fs_out)
{
    uint32_t codeValueIndex;
    float aux;
    const float t_out = 1.0F / fs_out;  // Output sampling period
    const size_t dest_size = dest.size();
    for (size_t i = 0; i < dest_size - 1; i++)
        {
            aux = (t_out * (static_cast<float>(i) + 1.0F)) * fs_in;
            codeValueIndex = AUX_CEIL2(aux) - 1;
            dest[i] = from[codeValueIndex];
        }
    // Correct the last index (due to number rounding issues)
    dest[dest_size - 1] = from[from.size() - 1];
}


void resampler(own::span<const std::complex<float>> from, own::span<std::complex<float>> dest, float fs_in,
    float fs_out)
{
    uint32_t codeValueIndex;
    float aux;
    const float t_out = 1.0F / fs_out;  // Output sampling period
    const size_t dest_size = dest.size();
    for (size_t i = 0; i < dest_size - 1; i++)
        {
            aux = (t_out * (static_cast<float>(i) + 1.0F)) * fs_in;
            codeValueIndex = AUX_CEIL2(aux) - 1;
            dest[i] = from[codeValueIndex];
        }
    // Correct the last index (due to number rounding issues)
    dest[dest_size - 1] = from[from.size() - 1];
}
