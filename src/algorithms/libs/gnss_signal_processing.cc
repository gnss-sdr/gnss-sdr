/*!
 * \file gnss_signal_processing.cc
 * \brief This library gathers a few functions used by the algorithms of gnss-sdr,
 *  regardless of system used
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
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

#include "gnss_signal_processing.h"
#include "MATH_CONSTANTS.h"
#include <gnuradio/fxpt_nco.h>
#include <cstddef>  // for size_t


const auto AUX_CEIL2 = [](float x) { return static_cast<int32_t>(static_cast<int64_t>((x) + 1)); };

void complex_exp_gen(own::span<std::complex<float>> _dest, double _f, double _fs)
{
    gr::fxpt_nco d_nco;
    d_nco.set_freq((TWO_PI * _f) / _fs);
    d_nco.sincos(_dest.data(), _dest.size(), 1);
}


void complex_exp_gen_conj(own::span<std::complex<float>> _dest, double _f, double _fs)
{
    gr::fxpt_nco d_nco;
    d_nco.set_freq(-(TWO_PI * _f) / _fs);
    d_nco.sincos(_dest.data(), _dest.size(), 1);
}


void hex_to_binary_converter(own::span<int32_t> _dest, char _from)
{
    switch (_from)
        {
        case '0':
            _dest[0] = 1;
            _dest[1] = 1;
            _dest[2] = 1;
            _dest[3] = 1;
            break;
        case '1':
            _dest[0] = 1;
            _dest[1] = 1;
            _dest[2] = 1;
            _dest[3] = -1;
            break;
        case '2':
            _dest[0] = 1;
            _dest[1] = 1;
            _dest[2] = -1;
            _dest[3] = 1;
            break;
        case '3':
            _dest[0] = 1;
            _dest[1] = 1;
            _dest[2] = -1;
            _dest[3] = -1;
            break;
        case '4':
            _dest[0] = 1;
            _dest[1] = -1;
            _dest[2] = 1;
            _dest[3] = 1;
            break;
        case '5':
            _dest[0] = 1;
            _dest[1] = -1;
            _dest[2] = 1;
            _dest[3] = -1;
            break;
        case '6':
            _dest[0] = 1;
            _dest[1] = -1;
            _dest[2] = -1;
            _dest[3] = 1;
            break;
        case '7':
            _dest[0] = 1;
            _dest[1] = -1;
            _dest[2] = -1;
            _dest[3] = -1;
            break;
        case '8':
            _dest[0] = -1;
            _dest[1] = 1;
            _dest[2] = 1;
            _dest[3] = 1;
            break;
        case '9':
            _dest[0] = -1;
            _dest[1] = 1;
            _dest[2] = 1;
            _dest[3] = -1;
            break;
        case 'A':
            _dest[0] = -1;
            _dest[1] = 1;
            _dest[2] = -1;
            _dest[3] = 1;
            break;
        case 'B':
            _dest[0] = -1;
            _dest[1] = 1;
            _dest[2] = -1;
            _dest[3] = -1;
            break;
        case 'C':
            _dest[0] = -1;
            _dest[1] = -1;
            _dest[2] = 1;
            _dest[3] = 1;
            break;
        case 'D':
            _dest[0] = -1;
            _dest[1] = -1;
            _dest[2] = 1;
            _dest[3] = -1;
            break;
        case 'E':
            _dest[0] = -1;
            _dest[1] = -1;
            _dest[2] = -1;
            _dest[3] = 1;
            break;
        case 'F':
            _dest[0] = -1;
            _dest[1] = -1;
            _dest[2] = -1;
            _dest[3] = -1;
            break;
        default:
            break;
        }
}


void resampler(const own::span<float> _from, own::span<float> _dest, float _fs_in,
    float _fs_out)
{
    uint32_t _codeValueIndex;
    float aux;
    // --- Find time constants -------------------------------------------------
    const float _t_in = 1 / _fs_in;    // Incoming sampling  period in sec
    const float _t_out = 1 / _fs_out;  // Out sampling period in sec
    for (size_t i = 0; i < _dest.size() - 1; i++)
        {
            // === Digitizing ==================================================
            // --- compute index array to read sampled values ------------------
            aux = (_t_out * (i + 1)) / _t_in;
            _codeValueIndex = AUX_CEIL2(aux) - 1;

            // if repeat the chip -> upsample by nearest neighborhood interpolation
            _dest[i] = _from[_codeValueIndex];
        }
    // --- Correct the last index (due to number rounding issues) -----------
    _dest[_dest.size() - 1] = _from[_from.size() - 1];
}


void resampler(own::span<const std::complex<float>> _from, own::span<std::complex<float>> _dest, float _fs_in,
    float _fs_out)
{
    uint32_t _codeValueIndex;
    float aux;
    // --- Find time constants -------------------------------------------------
    const float _t_in = 1 / _fs_in;    // Incoming sampling  period in sec
    const float _t_out = 1 / _fs_out;  // Out sampling period in sec
    for (size_t i = 0; i < _dest.size() - 1; i++)
        {
            // === Digitizing ==================================================
            // --- compute index array to read sampled values ------------------
            aux = (_t_out * (i + 1)) / _t_in;
            _codeValueIndex = AUX_CEIL2(aux) - 1;

            // if repeat the chip -> upsample by nearest neighborhood interpolation
            _dest[i] = _from[_codeValueIndex];
        }
    // --- Correct the last index (due to number rounding issues) -----------
    _dest[_dest.size() - 1] = _from[_from.size() - 1];
}
