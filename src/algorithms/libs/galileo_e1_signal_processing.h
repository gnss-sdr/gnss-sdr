/*!
 * \file galileo_e1_signal_processing.h
 * \brief This library implements various functions for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_E1_SIGNAL_PROCESSING_H
#define GNSS_SDR_GALILEO_E1_SIGNAL_PROCESSING_H

#include <array>
#include <complex>
#include <cstdint>
#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl/gsl>
namespace own = gsl;
#endif


/*!
 * \brief This function generates Galileo E1 code (can select E1B or E1C sinboc).
 *
 */
void galileo_e1_code_gen_sinboc11_float(own::span<float> _dest, const std::array<char, 3>& _Signal, uint32_t _prn);

/*!
 * \brief This function generates Galileo E1 code (can select E1B or E1C, cboc or sinboc
 * and the sample frequency _fs).
 *
 */
void galileo_e1_code_gen_float_sampled(own::span<float> _dest, const std::array<char, 3>& _Signal,
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift,
    bool _secondary_flag);

/*!
 * \brief This function generates Galileo E1 code (can select E1B or E1C, cboc or sinboc
 * and the sample frequency _fs).
 *
 */
void galileo_e1_code_gen_float_sampled(own::span<float> _dest, const std::array<char, 3>& _Signal,
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift);

/*!
 * \brief This function generates Galileo E1 code (can select E1B or E1C, cboc or sinboc
 * and the sample frequency _fs).
 *
 */
void galileo_e1_code_gen_complex_sampled(own::span<std::complex<float>> _dest, const std::array<char, 3>& _Signal,
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift,
    bool _secondary_flag);

/*!
 * \brief galileo_e1_code_gen_complex_sampled without _secondary_flag for backward compatibility.
 */
void galileo_e1_code_gen_complex_sampled(own::span<std::complex<float>> _dest, const std::array<char, 3>& _Signal,
    bool _cboc, uint32_t _prn, int32_t _fs, uint32_t _chip_shift);

#endif  // GNSS_SDR_GALILEO_E1_SIGNAL_PROCESSING_H
