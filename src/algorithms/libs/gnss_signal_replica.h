/*!
 * \file gnss_signal_replica.h
 * \brief This library gathers a few functions used for GNSS signal replica
 * generation regardless of system used
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

#ifndef GNSS_SDR_GNSS_SIGNAL_REPLICA_H
#define GNSS_SDR_GNSS_SIGNAL_REPLICA_H

#include <complex>
#include <cstdint>
#include <string>
#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl/gsl>
namespace own = gsl;
#endif

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


/*!
 * \brief This function generates a complex exponential in _dest.
 *
 */
void complex_exp_gen(own::span<std::complex<float>> _dest, double _f, double _fs);

/*!
 * \brief This function generates a conjugate complex exponential in _dest.
 *
 */
void complex_exp_gen_conj(own::span<std::complex<float>> _dest, double _f, double _fs);

/*!
 * \brief This function makes a conversion from hex (the input is a char)
 *  to binary (the output are 4 ints with +1 or -1 values).
 *
 */
void hex_to_binary_converter(own::span<int32_t> _dest, char _from);

/*!
 * \brief This function makes a conversion from hex (the input is a char)
 *  to binary (the output is a string of 4 char with 0 or 1 values).
 *
 */
std::string hex_to_binary_string(char _from);

/*!
 * \brief This function resamples a sequence of float values.
 *
 */
void resampler(const own::span<float> _from, own::span<float> _dest,
    float _fs_in, float _fs_out);

/*!
 * \brief This function resamples a sequence of complex values.
 *
 */
void resampler(own::span<const std::complex<float>> _from, own::span<std::complex<float>> _dest,
    float _fs_in, float _fs_out);


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SIGNAL_REPLICA_H
