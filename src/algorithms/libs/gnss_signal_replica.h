/*!
 * \file gnss_signal_replica.h
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

#ifndef GNSS_SDR_GNSS_SIGNAL_REPLICA_H
#define GNSS_SDR_GNSS_SIGNAL_REPLICA_H

#include <complex>
#include <cstdint>
#include <string>
#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl/gsl-lite.hpp>
namespace own = gsl;
#endif

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


/*!
 * \brief This function generates a complex exponential in dest.
 *
 */
void complex_exp_gen(own::span<std::complex<float>> dest, double freq, double sampling_freq);

/*!
 * \brief This function generates a conjugate complex exponential in dest.
 *
 */
void complex_exp_gen_conj(own::span<std::complex<float>> dest, double freq, double sampling_freq);

/*!
 * \brief This function makes a conversion from hex (the input is a char)
 *  to binary (the output are 4 ints with +1 or -1 values).
 *
 */
void hex_to_binary_converter(own::span<int32_t> dest, char from);

/*!
 * \brief This function makes a conversion from hex (the input is a char)
 *  to binary (the output is a string of 4 char with 0 or 1 values).
 *
 */
std::string hex_to_binary_string(char from);

/*!
 * \brief This function resamples a sequence of float values.
 *
 */
void resampler(const own::span<float> from, own::span<float> dest,
    float fs_in, float fs_out);

/*!
 * \brief This function resamples a sequence of complex values.
 *
 */
void resampler(own::span<const std::complex<float>> from, own::span<std::complex<float>> dest,
    float fs_in, float fs_out);


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SIGNAL_REPLICA_H
