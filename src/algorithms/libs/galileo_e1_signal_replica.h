/*!
 * \file galileo_e1_signal_replica.h
 * \brief This library implements various functions for Galileo E1 signal
 * replica generation
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

#ifndef GNSS_SDR_GALILEO_E1_SIGNAL_REPLICA_H
#define GNSS_SDR_GALILEO_E1_SIGNAL_REPLICA_H

#include <array>
#include <complex>
#include <cstdint>
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
 * \brief This function generates Galileo E1 code (can select E1B or E1C sinboc).
 *
 */
void galileo_e1_code_gen_sinboc11_float(own::span<float> dest, const std::array<char, 3>& signal_id, uint32_t prn);

/*!
 * \brief This function generates Galileo E1 code (can select E1B or E1C, cboc or sinboc
 * and the sample frequency sampling_freq).
 *
 */
void galileo_e1_code_gen_float_sampled(own::span<float> dest, const std::array<char, 3>& signal_id,
    bool cboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift,
    bool secondary_flag);

/*!
 * \brief This function generates Galileo E1 code (can select E1B or E1C, cboc or sinboc
 * and the sample frequency sampling_freq).
 *
 */
void galileo_e1_code_gen_float_sampled(own::span<float> dest, const std::array<char, 3>& signal_id,
    bool cboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift);

/*!
 * \brief This function generates Galileo E1 code (can select E1B or E1C, cboc or sinboc
 * and the sample frequency sampling_freq).
 *
 */
void galileo_e1_code_gen_complex_sampled(own::span<std::complex<float>> dest, const std::array<char, 3>& signal_id,
    bool cboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift,
    bool secondary_flag);

/*!
 * \brief galileo_e1_code_gen_complex_sampled without secondary_flag for backward compatibility.
 */
void galileo_e1_code_gen_complex_sampled(own::span<std::complex<float>> dest, const std::array<char, 3>& signal_id,
    bool cboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift);


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E1_SIGNAL_REPLICA_H
