/*!
 * \file galileo_e6_signal_replica.h
 * \brief This library implements various functions for Galileo E6 signal
 * replica generation
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_GALILEO_E6_SIGNAL_REPLICA_H
#define GNSS_SDR_GALILEO_E6_SIGNAL_REPLICA_H

#include <array>
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
 * \brief Generates Galileo E6B code at 1 sample/chip
 */
void galileo_e6_b_code_gen_complex_primary(own::span<std::complex<float>> dest,
    int32_t prn);


/*!
 * \brief Generates Galileo E6B code at 1 sample/chip
 */
void galileo_e6_b_code_gen_float_primary(own::span<float> dest, int32_t prn);


/*!
 * \brief Generates Galileo E6B complex code, shifted to the desired chip and
 * sampled at a frequency sampling_freq
 */
void galileo_e6_b_code_gen_complex_sampled(own::span<std::complex<float>> dest,
    uint32_t prn,
    int32_t sampling_freq,
    uint32_t chip_shift);


/*!
 * \brief Generates Galileo E6C codes at 1 sample/chip
 */
void galileo_e6_c_code_gen_complex_primary(own::span<std::complex<float>> dest,
    int32_t prn);


/*!
 * \brief Generates Galileo E6C codes at 1 sample/chip
 */
void galileo_e6_c_code_gen_float_primary(own::span<float> dest, int32_t prn);


/*!
 * \brief Generates Galileo E6C complex codes, shifted to the desired chip and
 * sampled at a frequency sampling_freq
 */
void galileo_e6_c_code_gen_complex_sampled(own::span<std::complex<float>> dest,
    uint32_t prn,
    int32_t sampling_freq,
    uint32_t chip_shift);


/*!
 * \brief Generates Galileo E6C secondary codes at 1 sample/chip
 */
void galileo_e6_c_secondary_code_gen_complex(own::span<std::complex<float>> dest,
    int32_t prn);


/*!
 * \brief Generates Galileo E6C secondary codes at 1 sample/chip
 */
void galileo_e6_c_secondary_code_gen_float(own::span<float> dest,
    int32_t prn);


/*!
 * \brief Generates a string with Galileo E6C secondary codes at 1 sample/chip
 */
std::string galileo_e6_c_secondary_code(int32_t prn);


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E6_SIGNAL_REPLICA_H
