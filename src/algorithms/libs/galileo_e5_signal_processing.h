/*!
 * \file galileo_e5_signal_processing.h
 * \brief This library implements various functions for Galileo E5 signals such
 * as replica code generation
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 *
 * Detailed description of the file here if needed.
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

#ifndef GNSS_SDR_GALILEO_E5_SIGNAL_PROCESSING_H
#define GNSS_SDR_GALILEO_E5_SIGNAL_PROCESSING_H

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
 * \brief Generates Galileo E5a code at 1 sample/chip
 * bool _pilot generates E5aQ code if true and E5aI (data signal) if false.
 */
void galileo_e5_a_code_gen_complex_primary(own::span<std::complex<float>> _dest, int32_t _prn, const std::array<char, 3>& _Signal);

/*!
 * \brief Generates Galileo E5a complex code, shifted to the desired chip and sampled at a frequency fs
 * bool _pilot generates E5aQ code if true and E5aI (data signal) if false.
 */
void galileo_e5_a_code_gen_complex_sampled(own::span<std::complex<float>> _dest,
    const std::array<char, 3>& _Signal, uint32_t _prn, int32_t _fs, uint32_t _chip_shift);


#endif  // GNSS_SDR_GALILEO_E5_SIGNAL_PROCESSING_H
