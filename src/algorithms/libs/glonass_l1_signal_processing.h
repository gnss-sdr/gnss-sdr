/*!
 * \file glonass_l1_signal_processing.h
 * \brief This class implements various functions for GLONASS L1 CA signals
 * \author Gabriel Araujo, 2017. gabriel.araujo(at)ieee.org
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

#ifndef GNSS_SDR_GLONASS_SDR_SIGNAL_PROCESSING_H
#define GNSS_SDR_GLONASS_SDR_SIGNAL_PROCESSING_H

#include <complex>
#include <cstdint>
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


//! Generates complex GLONASS L1 C/A code for the desired SV ID and code shift, and sampled to specific sampling frequency
void glonass_l1_ca_code_gen_complex(own::span<std::complex<float>> _dest, uint32_t _chip_shift);

//! Generates N complex GLONASS L1 C/A codes for the desired SV ID and code shift
void glonass_l1_ca_code_gen_complex_sampled(own::span<std::complex<float>> _dest, int32_t _fs, uint32_t _chip_shift, uint32_t _ncodes);

//! Generates complex GLONASS L1 C/A code for the desired SV ID and code shift
void glonass_l1_ca_code_gen_complex_sampled(own::span<std::complex<float>> _dest, int32_t _fs, uint32_t _chip_shift);


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_SDR_SIGNAL_PROCESSING_H
