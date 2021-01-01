/*!
 * \file glonass_l1_signal_processing.h
 * \brief This file implements various functions for GLONASS L1 CA signal
 * replica generation
 * \author Gabriel Araujo, 2017. gabriel.araujo(at)ieee.org
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

#ifndef GNSS_SDR_GLONASS_L1_SIGNAL_REPLICA_H
#define GNSS_SDR_GLONASS_L1_SIGNAL_REPLICA_H

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


//! Generates complex GLONASS L1 C/A code for the desired SV ID and code shift
void glonass_l1_ca_code_gen_complex(own::span<std::complex<float>> dest, uint32_t chip_shift);

//! Generates complex GLONASS L1 C/A code for the desired SV ID and code shift, and sampled to specific sampling frequency
void glonass_l1_ca_code_gen_complex_sampled(own::span<std::complex<float>> dest, int32_t sampling_freq, uint32_t chip_shift);


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_L1_SIGNAL_REPLICA_H
