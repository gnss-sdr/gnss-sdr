/*!
 * \file sbas_signal_replica.h
 * \brief This file implements signal generators for SBAS L1 signals
 * \author Miguel Gómez López, 2026. mgomezl(at)ing.uc3m.es
 * \author Víctor Castillo Agüero, 2026. victorcastilloaguero(at)gmail.com
 *
 * SBAS PRN codes (120–138) share the same Gold-code generator as GPS L1 C/A
 * but use SBAS-specific G2 initial state delays. The underlying code generator
 * already handles SBAS PRNs; these wrappers provide a clean SBAS-specific API.
 *
 * Reference: RTCA DO-229E, Appendix A
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_SBAS_SIGNAL_REPLICA_H
#define GNSS_SDR_SBAS_SIGNAL_REPLICA_H

#include <complex>
#include <cstdint>
#if HAS_STD_SPAN
#include <span>
namespace own = std;
#else
#include <gsl-lite/gsl-lite.hpp>
namespace own = gsl_lite;
#endif

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */

//! Generates complex SBAS L1 C/A code for the desired SV PRN, sampled at sampling_freq [Hz]
void sbas_l1_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq);

//! Generates real SBAS L1 C/A code for the desired SV PRN (chip_shift=0)
void sbas_l1_code_gen_float(own::span<float> dest, uint32_t prn);

//! Generates complex SBAS L1 C/A code for the desired SV PRN with a given chip_shift
void sbas_l1_code_gen_complex(own::span<std::complex<float>> dest, uint32_t prn, uint32_t chip_shift);

/** \} */
/** \} */

#endif  // GNSS_SDR_SBAS_SIGNAL_REPLICA_H
