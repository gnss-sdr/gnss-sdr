/*!
 * \file qzss_signal_replica.h
 * \brief This file implements signal generators for QZSS signals
 * \author Carles Fernández-Prades, 2026. cfernandez (at) cttc.es
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

#ifndef GNSS_SDR_QZSS_SIGNAL_REPLICA_H
#define GNSS_SDR_QZSS_SIGNAL_REPLICA_H

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

//! Generates complex QZSS L1 C/A code for the desired SV ID, and sampled to specific sampling frequency
void qzss_l1_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq);

//! Generates real QZSS L1 C/A code for the desired SV ID
void qzss_l1_code_gen_float(own::span<float> dest, uint32_t prn);

//! Generates complex QZSS L5I code for the desired SV ID, and sampled to specific sampling frequency
void qzss_l5i_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq);

//! Generates real QZSS L5I code for the desired SV ID
void qzss_l5i_code_gen_float(own::span<float> dest, uint32_t prn);

//! Generates complex QZSS L5Q code for the desired SV ID, and sampled to specific sampling frequency
void qzss_l5q_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq);

//! Generates real QZSS L5I code for the desired SV ID
void qzss_l5q_code_gen_float(own::span<float> dest, uint32_t prn);

/** \} */
/** \} */
#endif  // GNSS_SDR_QZSS_SIGNAL_REPLICA_H