/*!
 * \file gps_l5_signal_replica.h
 * \brief This file implements signal generators for GPS L5 signals
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L5_SIGNAL_REPLICA_H
#define GNSS_SDR_GPS_L5_SIGNAL_REPLICA_H

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


//! Generates complex GPS L5I code for the desired SV ID
void gps_l5i_code_gen_complex(own::span<std::complex<float>> dest, uint32_t prn);

//! Generates real GPS L5I code for the desired SV ID
void gps_l5i_code_gen_float(own::span<float> dest, uint32_t prn);

//! Generates complex GPS L5Q code for the desired SV ID
void gps_l5q_code_gen_complex(own::span<std::complex<float>> dest, uint32_t prn);

//! Generates real GPS L5Q code for the desired SV ID
void gps_l5q_code_gen_float(own::span<float> dest, uint32_t prn);

//! Generates complex GPS L5I code for the desired SV ID, and sampled to specific sampling frequency
void gps_l5i_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq);

//! Generates complex GPS L5Q code for the desired SV ID, and sampled to specific sampling frequency
void gps_l5q_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq);


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L5_SIGNAL_REPLICA_H
