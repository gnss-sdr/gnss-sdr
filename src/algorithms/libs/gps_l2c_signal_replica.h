/*!
 * \file gps_l2c_signal_replica.h
 * \brief This file implements signal generators for GPS L2C signals
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L2C_SIGNAL_REPLICA_H
#define GNSS_SDR_GPS_L2C_SIGNAL_REPLICA_H

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


//! Generates complex GPS L2C M code for the desired SV ID
void gps_l2c_m_code_gen_complex(own::span<std::complex<float>> dest, uint32_t prn);

//! Generates float GPS L2C M code for the desired SV ID
void gps_l2c_m_code_gen_float(own::span<float> dest, uint32_t prn);

//! Generates complex GPS L2C M code for the desired SV ID, and sampled to specific sampling frequency
void gps_l2c_m_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq);


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L2C_SIGNAL_REPLICA_H
