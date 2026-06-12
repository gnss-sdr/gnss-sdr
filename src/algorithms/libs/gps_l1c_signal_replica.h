/*!
 * \file gps_l1c_signal_replica.h
 * \brief This library implements various functions for GPS L1C signal
 * replica generation
 * \author José Antonio Mayo, 2026. contact(at)tatjam.eu
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

#ifndef GNSS_SDR_GPS_L1C_SIGNAL_REPLICA_H
#define GNSS_SDR_GPS_L1C_SIGNAL_REPLICA_H

#include <array>
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


/*!
 * \brief This function generates GPS L1C-P sinboc code as
 * {-1.0, 1.0} x10230x2 floats
 *
 */
void gps_l1c_p_sinboc(own::span<float> dest, uint32_t prn);

/*!
 * \brief This function generates GPS L1C-D sinboc code as
 * {-1.0, 1.0} x10230x1800x2 floats, including the fully
 * expanded overlay code.
 *
 */
void gps_l1c_d_sinboc_with_overlay(own::span<float> dest, uint32_t prn);


/*!
 * \brief This function generates GPS L1C-D sinboc code as
 * {-1.0, 1.0} x10230x2 floats, NOT including the overlay code.
 *
 */
void gps_l1c_d_sinboc_no_overlay(own::span<float> dest, uint32_t prn);

/*!
 * \brief This function generates a full GPS L1C-P code at the sample
 *  frequency sampling_freq.
 *
 */
void gps_l1c_p_code_gen_float_sampled(own::span<float> dest, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift);

/*!
 * \brief This function generates a full GPS L1C-P code at the sample
 *  frequency sampling_freq.
 *
 */
void gps_l1c_p_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift);


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1C_SIGNAL_REPLICA_H
