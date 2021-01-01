/*!
 * \file beidou_b3i_signal_replica.h
 * \brief This file implements various functions for BeiDou B3I signal replica
 * generation
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
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

#ifndef GNSS_SDR_BEIDOU_B3I_SIGNAL_REPLICA_H
#define GNSS_SDR_BEIDOU_B3I_SIGNAL_REPLICA_H

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


//! Generates int BeiDou B3I code for the desired SV ID and code shift
void beidou_b3i_code_gen_int(own::span<int> dest, int32_t prn, uint32_t chip_shift);

//! Generates float BeiDou B3I code for the desired SV ID and code shift
void beidou_b3i_code_gen_float(own::span<float> dest, int32_t prn, uint32_t chip_shift);

//! Generates complex BeiDou B3I code for the desired SV ID and code shift
void beidou_b3i_code_gen_complex(own::span<std::complex<float>> dest, int32_t prn, uint32_t chip_shift);

//! Generates complex BeiDou B3I code for the desired SV ID and code shift, and sampled to specific sampling frequency
void beidou_b3i_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int sampling_freq, uint32_t chip_shift);


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B3I_SIGNAL_REPLICA_H
