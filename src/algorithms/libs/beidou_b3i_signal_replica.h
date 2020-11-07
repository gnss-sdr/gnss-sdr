/*!
 * \file beidou_b3i_signal_replica.h
 * \brief This file implements various functions for BeiDou B3I signal replica
 * generation
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
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
void beidou_b3i_code_gen_int(own::span<int> _dest, int32_t _prn, uint32_t _chip_shift);

//! Generates float BeiDou B3I code for the desired SV ID and code shift
void beidou_b3i_code_gen_float(own::span<float> _dest, int32_t _prn, uint32_t _chip_shift);

//! Generates complex BeiDou B3I code for the desired SV ID and code shift, and sampled to specific sampling frequency
void beidou_b3i_code_gen_complex(own::span<std::complex<float>> _dest, int32_t _prn, uint32_t _chip_shift);

//! Generates N complex BeiDou B3I codes for the desired SV ID and code shift
void beidou_b3i_code_gen_complex_sampled(own::span<std::complex<float>> _dest, uint32_t _prn, int _fs, uint32_t _chip_shift, uint32_t _ncodes);

//! Generates complex BeiDou B3I code for the desired SV ID and code shift
void beidou_b3i_code_gen_complex_sampled(own::span<std::complex<float>> _dest, uint32_t _prn, int _fs, uint32_t _chip_shift);


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B3I_SIGNAL_REPLICA_H
