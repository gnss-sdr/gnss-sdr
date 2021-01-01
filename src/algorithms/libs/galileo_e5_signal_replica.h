/*!
 * \file galileo_e5_signal_replica.h
 * \brief This library implements various functions for Galileo E5 signal
 * replica generation
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \author Piyush Gupta, 2020. piyush04111999@gmail.com
 * \note Code added as part of GSoC 2020 Program.
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

#ifndef GNSS_SDR_GALILEO_E5_SIGNAL_REPLICA_H
#define GNSS_SDR_GALILEO_E5_SIGNAL_REPLICA_H

#include <array>
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


/*!
 * \brief Generates Galileo E5a code at 1 sample/chip
 */
void galileo_e5_a_code_gen_complex_primary(own::span<std::complex<float>> dest,
    int32_t prn,
    const std::array<char, 3>& signal_id);


/*!
 * \brief Generates Galileo E5a complex code, shifted to the desired chip and
 * sampled at a frequency sampling_freq
 */
void galileo_e5_a_code_gen_complex_sampled(own::span<std::complex<float>> dest,
    uint32_t prn,
    const std::array<char, 3>& signal_id,
    int32_t sampling_freq,
    uint32_t chip_shift);


/*!
 * \brief Generates Galileo E5b code at 1 sample/chip
 */
void galileo_e5_b_code_gen_complex_primary(own::span<std::complex<float>> dest,
    int32_t prn,
    const std::array<char, 3>& signal_id);


/*!
 * \brief Generates Galileo E5b complex code, shifted to the desired chip and
 * sampled at a frequency sampling_freq
 */
void galileo_e5_b_code_gen_complex_sampled(own::span<std::complex<float>> dest,
    uint32_t prn,
    const std::array<char, 3>& signal_id,
    int32_t sampling_freq,
    uint32_t chip_shift);


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E5_SIGNAL_REPLICA_H
