/*!
 * \file navic_l5_signal_replica.h
 * \brief  This file implements functions for NavIC L5 SPS signal replica generation
 * \author GNSS-SDR developers, 2024-2025
 *
 * Ref: IRNSS SIS ICD for SPS, Version 1.1, August 2017
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NAVIC_L5_SIGNAL_REPLICA_H
#define GNSS_SDR_NAVIC_L5_SIGNAL_REPLICA_H

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

/*!
 * \brief Generates the NavIC L5 SPS PRN code for a given satellite PRN.
 *
 * Uses Gold code generation with two 10-bit LFSRs (G1 and G2).
 * G1 polynomial: X^10 + X^3 + 1
 * G2 polynomial: X^10 + X^9 + X^8 + X^6 + X^3 + X^2 + 1
 * G1 initialized to all 1s.
 * G2 initialized per-PRN from ICD Table 7.
 *
 * \param[out] dest  Pointer to output array (minimum 1023 elements).
 *                   Values are +1 or -1 (int32_t).
 * \param[in]  prn   PRN ID (1 to 14)
 * \param[in]  chip_shift  Number of chips to cyclically shift the code (default: 0)
 */
void navic_l5_code_gen_int(own::span<int32_t> dest, int32_t prn, uint32_t chip_shift);

/*!
 * \brief Generates the NavIC L5 SPS PRN code as float values.
 */
void navic_l5_code_gen_float(own::span<float> dest, int32_t prn, uint32_t chip_shift);

/*!
 * \brief Generates the NavIC L5 SPS PRN code as complex float values.
 */
void navic_l5_code_gen_complex(own::span<std::complex<float>> dest, int32_t prn, uint32_t chip_shift);

/*!
 * \brief Generates a sampled NavIC L5 SPS PRN code (complex), resampled to a given sampling frequency.
 */
void navic_l5_code_gen_complex_sampled(own::span<std::complex<float>> dest,
    int32_t prn,
    int32_t sampling_freq,
    uint32_t chip_shift);

/** \} */

#endif  // GNSS_SDR_NAVIC_L5_SIGNAL_REPLICA_H
