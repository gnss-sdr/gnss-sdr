/*!
 * \file beidou_b2b_signal_replica.h
 * \brief BeiDou B2b_I ranging-code replica (BDS-SIS-ICD-B2b-1.0 Table 5-1)
 * \author Chandoss, 2026. huangchh37(at)mail2.sysu.edu.cn
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

#ifndef GNSS_SDR_BEIDOU_B2B_SIGNAL_REPLICA_H
#define GNSS_SDR_BEIDOU_B2B_SIGNAL_REPLICA_H

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

void beidou_b2b_code_gen_int(own::span<int> dest, int32_t prn, uint32_t chip_shift);
void beidou_b2b_code_gen_float(own::span<float> dest, int32_t prn, uint32_t chip_shift);
void beidou_b2b_code_gen_complex(own::span<std::complex<float>> dest, int32_t prn, uint32_t chip_shift);
void beidou_b2b_code_gen_complex_sampled(own::span<std::complex<float>> dest, uint32_t prn, int sampling_freq, uint32_t chip_shift);

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B2B_SIGNAL_REPLICA_H
