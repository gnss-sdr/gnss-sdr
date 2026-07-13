/*!
 * \file beidou_b1c_signal_replica.h
 * \brief Library for BeiDou B1C signal replica generation
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
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
#ifndef GNSS_SDR_BEIDOU_B1C_SIGNAL_REPLICA_H
#define GNSS_SDR_BEIDOU_B1C_SIGNAL_REPLICA_H

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

void beidou_b1c_code_gen_int(own::span<int32_t> dest, const std::array<char, 3>& signal_id, int32_t prn);
void beidou_b1c_code_gen_sinboc11_float(own::span<float> dest, const std::array<char, 3>& signal_id, uint32_t prn);
void beidou_b1c_code_gen_float_sampled(own::span<float> dest, const std::array<char, 3>& signal_id,
    bool qmboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift, bool secondary_flag);
void beidou_b1c_code_gen_complex_sampled(own::span<std::complex<float>> dest, const std::array<char, 3>& signal_id,
    bool qmboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift, bool secondary_flag);
void beidou_b1c_code_gen_complex_sampled(own::span<std::complex<float>> dest, const std::array<char, 3>& signal_id,
    bool qmboc, uint32_t prn, int32_t sampling_freq, uint32_t chip_shift);

/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_B1C_SIGNAL_REPLICA_H
