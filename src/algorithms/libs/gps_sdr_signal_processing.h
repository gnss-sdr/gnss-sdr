/*!
 * \file gps_sdr_signal_processing.h
 * \brief This class implements various functions for GPS L1 CA signals
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_SDR_SIGNAL_PROCESSING_H
#define GNSS_SDR_GPS_SDR_SIGNAL_PROCESSING_H

#include <complex>
#include <cstdint>
#if HAS_STD_SPAN
#include <span>
#else
#include <gsl/gsl>
using std::span = gsl::span;
#endif

//! Generates int GPS L1 C/A code for the desired SV ID and code shift
void gps_l1_ca_code_gen_int(std::span<int32_t> _dest, int32_t _prn, uint32_t _chip_shift);

//! Generates float GPS L1 C/A code for the desired SV ID and code shift
void gps_l1_ca_code_gen_float(std::span<float> _dest, int32_t _prn, uint32_t _chip_shift);

//! Generates complex GPS L1 C/A code for the desired SV ID and code shift, and sampled to specific sampling frequency
void gps_l1_ca_code_gen_complex(std::span<std::complex<float>> _dest, int32_t _prn, uint32_t _chip_shift);

//! Generates N complex GPS L1 C/A codes for the desired SV ID and code shift
void gps_l1_ca_code_gen_complex_sampled(std::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs, uint32_t _chip_shift, uint32_t _ncodes);

//! Generates complex GPS L1 C/A code for the desired SV ID and code shift
void gps_l1_ca_code_gen_complex_sampled(std::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs, uint32_t _chip_shift);

#endif  // GNSS_SDR_GPS_SDR_SIGNAL_PROCESSING_H
