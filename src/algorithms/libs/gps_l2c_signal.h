/*!
 * \file gps_l2c_signal.h
 * \brief This class implements signal generators for the GPS L2C signals
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_GPS_L2C_SIGNAL_H_
#define GNSS_SDR_GPS_L2C_SIGNAL_H_

#include <gsl/gsl>
#include <complex>
#include <cstdint>

//! Generates complex GPS L2C M code for the desired SV ID
void gps_l2c_m_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn);
void gps_l2c_m_code_gen_float(gsl::span<float> _dest, uint32_t _prn);

//! Generates complex GPS L2C M code for the desired SV ID, and sampled to specific sampling frequency
void gps_l2c_m_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs);

#endif  // GNSS_SDR_GPS_L2C_SIGNAL_H_
