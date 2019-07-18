/*!
 * \file beidou_b3i_signal_processing.h
 * \brief This class implements various functions for BeiDou B3I signals
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
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
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_BEIDOU_B3I_SIGNAL_PROCESSING_H_
#define GNSS_SDR_BEIDOU_B3I_SIGNAL_PROCESSING_H_

#include <complex>
#include <iostream>
#include <cstdint>
#include <array>
#include <algorithm>

#if HAS_SPAN
#include <span>
namespace gsl = std;
#else
#include <gsl/gsl>
#endif

//! Generates int BeiDou B3I code for the desired SV ID and code shift
void beidou_b3i_code_gen_int(gsl::span<int> _dest, signed int _prn, unsigned int _chip_shift);

//! Generates float BeiDou B3I code for the desired SV ID and code shift
void beidou_b3i_code_gen_float(gsl::span<float> _dest, signed int _prn, unsigned int _chip_shift);

//! Generates complex BeiDou B3I code for the desired SV ID and code shift, and sampled to specific sampling frequency
void beidou_b3i_code_gen_complex(gsl::span<std::complex<float>> _dest, signed int _prn, unsigned int _chip_shift);

//! Generates N complex BeiDou B3I codes for the desired SV ID and code shift
void beidou_b3i_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, unsigned int _prn, int _fs, unsigned int _chip_shift, unsigned int _ncodes);

//! Generates complex BeiDou B3I code for the desired SV ID and code shift
void beidou_b3i_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, unsigned int _prn, int _fs, unsigned int _chip_shift);

#endif /* GNSS_SDR_BEIDOU_B3I_SIGNAL_PROCESSING_H_ */
