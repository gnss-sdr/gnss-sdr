/*!
 * \file glonass_l1_signal_processing.h
 * \brief This class implements various functions for GLONASS L1 CA signals
 * \author Gabriel Araujo, 2017. gabriel.araujo(at)ieee.org
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GLONASS_SDR_SIGNAL_PROCESSING_H_
#define GNSS_SDR_GLONASS_SDR_SIGNAL_PROCESSING_H_

#include <gsl/gsl>
#include <complex>
#include <cstdint>

//! Generates complex GLONASS L1 C/A code for the desired SV ID and code shift, and sampled to specific sampling frequency
void glonass_l1_ca_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _chip_shift);

//! Generates N complex GLONASS L1 C/A codes for the desired SV ID and code shift
void glonass_l1_ca_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, int32_t _fs, uint32_t _chip_shift, uint32_t _ncodes);

//! Generates complex GLONASS L1 C/A code for the desired SV ID and code shift
void glonass_l1_ca_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, int32_t _fs, uint32_t _chip_shift);

#endif /* GNSS_SDR_GLONASS_SDR_SIGNAL_PROCESSING_H_ */
