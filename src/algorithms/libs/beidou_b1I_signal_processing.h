/*!
 * \file beidou_b1I_signal_processing.h
 * \brief This class implements various functions for BeiDou B1I signals
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#ifndef BEIDOU_B1I_SDR_SIGNAL_PROCESSING_H_
#define BEIDOU_B1I_SDR_SIGNAL_PROCESSING_H_

#include <complex>

//!Generates int GPS L1 C/A code for the desired SV ID and code shift
void beidou_b1i_code_gen_int(int* _dest, signed int _prn, unsigned int _chip_shift);

//!Generates float GPS L1 C/A code for the desired SV ID and code shift
void beidou_b1i_code_gen_float(float* _dest, signed int _prn, unsigned int _chip_shift);

//!Generates complex GPS L1 C/A code for the desired SV ID and code shift, and sampled to specific sampling frequency
void beidou_b1i_code_gen_complex(std::complex<float>* _dest, signed int _prn, unsigned int _chip_shift);

//! Generates N complex GPS L1 C/A codes for the desired SV ID and code shift
void beidou_b1i_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, int _fs, unsigned int _chip_shift, unsigned int _ncodes);

//! Generates complex GPS L1 C/A code for the desired SV ID and code shift
void beidou_b1i_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, int _fs, unsigned int _chip_shift);

#endif /* BEIDOU_B1I_SDR_SIGNAL_PROCESSING_H_ */
