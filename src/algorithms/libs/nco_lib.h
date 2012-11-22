/*!
 * \file nco_lib.h
 * \brief A set of Numeric Controlled Oscillator (NCO) functions to generate the carrier wipeoff signal,
 *  regardless of system used
 *
 * \author Javier Arribas 2012, jarribas(at)cttc.es
 *
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

/*!
 * \brief Implements a conjugate complex exponential vector in std::complex<float> *d_carr_sign
 * containing int n_samples, with the starting phase .
 *
 */

#ifndef NCO_LIB_CC_H
#define	 NCO_LIB_CC_H

#include <gr_fxpt.h>
#include <xmmintrin.h>
#include <sse_mathfun.h>
#include <cmath>

void sse_nco(std::complex<float> *dest, int n_samples,float start_phase_rad, float phase_step_rad);

void fxp_nco(std::complex<float> *dest, int n_samples,float start_phase_rad, float phase_step_rad);

void std_nco(std::complex<float> *dest, int n_samples,float start_phase_rad, float phase_step_rad);

#endif //NCO_LIB_CC_H
