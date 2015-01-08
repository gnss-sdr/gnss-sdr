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



#ifndef GNSS_SDR_NCO_LIB_CC_H_
#define	GNSS_SDR_NCO_LIB_CC_H_

#include <cmath>
#include <gnuradio/fxpt.h>
//#include <xmmintrin.h>
//#include <sse_mathfun.h>


/*!
 * \brief Implements a complex conjugate exponential vector in std::complex<float> *d_carr_sign
 * containing int n_samples, with the starting phase float start_phase_rad and the pase step between vector elements
 * float phase_step_rad. This function uses a SSE CORDIC implementation.
 *
 */
void sse_nco(std::complex<float> *dest, int n_samples,float start_phase_rad, float phase_step_rad);

/*!
 * \brief Implements a complex conjugate exponential vector in std::complex<float> *d_carr_sign
 * containing int n_samples, with the starting phase float start_phase_rad and the pase step between vector elements
 * float phase_step_rad. This function uses the GNU Radio fixed point CORDIC implementation.
 *
 */
void fxp_nco(std::complex<float> *dest, int n_samples,float start_phase_rad, float phase_step_rad);

/*!
 * \brief Implements a complex conjugate exponential vector in std::complex<float> *d_carr_sign
 * containing int n_samples, with the starting phase float start_phase_rad and the pase step between vector elements
 * float phase_step_rad. This function uses the stdlib sin() and cos() implementation.
 *
 */

void std_nco(std::complex<float> *dest, int n_samples,float start_phase_rad, float phase_step_rad);

/*!
 * \brief Implements a complex conjugate exponential vector in std::complex<float> *d_carr_sign
 * containing int n_samples, with the starting phase float start_phase_rad and the pase step between vector elements
 * float phase_step_rad. This function uses the GNU Radio fixed point CORDIC implementation.
 *
 */

void fxp_nco_cpyref(std::complex<float> *dest, int n_samples,float start_phase_rad, float phase_step_rad);


/*!
 * \brief Implements a complex conjugate exponential vector in two separated float arrays (In-phase and Quadrature)
 * containing int n_samples, with the starting phase float start_phase_rad and the pase step between vector elements
 * float phase_step_rad. This function uses the GNU Radio fixed point CORDIC implementation.
 *
 */

void fxp_nco_IQ_split(float* I, float* Q, int n_samples,float start_phase_rad, float phase_step_rad);

#endif //NCO_LIB_CC_H
