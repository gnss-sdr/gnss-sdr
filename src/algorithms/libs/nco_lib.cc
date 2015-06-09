/*!
 * \file nco_lib.cc
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

#include "nco_lib.h"



void fxp_nco(std::complex<float> *dest, int n_samples, float start_phase_rad, float phase_step_rad)
{
    int phase_rad_i;
    phase_rad_i = gr::fxpt::float_to_fixed(start_phase_rad);
    int phase_step_rad_i;
    phase_step_rad_i = gr::fxpt::float_to_fixed(phase_step_rad);
    float sin_f,cos_f;

    for(int i = 0; i < n_samples; i++)
        {
            //using temp variables
            gr::fxpt::sincos(-phase_rad_i,&sin_f,&cos_f);
            dest[i] = gr_complex(cos_f, sin_f);
            phase_rad_i += phase_step_rad_i;
        }
}

void fxp_nco_cpyref(std::complex<float> *dest, int n_samples, float start_phase_rad, float phase_step_rad)
{
    int phase_rad_i;
    phase_rad_i = gr::fxpt::float_to_fixed(start_phase_rad);
    int phase_step_rad_i;
    phase_step_rad_i = gr::fxpt::float_to_fixed(phase_step_rad);

    float* vector_cpx;
    vector_cpx = (float*)dest;
    for(int i = 0; i < n_samples; i++)
        {
            //using references (maybe it can be a problem for c++11 ?)
            //gr_fxpt::sincos(phase_rad_i,&d_carr_sign[i].imag(),&d_carr_sign[i].real());
            gr::fxpt::sincos(-phase_rad_i, &vector_cpx[i*2+1], &vector_cpx[i*2]);
            phase_rad_i += phase_step_rad_i;

        }
}

void fxp_nco_IQ_split(float* I, float* Q , int n_samples,float start_phase_rad, float phase_step_rad)
{
    int phase_rad_i;
    phase_rad_i = gr::fxpt::float_to_fixed(start_phase_rad);
    int phase_step_rad_i;
    phase_step_rad_i = gr::fxpt::float_to_fixed(phase_step_rad);

    float sin_f,cos_f;
    for(int i = 0; i < n_samples; i++)
        {
            gr::fxpt::sincos(-phase_rad_i,&sin_f,&cos_f);
            I[i] = cos_f;
            Q[i] = sin_f;
            phase_rad_i += phase_step_rad_i;

        }
}



void std_nco(std::complex<float> *dest, int n_samples, float start_phase_rad, float phase_step_rad)
{
    float phase_rad;
    phase_rad = start_phase_rad;

    for(int i = 0; i < n_samples; i++)
        {
            // Using std::cos and std::sin
            dest[i] = gr_complex(std::cos(phase_rad), -std::sin(phase_rad));
            phase_rad = phase_rad+phase_step_rad;
        }
}
