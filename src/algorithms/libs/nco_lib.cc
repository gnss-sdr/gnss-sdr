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

#include "nco_lib.h"


typedef ALIGN16_BEG union {
  float f[4];
  int i[4];
  v4sf  v;
} ALIGN16_END V4SF;

void sse_nco(std::complex<float> *dest, int n_samples,float start_phase_rad, float phase_step_rad)
{
    //SSE NCO
    int sse_loops_four_op;
    int remnant_ops;
    sse_loops_four_op=(int)n_samples/4;
    remnant_ops=n_samples%4;
    V4SF vx, sin4, cos4;
    float phase_rad;
    phase_rad=start_phase_rad;

    int index=0;
    for(int i=0;i<sse_loops_four_op;i++)
    {
    	vx.f[0]=phase_rad;
    	phase_rad=phase_rad+phase_step_rad;
    	vx.f[1]=phase_rad;
    	phase_rad=phase_rad+phase_step_rad;
    	vx.f[2]=phase_rad;
    	phase_rad=phase_rad+phase_step_rad;
    	vx.f[3]=phase_rad;
    	phase_rad=phase_rad+phase_step_rad;
    	sincos_ps(vx.v, &sin4.v, &cos4.v);
    	dest[index] = std::complex<float>(cos4.f[0], -sin4.f[0]);
    	index++;
    	dest[index] = std::complex<float>(cos4.f[1], -sin4.f[1]);
    	index++;
    	dest[index] = std::complex<float>(cos4.f[2], -sin4.f[2]);
    	index++;
    	dest[index] = std::complex<float>(cos4.f[3], -sin4.f[3]);
    	index++;
    }
    for(int i=0;i<remnant_ops;i++)
    {
    	vx.f[i]=phase_rad;
    	phase_rad=phase_rad+phase_step_rad;
    }
    sincos_ps(vx.v, &sin4.v, &cos4.v);
    for(int i=0;i<remnant_ops;i++)
    {
    	dest[index] = std::complex<float>(cos4.f[i], -sin4.f[i]);
    	index++;
    }
}

void fxp_nco(std::complex<float> *dest, int n_samples,float start_phase_rad, float phase_step_rad)
{
	int phase_rad_i;
	phase_rad_i=gr_fxpt::float_to_fixed(start_phase_rad);
	int phase_step_rad_i;
	phase_step_rad_i=gr_fxpt::float_to_fixed(phase_step_rad);

	float sin_f,cos_f;

	for(int i = 0; i < n_samples; i++)
		{
			//using temp variables
			gr_fxpt::sincos(-phase_rad_i,&sin_f,&cos_f);
			dest[i] = gr_complex(cos_f, sin_f);
			phase_rad_i += phase_step_rad_i;

		}
}

void fxp_nco_cpyref(std::complex<float> *dest, int n_samples,float start_phase_rad, float phase_step_rad)
{
	int phase_rad_i;
	phase_rad_i=gr_fxpt::float_to_fixed(start_phase_rad);
	int phase_step_rad_i;
	phase_step_rad_i=gr_fxpt::float_to_fixed(phase_step_rad);

	float* vector_cpx;
	vector_cpx=(float*)dest;
	for(int i = 0; i < n_samples; i++)
		{
			//using references (may be it can be a problem for c++11 standard
			//gr_fxpt::sincos(phase_rad_i,&d_carr_sign[i].imag(),&d_carr_sign[i].real());
			gr_fxpt::sincos(-phase_rad_i,&vector_cpx[i*2+1],&vector_cpx[i*2]);
			phase_rad_i += phase_step_rad_i;

		}
}

void fxp_nco_IQ_split(float* I, float* Q , int n_samples,float start_phase_rad, float phase_step_rad)
{
	int phase_rad_i;
	phase_rad_i=gr_fxpt::float_to_fixed(start_phase_rad);
	int phase_step_rad_i;
	phase_step_rad_i=gr_fxpt::float_to_fixed(phase_step_rad);

	float sin_f,cos_f;
	for(int i = 0; i < n_samples; i++)
		{
			gr_fxpt::sincos(-phase_rad_i,&sin_f,&cos_f);
			I[i]=cos_f;
			Q[i]=sin_f;
			phase_rad_i += phase_step_rad_i;

		}
}



void std_nco(std::complex<float> *dest, int n_samples,float start_phase_rad, float phase_step_rad)
{
	float phase_rad;
	phase_rad=start_phase_rad;

	for(int i = 0; i < n_samples; i++)
		{
			// Using std::cos and std::sin
			dest[i] = gr_complex(std::cos(phase_rad), -std::sin(phase_rad));
	    	phase_rad=phase_rad+phase_step_rad;
		}
}
