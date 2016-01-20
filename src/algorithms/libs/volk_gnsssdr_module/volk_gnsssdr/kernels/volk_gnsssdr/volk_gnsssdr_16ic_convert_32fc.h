/*!
 * \file volk_gnsssdr_32fc_convert_16ic.h
 * \brief Volk protokernel: converts 16 bit integer complex complex values to  32 bits float complex values
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
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


#include <inttypes.h>
#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>

#ifndef INCLUDED_volk_gnsssdr_16ic_convert_32fc_H
#define INCLUDED_volk_gnsssdr_16ic_convert_32fc_H


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_convert_32fc_generic(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
{
    for(unsigned int i = 0; i < num_points; i++)
        {
    		outputVector[i]=lv_cmake((float)lv_creal(inputVector[i]),(float)lv_cimag(inputVector[i]));
        }
}
#endif /* LV_HAVE_GENERIC */

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_convert_32fc_a_sse2(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 2;

    const lv_16sc_t* _in = inputVector;
    lv_32fc_t* _out = outputVector;
    __m128 a;
    for(unsigned int number = 0; number < sse_iters; number++)
        {
	 	 	 a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0]))); // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
    	     _mm_store_ps((float*)_out, a);
    	     _in+=2;
    	     _out+=2;
             //*_out++ = lv_cmake((float)lv_creal(*_in),(float)lv_cimag(*_in));
             //_in++;
             //*_out++ = lv_cmake((float)lv_creal(*_in),(float)lv_cimag(*_in));
             //_in++;
        }
    for (unsigned int i = 0; i < (num_points % 2); ++i)
        {
            *_out++ = lv_cmake((float)lv_creal(*_in),(float)lv_cimag(*_in));
            _in++;
        }

}
#endif /* LV_HAVE_SSE2 */

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_convert_32fc_u_sse2(lv_32fc_t* outputVector, const lv_16sc_t* inputVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 2;

    const lv_16sc_t* _in = inputVector;
    lv_32fc_t* _out = outputVector;
    __m128 a;
    for(unsigned int number = 0; number < sse_iters; number++)
        {
	 	 	 a = _mm_set_ps((float)(lv_cimag(_in[1])), (float)(lv_creal(_in[1])), (float)(lv_cimag(_in[0])), (float)(lv_creal(_in[0]))); // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
	 	 	 _mm_storeu_ps((float*)_out, a);
    	     _in+=2;
    	     _out+=2;
        }
    for (unsigned int i = 0; i < (num_points % 2); ++i)
        {
            *_out++ = lv_cmake((float)lv_creal(*_in),(float)lv_cimag(*_in));
            _in++;
        }

}
#endif /* LV_HAVE_SSE2 */

// SSE4.1
// a = _mm_load_si128((__m128i*)_in); //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
//use _mm_cvtepi16_epi32 !!!!

#endif /* INCLUDED_volk_gnsssdr_32fc_convert_16ic_u_H */
