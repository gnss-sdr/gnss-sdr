/*!
 * \file volk_gnsssdr_16ic_x2_dot_prod_16ic.h
 * \brief Volk protokernel: multiplies two 16 bits vectors and accumulates them
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * Volk protokernel that multiplies two 16 bits vectors (8 bits the real part 
 * and 8 bits the imaginary part) and accumulates them
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

#ifndef INCLUDED_volk_gnsssdr_16ic_x2_multiply_16ic_a_H
#define INCLUDED_volk_gnsssdr_16ic_x2_multiply_16ic_a_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <stdio.h>
#include <string.h>

#ifdef LV_HAVE_GENERIC
/*!
 \brief Multiplies the two input complex vectors, point-by-point, storing the result in the third vector
 \param cVector The vector where the result will be stored
 \param aVector One of the vectors to be multiplied
 \param bVector One of the vectors to be multiplied
 \param num_points The number of complex values in aVector and bVector to be multiplied together, accumulated and stored into cVector
 */
static inline void volk_gnsssdr_16ic_x2_multiply_16ic_generic(lv_16sc_t* result, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points)
{
    for (unsigned int n = 0; n < num_points; n++)
        {
            //r*a.r - i*a.i, i*a.r + r*a.i
            result[n] = in_a[n] * in_b[n];
        }
}

#endif /*LV_HAVE_GENERIC*/


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
static inline void volk_gnsssdr_16ic_x2_multiply_16ic_a_sse2(lv_16sc_t* out, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    __m128i a,b,c, c_sr, mask_imag, mask_real, real, imag, imag1,imag2, b_sl, a_sl, result;

    mask_imag = _mm_set_epi8(255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0);
    mask_real = _mm_set_epi8(0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255);

    const lv_16sc_t* _in_a = in_a;
    const lv_16sc_t* _in_b = in_b;
    lv_16sc_t* _out = out;
    for(unsigned int number = 0; number < sse_iters; number++)
        {

            //std::complex<T> memory structure: real part -> reinterpret_cast<cv T*>(a)[2*i]
            //imaginery part -> reinterpret_cast<cv T*>(a)[2*i + 1]
            // a[127:0]=[a3.i,a3.r,a2.i,a2.r,a1.i,a1.r,a0.i,a0.r]
            a = _mm_loadu_si128((__m128i*)_in_a); //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
            b = _mm_loadu_si128((__m128i*)_in_b);
            c = _mm_mullo_epi16 (a, b); // a3.i*b3.i, a3.r*b3.r, ....

            c_sr = _mm_srli_si128 (c, 2); // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
            real = _mm_subs_epi16 (c, c_sr);
            real = _mm_and_si128 (real, mask_real); // a3.r*b3.r-a3.i*b3.i , 0,  a3.r*b3.r- a3.i*b3.i

            b_sl = _mm_slli_si128(b, 2); // b3.r, b2.i ....
            a_sl = _mm_slli_si128(a, 2); // a3.r, a2.i ....

            imag1 = _mm_mullo_epi16(a, b_sl); // a3.i*b3.r, ....
            imag2 = _mm_mullo_epi16(b, a_sl); // b3.i*a3.r, ....

            imag = _mm_adds_epi16(imag1, imag2);
            imag = _mm_and_si128 (imag, mask_imag); // a3.i*b3.r+b3.i*a3.r, 0, ...

            result = _mm_or_si128 (real, imag);

            _mm_storeu_si128((__m128i*)_out, result);

            _in_a += 4;
            _in_b += 4;
            _out += 4;
        }

    for (unsigned int i = 0; i < (num_points % 4); ++i)
        {
            *_out++ = (*_in_a++) * (*_in_b++);
        }
}
#endif /* LV_HAVE_SSE2 */

#endif /*INCLUDED_volk_gnsssdr_16ic_x2_multiply_16ic_a_H*/
