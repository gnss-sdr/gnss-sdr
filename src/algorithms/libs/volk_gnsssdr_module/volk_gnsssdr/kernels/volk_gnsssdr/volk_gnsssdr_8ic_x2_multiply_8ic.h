/*!
 * \file volk_gnsssdr_8ic_x2_multiply_8ic.h
 * \brief VOLK_GNSSSDR kernel: multiplies two 16 bits vectors.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that multiplies two 16 bits vectors (8 bits the real part
 * and 8 bits the imaginary part)
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

/*!
 * \page volk_gnsssdr_8ic_x2_multiply_8ic
 *
 * \b Overview
 *
 * Multiplies two input complex vectors, point-by-point, storing the result in the third vector
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_8ic_x2_multiply_8ic(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li aVector: One of the vectors to be multiplied
 * \li bVector: The other vector to be multiplied
 * \li num_points: The number of complex data points.
 *
 * \b Outputs
 * \li cVector: The vector where the result will be stored
 *
 */

#ifndef INCLUDED_volk_gnsssdr_8ic_x2_multiply_8ic_H
#define INCLUDED_volk_gnsssdr_8ic_x2_multiply_8ic_H

#include <volk_gnsssdr/volk_gnsssdr_complex.h>

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_u_sse2(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    unsigned int number;
    unsigned int i;
    __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc;
    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    const lv_8sc_t* b = bVector;

    mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);

    for (number = 0; number < sse_iters; number++)
        {
            x = _mm_loadu_si128((__m128i*)a);
            y = _mm_loadu_si128((__m128i*)b);

            imagx = _mm_srli_si128(x, 1);
            imagx = _mm_and_si128(imagx, mult1);
            realx = _mm_and_si128(x, mult1);

            imagy = _mm_srli_si128(y, 1);
            imagy = _mm_and_si128(imagy, mult1);
            realy = _mm_and_si128(y, mult1);

            realx_mult_realy = _mm_mullo_epi16(realx, realy);
            imagx_mult_imagy = _mm_mullo_epi16(imagx, imagy);
            realx_mult_imagy = _mm_mullo_epi16(realx, imagy);
            imagx_mult_realy = _mm_mullo_epi16(imagx, realy);

            realc = _mm_sub_epi16(realx_mult_realy, imagx_mult_imagy);
            realc = _mm_and_si128(realc, mult1);
            imagc = _mm_add_epi16(realx_mult_imagy, imagx_mult_realy);
            imagc = _mm_and_si128(imagc, mult1);
            imagc = _mm_slli_si128(imagc, 1);

            totalc = _mm_or_si128(realc, imagc);

            _mm_storeu_si128((__m128i*)c, totalc);

            a += 8;
            b += 8;
            c += 8;
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_u_sse4_1(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    unsigned int number;
    unsigned int i;
    __m128i x, y;
    __m128i mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc;
    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    const lv_8sc_t* b = bVector;

    _mm_setzero_si128();
    mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);

    for (number = 0; number < sse_iters; number++)
        {
            x = _mm_lddqu_si128((__m128i*)a);
            y = _mm_lddqu_si128((__m128i*)b);

            imagx = _mm_srli_si128(x, 1);
            imagx = _mm_and_si128(imagx, mult1);
            realx = _mm_and_si128(x, mult1);

            imagy = _mm_srli_si128(y, 1);
            imagy = _mm_and_si128(imagy, mult1);
            realy = _mm_and_si128(y, mult1);

            realx_mult_realy = _mm_mullo_epi16(realx, realy);
            imagx_mult_imagy = _mm_mullo_epi16(imagx, imagy);
            realx_mult_imagy = _mm_mullo_epi16(realx, imagy);
            imagx_mult_realy = _mm_mullo_epi16(imagx, realy);

            realc = _mm_sub_epi16(realx_mult_realy, imagx_mult_imagy);
            imagc = _mm_add_epi16(realx_mult_imagy, imagx_mult_realy);
            imagc = _mm_slli_si128(imagc, 1);

            totalc = _mm_blendv_epi8(imagc, realc, mult1);

            _mm_storeu_si128((__m128i*)c, totalc);

            a += 8;
            b += 8;
            c += 8;
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE4_1 */


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_generic(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    lv_8sc_t* cPtr = cVector;
    const lv_8sc_t* aPtr = aVector;
    const lv_8sc_t* bPtr = bVector;
    unsigned int number;

    for (number = 0; number < num_points; number++)
        {
            *cPtr++ = (*aPtr++) * (*bPtr++);
        }
}
#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_a_sse2(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    unsigned int number;
    unsigned int i;
    __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc;
    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    const lv_8sc_t* b = bVector;

    mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);

    for (number = 0; number < sse_iters; number++)
        {
            x = _mm_load_si128((__m128i*)a);
            y = _mm_load_si128((__m128i*)b);

            imagx = _mm_srli_si128(x, 1);
            imagx = _mm_and_si128(imagx, mult1);
            realx = _mm_and_si128(x, mult1);

            imagy = _mm_srli_si128(y, 1);
            imagy = _mm_and_si128(imagy, mult1);
            realy = _mm_and_si128(y, mult1);

            realx_mult_realy = _mm_mullo_epi16(realx, realy);
            imagx_mult_imagy = _mm_mullo_epi16(imagx, imagy);
            realx_mult_imagy = _mm_mullo_epi16(realx, imagy);
            imagx_mult_realy = _mm_mullo_epi16(imagx, realy);

            realc = _mm_sub_epi16(realx_mult_realy, imagx_mult_imagy);
            realc = _mm_and_si128(realc, mult1);
            imagc = _mm_add_epi16(realx_mult_imagy, imagx_mult_realy);
            imagc = _mm_and_si128(imagc, mult1);
            imagc = _mm_slli_si128(imagc, 1);

            totalc = _mm_or_si128(realc, imagc);

            _mm_store_si128((__m128i*)c, totalc);

            a += 8;
            b += 8;
            c += 8;
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_a_sse4_1(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    unsigned int number;
    unsigned int i;
    __m128i x, y;
    __m128i mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc;
    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    const lv_8sc_t* b = bVector;

    _mm_setzero_si128();
    mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);

    for (number = 0; number < sse_iters; number++)
        {
            x = _mm_load_si128((__m128i*)a);
            y = _mm_load_si128((__m128i*)b);

            imagx = _mm_srli_si128(x, 1);
            imagx = _mm_and_si128(imagx, mult1);
            realx = _mm_and_si128(x, mult1);

            imagy = _mm_srli_si128(y, 1);
            imagy = _mm_and_si128(imagy, mult1);
            realy = _mm_and_si128(y, mult1);

            realx_mult_realy = _mm_mullo_epi16(realx, realy);
            imagx_mult_imagy = _mm_mullo_epi16(imagx, imagy);
            realx_mult_imagy = _mm_mullo_epi16(realx, imagy);
            imagx_mult_realy = _mm_mullo_epi16(imagx, realy);

            realc = _mm_sub_epi16(realx_mult_realy, imagx_mult_imagy);
            imagc = _mm_add_epi16(realx_mult_imagy, imagx_mult_realy);
            imagc = _mm_slli_si128(imagc, 1);

            totalc = _mm_blendv_epi8(imagc, realc, mult1);

            _mm_store_si128((__m128i*)c, totalc);

            a += 8;
            b += 8;
            c += 8;
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE4_1 */


#ifdef LV_HAVE_ORC

extern void volk_gnsssdr_8ic_x2_multiply_8ic_a_orc_impl(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points);

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_u_orc(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    volk_gnsssdr_8ic_x2_multiply_8ic_a_orc_impl(cVector, aVector, bVector, num_points);
}
#endif /* LV_HAVE_ORC */

#endif /* INCLUDED_volk_gnsssdr_8ic_x2_multiply_8ic_H */
