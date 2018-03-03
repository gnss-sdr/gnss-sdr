/*!
 * \file volk_gnsssdr_8u_x2_multiply_8u.h
 * \brief VOLK_GNSSSDR kernel: multiplies unsigned char values.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that multiplies unsigned char values (8 bits data)
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

/*!
 * \page volk_gnsssdr_8u_x2_multiply_8u
 *
 * \b Overview
 *
 * Multiplies two input vectors of unsigned char, point-by-point, storing the result in the third vector
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_8u_x2_multiply_8u(unsigned char* cChar, const unsigned char* aChar, const unsigned char* bChar, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li aChar: One of the vectors to be multiplied
 * \li bChar: The other vector to be multiplied
 * \li num_points: The number of complex data points.
 *
 * \b Outputs
 * \li cChar: The vector where the result will be stored
 *
 */


#ifndef INCLUDED_volk_gnsssdr_8u_x2_multiply_8u_H
#define INCLUDED_volk_gnsssdr_8u_x2_multiply_8u_H

#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_8u_x2_multiply_8u_u_avx2(unsigned char* cChar, const unsigned char* aChar, const unsigned char* bChar, unsigned int num_points)
{
    const unsigned int avx2_iters = num_points / 32;
    unsigned int number;
    unsigned int i;

    __m256i x, y, x1, x2, y1, y2, mult1, x1_mult_y1, x2_mult_y2, tmp, tmp1, tmp2, totalc;
    unsigned char* c = cChar;
    const unsigned char* a = aChar;
    const unsigned char* b = bChar;

    for (number = 0; number < avx2_iters; number++)
        {
            x = _mm256_loadu_si256((__m256i*)a);
            y = _mm256_loadu_si256((__m256i*)b);

            mult1 = _mm256_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);
            x1 = _mm256_srli_si256(x, 1);
            x1 = _mm256_and_si256(x1, mult1);
            x2 = _mm256_and_si256(x, mult1);

            y1 = _mm256_srli_si256(y, 1);
            y1 = _mm256_and_si256(y1, mult1);
            y2 = _mm256_and_si256(y, mult1);

            x1_mult_y1 = _mm256_mullo_epi16(x1, y1);
            x2_mult_y2 = _mm256_mullo_epi16(x2, y2);

            tmp = _mm256_and_si256(x1_mult_y1, mult1);
            tmp1 = _mm256_slli_si256(tmp, 1);
            tmp2 = _mm256_and_si256(x2_mult_y2, mult1);
            totalc = _mm256_or_si256(tmp1, tmp2);

            _mm256_storeu_si256((__m256i*)c, totalc);

            a += 32;
            b += 32;
            c += 32;
        }

    for (i = avx2_iters * 32; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

static inline void volk_gnsssdr_8u_x2_multiply_8u_u_sse3(unsigned char* cChar, const unsigned char* aChar, const unsigned char* bChar, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;
    unsigned int number;
    unsigned int i;

    __m128i x, y, x1, x2, y1, y2, mult1, x1_mult_y1, x2_mult_y2, tmp, tmp1, tmp2, totalc;
    unsigned char* c = cChar;
    const unsigned char* a = aChar;
    const unsigned char* b = bChar;

    for (number = 0; number < sse_iters; number++)
        {
            x = _mm_lddqu_si128((__m128i*)a);
            y = _mm_lddqu_si128((__m128i*)b);

            mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);
            x1 = _mm_srli_si128(x, 1);
            x1 = _mm_and_si128(x1, mult1);
            x2 = _mm_and_si128(x, mult1);

            y1 = _mm_srli_si128(y, 1);
            y1 = _mm_and_si128(y1, mult1);
            y2 = _mm_and_si128(y, mult1);

            x1_mult_y1 = _mm_mullo_epi16(x1, y1);
            x2_mult_y2 = _mm_mullo_epi16(x2, y2);

            tmp = _mm_and_si128(x1_mult_y1, mult1);
            tmp1 = _mm_slli_si128(tmp, 1);
            tmp2 = _mm_and_si128(x2_mult_y2, mult1);
            totalc = _mm_or_si128(tmp1, tmp2);

            _mm_storeu_si128((__m128i*)c, totalc);

            a += 16;
            b += 16;
            c += 16;
        }

    for (i = sse_iters * 16; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE3 */

#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_8u_x2_multiply_8u_generic(unsigned char* cChar, const unsigned char* aChar, const unsigned char* bChar, unsigned int num_points)
{
    unsigned char* cPtr = cChar;
    const unsigned char* aPtr = aChar;
    const unsigned char* bPtr = bChar;
    unsigned int number;

    for (number = 0; number < num_points; number++)
        {
            *cPtr++ = (*aPtr++) * (*bPtr++);
        }
}
#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

static inline void volk_gnsssdr_8u_x2_multiply_8u_a_sse3(unsigned char* cChar, const unsigned char* aChar, const unsigned char* bChar, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;
    unsigned int number;
    unsigned int i;
    __m128i x, y, x1, x2, y1, y2, mult1, x1_mult_y1, x2_mult_y2, tmp, tmp1, tmp2, totalc;
    unsigned char* c = cChar;
    const unsigned char* a = aChar;
    const unsigned char* b = bChar;

    for (number = 0; number < sse_iters; number++)
        {
            x = _mm_load_si128((__m128i*)a);
            y = _mm_load_si128((__m128i*)b);

            mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);
            x1 = _mm_srli_si128(x, 1);
            x1 = _mm_and_si128(x1, mult1);
            x2 = _mm_and_si128(x, mult1);

            y1 = _mm_srli_si128(y, 1);
            y1 = _mm_and_si128(y1, mult1);
            y2 = _mm_and_si128(y, mult1);

            x1_mult_y1 = _mm_mullo_epi16(x1, y1);
            x2_mult_y2 = _mm_mullo_epi16(x2, y2);

            tmp = _mm_and_si128(x1_mult_y1, mult1);
            tmp1 = _mm_slli_si128(tmp, 1);
            tmp2 = _mm_and_si128(x2_mult_y2, mult1);
            totalc = _mm_or_si128(tmp1, tmp2);

            _mm_store_si128((__m128i*)c, totalc);

            a += 16;
            b += 16;
            c += 16;
        }

    for (i = sse_iters * 16; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_8u_x2_multiply_8u_a_avx2(unsigned char* cChar, const unsigned char* aChar, const unsigned char* bChar, unsigned int num_points)
{
    const unsigned int avx2_iters = num_points / 32;
    unsigned int number;
    unsigned int i;

    __m256i x, y, x1, x2, y1, y2, mult1, x1_mult_y1, x2_mult_y2, tmp, tmp1, tmp2, totalc;
    unsigned char* c = cChar;
    const unsigned char* a = aChar;
    const unsigned char* b = bChar;

    for (number = 0; number < avx2_iters; number++)
        {
            x = _mm256_load_si256((__m256i*)a);
            y = _mm256_load_si256((__m256i*)b);

            mult1 = _mm256_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);
            x1 = _mm256_srli_si256(x, 1);
            x1 = _mm256_and_si256(x1, mult1);
            x2 = _mm256_and_si256(x, mult1);

            y1 = _mm256_srli_si256(y, 1);
            y1 = _mm256_and_si256(y1, mult1);
            y2 = _mm256_and_si256(y, mult1);

            x1_mult_y1 = _mm256_mullo_epi16(x1, y1);
            x2_mult_y2 = _mm256_mullo_epi16(x2, y2);

            tmp = _mm256_and_si256(x1_mult_y1, mult1);
            tmp1 = _mm256_slli_si256(tmp, 1);
            tmp2 = _mm256_and_si256(x2_mult_y2, mult1);
            totalc = _mm256_or_si256(tmp1, tmp2);

            _mm256_store_si256((__m256i*)c, totalc);

            a += 32;
            b += 32;
            c += 32;
        }

    for (i = avx2_iters * 32; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_ORC

extern void volk_gnsssdr_8u_x2_multiply_8u_a_orc_impl(unsigned char* cVector, const unsigned char* aVector, const unsigned char* bVector, unsigned int num_points);
static inline void volk_gnsssdr_8u_x2_multiply_8u_u_orc(unsigned char* cVector, const unsigned char* aVector, const unsigned char* bVector, unsigned int num_points)
{
    volk_gnsssdr_8u_x2_multiply_8u_a_orc_impl(cVector, aVector, bVector, num_points);
}
#endif /* LV_HAVE_ORC */

#endif /* INCLUDED_volk_gnsssdr_8u_x2_multiply_8u_H */
