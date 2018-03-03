/*!
 * \file volk_gnsssdr_8i_x2_add_8i.h
 * \brief VOLK_GNSSSDR kernel: adds pairs of 8 bits (char) scalars.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that adds pairs of 8 bits (char) scalars
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
 * \page volk_gnsssdr_8i_x2_add_8i
 *
 * \b Overview
 *
 * Adds the two input vectors and store the results in the third vector.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_8i_x2_add_8i(char* cVector, const char* aVector, const char* bVector, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li aVector: One of the vectors of to be added.
 * \li bVector: The other vector to be added.
 * \li num_points: Number of values in \p aVector and \p bVector to be added together and stored into \p cVector
 *
 * \b Outputs
 * \li cVector: The vector where the result will be stored.
 *
 */

#ifndef INCLUDED_volk_gnsssdr_8i_x2_add_8i_H
#define INCLUDED_volk_gnsssdr_8i_x2_add_8i_H


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_8i_x2_add_8i_u_sse2(char* cVector, const char* aVector, const char* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;
    unsigned int number;
    unsigned int i;
    char* cPtr = cVector;
    const char* aPtr = aVector;
    const char* bPtr = bVector;

    __m128i aVal, bVal, cVal;

    for (number = 0; number < sse_iters; number++)
        {
            aVal = _mm_loadu_si128((__m128i*)aPtr);
            bVal = _mm_loadu_si128((__m128i*)bPtr);

            cVal = _mm_add_epi8(aVal, bVal);

            _mm_storeu_si128((__m128i*)cPtr, cVal);  // Store the results back into the C container

            aPtr += 16;
            bPtr += 16;
            cPtr += 16;
        }

    for (i = sse_iters * 16; i < num_points; ++i)
        {
            *cPtr++ = (*aPtr++) + (*bPtr++);
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_8i_x2_add_8i_u_avx2(char* cVector, const char* aVector, const char* bVector, unsigned int num_points)
{
    const unsigned int avx_iters = num_points / 32;
    unsigned int number;
    unsigned int i;
    char* cPtr = cVector;
    const char* aPtr = aVector;
    const char* bPtr = bVector;

    __m256i aVal, bVal, cVal;

    for (number = 0; number < avx_iters; number++)
        {
            aVal = _mm256_loadu_si256((__m256i*)aPtr);
            bVal = _mm256_loadu_si256((__m256i*)bPtr);

            cVal = _mm256_add_epi8(aVal, bVal);

            _mm256_storeu_si256((__m256i*)cPtr, cVal);  // Store the results back into the C container

            aPtr += 32;
            bPtr += 32;
            cPtr += 32;
        }

    for (i = avx_iters * 32; i < num_points; ++i)
        {
            *cPtr++ = (*aPtr++) + (*bPtr++);
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_8i_x2_add_8i_generic(char* cVector, const char* aVector, const char* bVector, unsigned int num_points)
{
    char* cPtr = cVector;
    const char* aPtr = aVector;
    const char* bPtr = bVector;
    unsigned int number;

    for (number = 0; number < num_points; number++)
        {
            *cPtr++ = (*aPtr++) + (*bPtr++);
        }
}
#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_8i_x2_add_8i_a_sse2(char* cVector, const char* aVector, const char* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;
    unsigned int number;
    unsigned int i;
    char* cPtr = cVector;
    const char* aPtr = aVector;
    const char* bPtr = bVector;

    __m128i aVal, bVal, cVal;

    for (number = 0; number < sse_iters; number++)
        {
            aVal = _mm_load_si128((__m128i*)aPtr);
            bVal = _mm_load_si128((__m128i*)bPtr);

            cVal = _mm_add_epi8(aVal, bVal);

            _mm_store_si128((__m128i*)cPtr, cVal);  // Store the results back into the C container

            aPtr += 16;
            bPtr += 16;
            cPtr += 16;
        }

    for (i = sse_iters * 16; i < num_points; ++i)
        {
            *cPtr++ = (*aPtr++) + (*bPtr++);
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_8i_x2_add_8i_a_avx2(char* cVector, const char* aVector, const char* bVector, unsigned int num_points)
{
    const unsigned int avx_iters = num_points / 32;
    unsigned int number;
    unsigned int i;
    char* cPtr = cVector;
    const char* aPtr = aVector;
    const char* bPtr = bVector;

    __m256i aVal, bVal, cVal;

    for (number = 0; number < avx_iters; number++)
        {
            aVal = _mm256_load_si256((__m256i*)aPtr);
            bVal = _mm256_load_si256((__m256i*)bPtr);

            cVal = _mm256_add_epi8(aVal, bVal);

            _mm256_store_si256((__m256i*)cPtr, cVal);  // Store the results back into the C container

            aPtr += 32;
            bPtr += 32;
            cPtr += 32;
        }

    for (i = avx_iters * 32; i < num_points; ++i)
        {
            *cPtr++ = (*aPtr++) + (*bPtr++);
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_ORC

extern void volk_gnsssdr_8i_x2_add_8i_a_orc_impl(char* cVector, const char* aVector, const char* bVector, unsigned int num_points);
static inline void volk_gnsssdr_8i_x2_add_8i_u_orc(char* cVector, const char* aVector, const char* bVector, unsigned int num_points)
{
    volk_gnsssdr_8i_x2_add_8i_a_orc_impl(cVector, aVector, bVector, num_points);
}
#endif /* LV_HAVE_ORC */

#endif /* INCLUDED_volk_gnsssdr_8i_x2_add_8i_H */
