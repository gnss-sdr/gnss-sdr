/*!
 * \file volk_gnsssdr_64f_accumulator_64f.h
 * \brief VOLK_GNSSSDR kernel: 64 bits (double) scalar accumulator.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that implements an accumulator of double (64-bit float) values.
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
 * \page volk_gnsssdr_64f_accumulator_64f
 *
 * \b Overview
 *
 * Accumulates the values in the input buffer.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_64f_accumulator_64f(double* result, const double* inputBuffer, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li inputBuffer: The buffer of data to be accumulated.
 * \li num_points:  The number of values in \p inputBuffer to be accumulated.
 *
 * \b Outputs
 * \li result: The accumulated result.
 *
 */

#ifndef INCLUDED_volk_gnsssdr_64f_accumulator_64f_u_H
#define INCLUDED_volk_gnsssdr_64f_accumulator_64f_u_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>

#ifdef LV_HAVE_AVX
#include <immintrin.h>

static inline void volk_gnsssdr_64f_accumulator_64f_u_avx(double* result, const double* inputBuffer, unsigned int num_points)
{
    double returnValue = 0;
    const unsigned int sse_iters = num_points / 4;
    unsigned int number;
    unsigned int i;
    const double* aPtr = inputBuffer;

    __VOLK_ATTR_ALIGNED(32)
    double tempBuffer[4];
    __m256d accumulator = _mm256_setzero_pd();
    __m256d aVal = _mm256_setzero_pd();

    for (number = 0; number < sse_iters; number++)
        {
            aVal = _mm256_loadu_pd(aPtr);
            accumulator = _mm256_add_pd(accumulator, aVal);
            aPtr += 4;
        }

    _mm256_storeu_pd((double*)tempBuffer, accumulator);

    for (i = 0; i < 4; ++i)
        {
            returnValue += tempBuffer[i];
        }

    for (i = 0; i < (num_points % 4); ++i)
        {
            returnValue += (*aPtr++);
        }

    *result = returnValue;
}
#endif /* LV_HAVE_AVX */


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

static inline void volk_gnsssdr_64f_accumulator_64f_u_sse3(double* result, const double* inputBuffer, unsigned int num_points)
{
    double returnValue = 0;
    const unsigned int sse_iters = num_points / 2;
    unsigned int number;
    unsigned int i;
    const double* aPtr = inputBuffer;

    __VOLK_ATTR_ALIGNED(16)
    double tempBuffer[2];
    __m128d accumulator = _mm_setzero_pd();
    __m128d aVal = _mm_setzero_pd();

    for (number = 0; number < sse_iters; number++)
        {
            aVal = _mm_loadu_pd(aPtr);
            accumulator = _mm_add_pd(accumulator, aVal);
            aPtr += 2;
        }

    _mm_storeu_pd((double*)tempBuffer, accumulator);

    for (i = 0; i < 2; ++i)
        {
            returnValue += tempBuffer[i];
        }

    for (i = 0; i < (num_points % 2); ++i)
        {
            returnValue += (*aPtr++);
        }

    *result = returnValue;
}
#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_64f_accumulator_64f_generic(double* result, const double* inputBuffer, unsigned int num_points)
{
    const double* aPtr = inputBuffer;
    double returnValue = 0;
    unsigned int number;

    for (number = 0; number < num_points; number++)
        {
            returnValue += (*aPtr++);
        }
    *result = returnValue;
}
#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_AVX
#include <immintrin.h>

static inline void volk_gnsssdr_64f_accumulator_64f_a_avx(double* result, const double* inputBuffer, unsigned int num_points)
{
    double returnValue = 0;
    const unsigned int sse_iters = num_points / 4;
    unsigned int number;
    unsigned int i;
    const double* aPtr = inputBuffer;

    __VOLK_ATTR_ALIGNED(32)
    double tempBuffer[4];
    __m256d accumulator = _mm256_setzero_pd();
    __m256d aVal = _mm256_setzero_pd();

    for (number = 0; number < sse_iters; number++)
        {
            aVal = _mm256_load_pd(aPtr);
            accumulator = _mm256_add_pd(accumulator, aVal);
            aPtr += 4;
        }

    _mm256_store_pd((double*)tempBuffer, accumulator);

    for (i = 0; i < 4; ++i)
        {
            returnValue += tempBuffer[i];
        }

    for (i = 0; i < (num_points % 4); ++i)
        {
            returnValue += (*aPtr++);
        }

    *result = returnValue;
}
#endif /* LV_HAVE_AVX */


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

static inline void volk_gnsssdr_64f_accumulator_64f_a_sse3(double* result, const double* inputBuffer, unsigned int num_points)
{
    double returnValue = 0;
    const unsigned int sse_iters = num_points / 2;
    unsigned int number;
    unsigned int i;
    const double* aPtr = inputBuffer;

    __VOLK_ATTR_ALIGNED(16)
    double tempBuffer[2];
    __m128d accumulator = _mm_setzero_pd();
    __m128d aVal = _mm_setzero_pd();

    for (number = 0; number < sse_iters; number++)
        {
            aVal = _mm_load_pd(aPtr);
            accumulator = _mm_add_pd(accumulator, aVal);
            aPtr += 2;
        }

    _mm_store_pd((double*)tempBuffer, accumulator);

    for (i = 0; i < 2; ++i)
        {
            returnValue += tempBuffer[i];
        }

    for (i = 0; i < (num_points % 2); ++i)
        {
            returnValue += (*aPtr++);
        }

    *result = returnValue;
}
#endif /* LV_HAVE_SSE3 */

#endif /* INCLUDED_volk_gnsssdr_64f_accumulator_64f_H */
