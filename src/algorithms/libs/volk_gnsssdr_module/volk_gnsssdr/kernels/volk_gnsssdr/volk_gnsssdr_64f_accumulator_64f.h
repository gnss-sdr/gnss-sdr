/*!
 * \file volk_gnsssdr_64f_accumulator_64f.h
 * \brief VOLK_GNSSSDR kernel: 64 bits (double) scalar accumulator.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that implements an accumulator of double (64-bit float) values.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
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
