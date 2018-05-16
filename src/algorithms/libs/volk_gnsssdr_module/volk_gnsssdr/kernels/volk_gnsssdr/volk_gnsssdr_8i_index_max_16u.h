/*!
 * \file volk_gnsssdr_8i_index_max_16u.h
 * \brief VOLK_GNSSSDR kernel: calculates the index of the maximum value in a group of 8 bits (char) scalars.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that returns the index of the maximum value of a group of 8 bits (char) scalars
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
 * \page volk_gnsssdr_8i_index_max_16u
 *
 * \b Overview
 *
 * Returns the index of the max value in \p src0
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_8i_index_max_16u(unsigned int* target, const char* src0, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li src0: The buffer of data to be analyzed.
 * \li num_points:  The number of values in \p src0 to be analyzed.
 *
 * \b Outputs
 * \li target: The index of the maximum value in \p src0
 *
 */

#ifndef INCLUDED_volk_gnsssdr_8i_index_max_16u_H
#define INCLUDED_volk_gnsssdr_8i_index_max_16u_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_8i_index_max_16u_u_avx2(unsigned int* target, const char* src0, unsigned int num_points)
{
    if (num_points > 0)
        {
            const unsigned int avx2_iters = num_points / 32;
            unsigned int number;
            unsigned int i;
            char* basePtr = (char*)src0;
            char* inputPtr = (char*)src0;
            char max = src0[0];
            unsigned int index = 0;
            unsigned int mask;
            __VOLK_ATTR_ALIGNED(32)
            char currentValuesBuffer[32];
            __m256i maxValues, compareResults, currentValues;

            maxValues = _mm256_set1_epi8(max);

            for (number = 0; number < avx2_iters; number++)
                {
                    currentValues = _mm256_loadu_si256((__m256i*)inputPtr);
                    compareResults = _mm256_cmpgt_epi8(maxValues, currentValues);
                    mask = _mm256_movemask_epi8(compareResults);

                    if (mask != 0xFFFFFFFF)
                        {
                            _mm256_storeu_si256((__m256i*)&currentValuesBuffer, currentValues);
                            mask = ~mask;
                            i = 0;
                            while (mask > 0)
                                {
                                    if ((mask & 1) == 1)
                                        {
                                            if (currentValuesBuffer[i] > max)
                                                {
                                                    index = inputPtr - basePtr + i;
                                                    max = currentValuesBuffer[i];
                                                }
                                        }
                                    i++;
                                    mask >>= 1;
                                }
                            maxValues = _mm256_set1_epi8(max);
                        }
                    inputPtr += 32;
                }

            for (i = 0; i < (num_points % 32); ++i)
                {
                    if (src0[i] > max)
                        {
                            index = i;
                            max = src0[i];
                        }
                }
            target[0] = index;
        }
}

#endif /*LV_HAVE_AVX2*/


#ifdef LV_HAVE_AVX
#include <immintrin.h>

static inline void volk_gnsssdr_8i_index_max_16u_u_avx(unsigned int* target, const char* src0, unsigned int num_points)
{
    if (num_points > 0)
        {
            const unsigned int sse_iters = num_points / 32;
            unsigned int number;
            unsigned int i;
            char* basePtr = (char*)src0;
            char* inputPtr = (char*)src0;
            char max = src0[0];
            unsigned int index = 0;
            __VOLK_ATTR_ALIGNED(32)
            char currentValuesBuffer[32];
            __m256i ones, compareResults, currentValues;
            __m128i compareResultslo, compareResultshi, maxValues, lo, hi;

            ones = _mm256_set1_epi8(0xFF);
            maxValues = _mm_set1_epi8(max);

            for (number = 0; number < sse_iters; number++)
                {
                    currentValues = _mm256_lddqu_si256((__m256i*)inputPtr);

                    lo = _mm256_castsi256_si128(currentValues);
                    hi = _mm256_extractf128_si256(currentValues, 1);

                    compareResultslo = _mm_cmpgt_epi8(maxValues, lo);
                    compareResultshi = _mm_cmpgt_epi8(maxValues, hi);

                    //compareResults = _mm256_set_m128i(compareResultshi , compareResultslo); //not defined in some versions of immintrin.h
                    compareResults = _mm256_insertf128_si256(_mm256_castsi128_si256(compareResultslo), (compareResultshi), 1);

                    if (!_mm256_testc_si256(compareResults, ones))
                        {
                            _mm256_storeu_si256((__m256i*)&currentValuesBuffer, currentValues);

                            for (i = 0; i < 32; i++)
                                {
                                    if (currentValuesBuffer[i] > max)
                                        {
                                            index = inputPtr - basePtr + i;
                                            max = currentValuesBuffer[i];
                                        }
                                }
                            maxValues = _mm_set1_epi8(max);
                        }

                    inputPtr += 32;
                }

            for (i = 0; i < (num_points % 32); ++i)
                {
                    if (src0[i] > max)
                        {
                            index = i;
                            max = src0[i];
                        }
                }
            target[0] = index;
        }
}

#endif /*LV_HAVE_AVX*/


#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>

static inline void volk_gnsssdr_8i_index_max_16u_u_sse4_1(unsigned int* target, const char* src0, unsigned int num_points)
{
    if (num_points > 0)
        {
            const unsigned int sse_iters = num_points / 16;
            unsigned int number;
            unsigned int i = 0;
            char* basePtr = (char*)src0;
            char* inputPtr = (char*)src0;
            char max = src0[0];
            unsigned int index = 0;
            __VOLK_ATTR_ALIGNED(16)
            char currentValuesBuffer[16];
            __m128i maxValues, compareResults, currentValues;

            maxValues = _mm_set1_epi8(max);

            for (number = 0; number < sse_iters; number++)
                {
                    currentValues = _mm_lddqu_si128((__m128i*)inputPtr);

                    compareResults = _mm_cmpgt_epi8(maxValues, currentValues);

                    if (!_mm_test_all_ones(compareResults))
                        {
                            _mm_storeu_si128((__m128i*)&currentValuesBuffer, currentValues);

                            for (i = 0; i < 16; i++)
                                {
                                    if (currentValuesBuffer[i] > max)
                                        {
                                            index = inputPtr - basePtr + i;
                                            max = currentValuesBuffer[i];
                                        }
                                }
                            maxValues = _mm_set1_epi8(max);
                        }

                    inputPtr += 16;
                }

            for (i = 0; i < (num_points % 16); ++i)
                {
                    if (src0[i] > max)
                        {
                            index = i;
                            max = src0[i];
                        }
                }
            target[0] = index;
        }
}

#endif /*LV_HAVE_SSE4_1*/


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_8i_index_max_16u_u_sse2(unsigned int* target, const char* src0, unsigned int num_points)
{
    if (num_points > 0)
        {
            const unsigned int sse_iters = num_points / 16;
            unsigned int number;
            unsigned int i;
            char* basePtr = (char*)src0;
            char* inputPtr = (char*)src0;
            char max = src0[0];
            unsigned int index = 0;
            unsigned short mask;
            __VOLK_ATTR_ALIGNED(16)
            char currentValuesBuffer[16];
            __m128i maxValues, compareResults, currentValues;

            maxValues = _mm_set1_epi8(max);

            for (number = 0; number < sse_iters; number++)
                {
                    currentValues = _mm_loadu_si128((__m128i*)inputPtr);
                    compareResults = _mm_cmpgt_epi8(maxValues, currentValues);
                    mask = _mm_movemask_epi8(compareResults);

                    if (mask != 0xFFFF)
                        {
                            _mm_storeu_si128((__m128i*)&currentValuesBuffer, currentValues);
                            mask = ~mask;
                            i = 0;
                            while (mask > 0)
                                {
                                    if ((mask & 1) == 1)
                                        {
                                            if (currentValuesBuffer[i] > max)
                                                {
                                                    index = inputPtr - basePtr + i;
                                                    max = currentValuesBuffer[i];
                                                }
                                        }
                                    i++;
                                    mask >>= 1;
                                }
                            maxValues = _mm_set1_epi8(max);
                        }
                    inputPtr += 16;
                }

            for (i = 0; i < (num_points % 16); ++i)
                {
                    if (src0[i] > max)
                        {
                            index = i;
                            max = src0[i];
                        }
                }
            target[0] = index;
        }
}

#endif /*LV_HAVE_SSE2*/


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_8i_index_max_16u_generic(unsigned int* target, const char* src0, unsigned int num_points)
{
    if (num_points > 0)
        {
            char max = src0[0];
            unsigned int index = 0;
            unsigned int i;
            for (i = 1; i < num_points; ++i)
                {
                    if (src0[i] > max)
                        {
                            index = i;
                            max = src0[i];
                        }
                }
            target[0] = index;
        }
}

#endif /*LV_HAVE_GENERIC*/


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_8i_index_max_16u_a_avx2(unsigned int* target, const char* src0, unsigned int num_points)
{
    if (num_points > 0)
        {
            const unsigned int avx2_iters = num_points / 32;
            unsigned int number;
            unsigned int i;
            char* basePtr = (char*)src0;
            char* inputPtr = (char*)src0;
            char max = src0[0];
            unsigned int index = 0;
            unsigned int mask;
            __VOLK_ATTR_ALIGNED(32)
            char currentValuesBuffer[32];
            __m256i maxValues, compareResults, currentValues;

            maxValues = _mm256_set1_epi8(max);

            for (number = 0; number < avx2_iters; number++)
                {
                    currentValues = _mm256_load_si256((__m256i*)inputPtr);
                    compareResults = _mm256_cmpgt_epi8(maxValues, currentValues);
                    mask = _mm256_movemask_epi8(compareResults);

                    if (mask != 0xFFFFFFFF)
                        {
                            _mm256_store_si256((__m256i*)&currentValuesBuffer, currentValues);
                            mask = ~mask;
                            i = 0;
                            while (mask > 0)
                                {
                                    if ((mask & 1) == 1)
                                        {
                                            if (currentValuesBuffer[i] > max)
                                                {
                                                    index = inputPtr - basePtr + i;
                                                    max = currentValuesBuffer[i];
                                                }
                                        }
                                    i++;
                                    mask >>= 1;
                                }
                            maxValues = _mm256_set1_epi8(max);
                        }
                    inputPtr += 32;
                }

            for (i = 0; i < (num_points % 32); ++i)
                {
                    if (src0[i] > max)
                        {
                            index = i;
                            max = src0[i];
                        }
                }
            target[0] = index;
        }
}

#endif /*LV_HAVE_AVX2*/


#ifdef LV_HAVE_AVX
#include <immintrin.h>

static inline void volk_gnsssdr_8i_index_max_16u_a_avx(unsigned int* target, const char* src0, unsigned int num_points)
{
    if (num_points > 0)
        {
            const unsigned int sse_iters = num_points / 32;
            unsigned int number;
            unsigned int i;
            char* basePtr = (char*)src0;
            char* inputPtr = (char*)src0;
            char max = src0[0];
            unsigned int index = 0;
            __VOLK_ATTR_ALIGNED(32)
            char currentValuesBuffer[32];
            __m256i ones, compareResults, currentValues;
            __m128i compareResultslo, compareResultshi, maxValues, lo, hi;

            ones = _mm256_set1_epi8(0xFF);
            maxValues = _mm_set1_epi8(max);

            for (number = 0; number < sse_iters; number++)
                {
                    currentValues = _mm256_load_si256((__m256i*)inputPtr);

                    lo = _mm256_castsi256_si128(currentValues);
                    hi = _mm256_extractf128_si256(currentValues, 1);

                    compareResultslo = _mm_cmpgt_epi8(maxValues, lo);
                    compareResultshi = _mm_cmpgt_epi8(maxValues, hi);

                    //compareResults = _mm256_set_m128i(compareResultshi , compareResultslo); //not defined in some versions of immintrin.h
                    compareResults = _mm256_insertf128_si256(_mm256_castsi128_si256(compareResultslo), (compareResultshi), 1);

                    if (!_mm256_testc_si256(compareResults, ones))
                        {
                            _mm256_store_si256((__m256i*)&currentValuesBuffer, currentValues);

                            for (i = 0; i < 32; i++)
                                {
                                    if (currentValuesBuffer[i] > max)
                                        {
                                            index = inputPtr - basePtr + i;
                                            max = currentValuesBuffer[i];
                                        }
                                }
                            maxValues = _mm_set1_epi8(max);
                        }

                    inputPtr += 32;
                }

            for (i = 0; i < (num_points % 32); ++i)
                {
                    if (src0[i] > max)
                        {
                            index = i;
                            max = src0[i];
                        }
                }
            target[0] = index;
        }
}

#endif /*LV_HAVE_AVX*/


#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>

static inline void volk_gnsssdr_8i_index_max_16u_a_sse4_1(unsigned int* target, const char* src0, unsigned int num_points)
{
    if (num_points > 0)
        {
            const unsigned int sse_iters = num_points / 16;
            unsigned int number;
            unsigned int i;
            char* basePtr = (char*)src0;
            char* inputPtr = (char*)src0;
            char max = src0[0];
            unsigned int index = 0;
            __VOLK_ATTR_ALIGNED(16)
            char currentValuesBuffer[16];
            __m128i maxValues, compareResults, currentValues;

            maxValues = _mm_set1_epi8(max);

            for (number = 0; number < sse_iters; number++)
                {
                    currentValues = _mm_load_si128((__m128i*)inputPtr);

                    compareResults = _mm_cmpgt_epi8(maxValues, currentValues);

                    if (!_mm_test_all_ones(compareResults))
                        {
                            _mm_store_si128((__m128i*)&currentValuesBuffer, currentValues);

                            for (i = 0; i < 16; i++)
                                {
                                    if (currentValuesBuffer[i] > max)
                                        {
                                            index = inputPtr - basePtr + i;
                                            max = currentValuesBuffer[i];
                                        }
                                }
                            maxValues = _mm_set1_epi8(max);
                        }

                    inputPtr += 16;
                }

            for (i = 0; i < (num_points % 16); ++i)
                {
                    if (src0[i] > max)
                        {
                            index = i;
                            max = src0[i];
                        }
                }
            target[0] = index;
        }
}

#endif /*LV_HAVE_SSE4_1*/


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_8i_index_max_16u_a_sse2(unsigned int* target, const char* src0, unsigned int num_points)
{
    if (num_points > 0)
        {
            const unsigned int sse_iters = num_points / 16;
            unsigned int number;
            unsigned int i;
            char* basePtr = (char*)src0;
            char* inputPtr = (char*)src0;
            char max = src0[0];
            unsigned int index = 0;
            unsigned short mask;
            __VOLK_ATTR_ALIGNED(16)
            char currentValuesBuffer[16];
            __m128i maxValues, compareResults, currentValues;

            maxValues = _mm_set1_epi8(max);

            for (number = 0; number < sse_iters; number++)
                {
                    currentValues = _mm_load_si128((__m128i*)inputPtr);
                    compareResults = _mm_cmpgt_epi8(maxValues, currentValues);
                    mask = _mm_movemask_epi8(compareResults);

                    if (mask != 0xFFFF)
                        {
                            _mm_store_si128((__m128i*)&currentValuesBuffer, currentValues);
                            mask = ~mask;
                            i = 0;
                            while (mask > 0)
                                {
                                    if ((mask & 1) == 1)
                                        {
                                            if (currentValuesBuffer[i] > max)
                                                {
                                                    index = inputPtr - basePtr + i;
                                                    max = currentValuesBuffer[i];
                                                }
                                        }
                                    i++;
                                    mask >>= 1;
                                }
                            maxValues = _mm_set1_epi8(max);
                        }
                    inputPtr += 16;
                }

            for (i = 0; i < (num_points % 16); ++i)
                {
                    if (src0[i] > max)
                        {
                            index = i;
                            max = src0[i];
                        }
                }
            target[0] = index;
        }
}

#endif /*LV_HAVE_SSE2*/


#endif /*INCLUDED_volk_gnsssdr_8i_index_max_16u_H*/
