/*!
 * \file volk_gnsssdr_8i_max_s8i.h
 * \brief Volk protokernel: calculates the maximum value in a group of 8 bits (char) scalars
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that returns the maximum value of a group of 8 bits (char) scalars
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

#ifndef INCLUDED_volk_gnsssdr_8i_max_s8i_u_H
#define INCLUDED_volk_gnsssdr_8i_max_s8i_u_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <inttypes.h>
#include <stdio.h>

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
/*!
 \brief Returns the max value in src0
 \param target The max value in src0
 \param src0 The buffer of data to be analysed
 \param num_points The number of values in src0 to be analysed
 */
static inline void volk_gnsssdr_8i_max_s8i_u_sse4_1(char* target, const char* src0, unsigned int num_points)
{
    if(num_points > 0)
        {
            const unsigned int sse_iters = num_points / 16;

            char* inputPtr = (char*)src0;
            char max = src0[0];
            __VOLK_ATTR_ALIGNED(16) char maxValuesBuffer[16];
            __m128i maxValues, compareResults, currentValues;

            maxValues = _mm_set1_epi8(max);

            for(unsigned int number = 0; number < sse_iters; number++)
                {
                    currentValues  = _mm_loadu_si128((__m128i*)inputPtr);
                    compareResults = _mm_cmpgt_epi8(maxValues, currentValues);
                    maxValues = _mm_blendv_epi8(currentValues, maxValues, compareResults);
                    inputPtr += 16;
                }

            _mm_storeu_si128((__m128i*)maxValuesBuffer, maxValues);

            for(unsigned int i = 0; i<16; ++i)
                {
                    if(maxValuesBuffer[i] > max)
                        {
                            max = maxValuesBuffer[i];
                        }
                }

            for(unsigned int i = 0; i<(num_points % 16); ++i)
                {
                    if(src0[i] > max)
                        {
                            max = src0[i];
                        }
                }
            target[0] = max;
        }
}

#endif /*LV_HAVE_SSE4_1*/

#ifdef LV_HAVE_SSE2
#include<emmintrin.h>
/*!
 \brief Returns the max value in src0
 \param target The max value in src0
 \param src0 The buffer of data to be analysed
 \param num_points The number of values in src0 to be analysed
 */
static inline void volk_gnsssdr_8i_max_s8i_u_sse2(char* target, const char* src0, unsigned int num_points)
{
    if(num_points > 0)
        {
            const unsigned int sse_iters = num_points / 16;

            char* inputPtr = (char*)src0;
            char max = src0[0];
            unsigned short mask;
            __VOLK_ATTR_ALIGNED(16) char currentValuesBuffer[16];
            __m128i maxValues, compareResults, currentValues;

            maxValues = _mm_set1_epi8(max);

            for(unsigned int number = 0; number < sse_iters; number++)
                {
                    currentValues  = _mm_loadu_si128((__m128i*)inputPtr);
                    compareResults = _mm_cmpgt_epi8(maxValues, currentValues);
                    mask = _mm_movemask_epi8(compareResults);

                    if (mask != 0xFFFF)
                        {
                            _mm_storeu_si128((__m128i*)&currentValuesBuffer, currentValues);
                            mask = ~mask;
                            int i = 0;
                            while (mask > 0)
                                {
                                    if ((mask & 1) == 1)
                                        {
                                            if(currentValuesBuffer[i] > max)
                                                {
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

            for(unsigned int i = 0; i<(num_points % 16); ++i)
                {
                    if(src0[i] > max)
                        {
                            max = src0[i];
                        }
                }
            target[0] = max;
        }
}

#endif /*LV_HAVE_SSE2*/

#ifdef LV_HAVE_GENERIC
/*!
 \brief Returns the max value in src0
 \param target The max value in src0
 \param src0 The buffer of data to be analysed
 \param num_points The number of values in src0 to be analysed
 */
static inline void volk_gnsssdr_8i_max_s8i_generic(char* target, const char* src0, unsigned int num_points)
{
    if(num_points > 0)
        {
            char max = src0[0];

            for(unsigned int i = 1; i < num_points; ++i)
                {
                    if(src0[i] > max)
                        {
                            max = src0[i];
                        }
                }
            target[0] = max;
        }
}

#endif /*LV_HAVE_GENERIC*/

#endif /*INCLUDED_volk_gnsssdr_8i_max_s8i_u_H*/


#ifndef INCLUDED_volk_gnsssdr_8i_max_s8i_a_H
#define INCLUDED_volk_gnsssdr_8i_max_s8i_a_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <inttypes.h>
#include <stdio.h>

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
/*!
 \brief Returns the max value in src0
 \param target The max value in src0
 \param src0 The buffer of data to be analysed
 \param num_points The number of values in src0 to be analysed
 */
static inline void volk_gnsssdr_8i_max_s8i_a_sse4_1(char* target, const char* src0, unsigned int num_points)
{
    if(num_points > 0)
        {
            const unsigned int sse_iters = num_points / 16;

            char* inputPtr = (char*)src0;
            char max = src0[0];
            __VOLK_ATTR_ALIGNED(16) char maxValuesBuffer[16];
            __m128i maxValues, compareResults, currentValues;

            maxValues = _mm_set1_epi8(max);

            for(unsigned int number = 0; number < sse_iters; number++)
                {
                    currentValues  = _mm_load_si128((__m128i*)inputPtr);
                    compareResults = _mm_cmpgt_epi8(maxValues, currentValues);
                    maxValues = _mm_blendv_epi8(currentValues, maxValues, compareResults);
                    inputPtr += 16;
                }

            _mm_store_si128((__m128i*)maxValuesBuffer, maxValues);

            for(unsigned int i = 0; i<16; ++i)
                {
                    if(maxValuesBuffer[i] > max)
                        {
                            max = maxValuesBuffer[i];
                        }
                }

            for(unsigned int i = 0; i<(num_points % 16); ++i)
                {
                    if(src0[i] > max)
                        {
                            max = src0[i];
                        }
                }
            target[0] = max;
        }
}

#endif /*LV_HAVE_SSE4_1*/

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/*!
 \brief Returns the max value in src0
 \param target The max value in src0
 \param src0 The buffer of data to be analysed
 \param num_points The number of values in src0 to be analysed
 */
static inline void volk_gnsssdr_8i_max_s8i_a_sse2(char* target, const char* src0, unsigned int num_points)
{
    if(num_points > 0)
        {
            const unsigned int sse_iters = num_points / 16;

            char* inputPtr = (char*)src0;
            char max = src0[0];
            unsigned short mask;
            __VOLK_ATTR_ALIGNED(16) char currentValuesBuffer[16];
            __m128i maxValues, compareResults, currentValues;

            maxValues = _mm_set1_epi8(max);

            for(unsigned int number = 0; number < sse_iters; number++)
                {
                    currentValues  = _mm_load_si128((__m128i*)inputPtr);
                    compareResults = _mm_cmpgt_epi8(maxValues, currentValues);
                    mask = _mm_movemask_epi8(compareResults);

                    if (mask != 0xFFFF)
                        {
                            _mm_store_si128((__m128i*)&currentValuesBuffer, currentValues);
                            mask = ~mask;
                            int i = 0;
                            while (mask > 0)
                                {
                                    if ((mask & 1) == 1)
                                        {
                                            if(currentValuesBuffer[i] > max)
                                                {
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

            for(unsigned int i = 0; i<(num_points % 16); ++i)
                {
                    if(src0[i] > max)
                        {
                            max = src0[i];
                        }
                }
            target[0] = max;
        }
}

#endif /*LV_HAVE_SSE2*/

#ifdef LV_HAVE_GENERIC
/*!
 \brief Returns the max value in src0
 \param target The max value in src0
 \param src0 The buffer of data to be analysed
 \param num_points The number of values in src0 to be analysed
 */
static inline void volk_gnsssdr_8i_max_s8i_a_generic(char* target, const char* src0, unsigned int num_points)
{
    if(num_points > 0)
        {
            if(num_points > 0)
                {
                    char max = src0[0];
                    for(unsigned int i = 1; i < num_points; ++i)
                        {
                            if(src0[i] > max)
                                {
                                    max = src0[i];
                                }
                        }
                    target[0] = max;
                }
        }
}

#endif /*LV_HAVE_GENERIC*/

#endif /*INCLUDED_volk_gnsssdr_8i_max_s8i_a_H*/
