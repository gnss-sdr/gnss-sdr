/*!
 * \file volk_gnsssdr_8i_accumulator_s8i.h
 * \brief Volk protokernel: 8 bits (char) scalar accumulator
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that implements an accumulator of char values
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

#ifndef INCLUDED_volk_gnsssdr_8i_accumulator_s8i_u_H
#define INCLUDED_volk_gnsssdr_8i_accumulator_s8i_u_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <inttypes.h>
#include <stdio.h>

#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>
/*!
 \brief Accumulates the values in the input buffer
 \param result The accumulated result
 \param inputBuffer The buffer of data to be accumulated
 \param num_points The number of values in inputBuffer to be accumulated
 */
static inline void volk_gnsssdr_8i_accumulator_s8i_u_sse3(char* result, const char* inputBuffer, unsigned int num_points)
{
    char returnValue = 0;
    const unsigned int sse_iters = num_points / 16;

    const char* aPtr = inputBuffer;

    __VOLK_ATTR_ALIGNED(16) char tempBuffer[16];
    __m128i accumulator = _mm_setzero_si128();
    __m128i aVal = _mm_setzero_si128();

    for(unsigned int number = 0; number < sse_iters; number++)
        {
            aVal = _mm_lddqu_si128((__m128i*)aPtr);
            accumulator = _mm_add_epi8(accumulator, aVal);
            aPtr += 16;
        }
    _mm_storeu_si128((__m128i*)tempBuffer,accumulator);

    for(unsigned int i = 0; i<16; ++i)
        {
            returnValue += tempBuffer[i];
        }

    for(unsigned int i = 0; i<(num_points % 16); ++i)
        {
            returnValue += (*aPtr++);
        }

    *result = returnValue;
}
#endif /* LV_HAVE_SSE3 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Accumulates the values in the input buffer
 \param result The accumulated result
 \param inputBuffer The buffer of data to be accumulated
 \param num_points The number of values in inputBuffer to be accumulated
 */
static inline void volk_gnsssdr_8i_accumulator_s8i_generic(char* result, const char* inputBuffer, unsigned int num_points)
{
    const char* aPtr = inputBuffer;
    char returnValue = 0;

    for(unsigned int number = 0;number < num_points; number++)
        {
            returnValue += (*aPtr++);
        }
    *result = returnValue;
}
#endif /* LV_HAVE_GENERIC */

#endif /* INCLUDED_volk_gnsssdr_8i_accumulator_s8i_u_H */


#ifndef INCLUDED_volk_gnsssdr_8i_accumulator_s8i_a_H
#define INCLUDED_volk_gnsssdr_8i_accumulator_s8i_a_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <inttypes.h>
#include <stdio.h>

#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>
/*!
 \brief Accumulates the values in the input buffer
 \param result The accumulated result
 \param inputBuffer The buffer of data to be accumulated
 \param num_points The number of values in inputBuffer to be accumulated
 */
static inline void volk_gnsssdr_8i_accumulator_s8i_a_sse3(char* result, const char* inputBuffer, unsigned int num_points)
{
    char returnValue = 0;
    const unsigned int sse_iters = num_points / 16;

    const char* aPtr = inputBuffer;

    __VOLK_ATTR_ALIGNED(16) char tempBuffer[16];
    __m128i accumulator = _mm_setzero_si128();
    __m128i aVal = _mm_setzero_si128();

    for(unsigned int number = 0; number < sse_iters; number++)
        {
            aVal = _mm_load_si128((__m128i*)aPtr);
            accumulator = _mm_add_epi8(accumulator, aVal);
            aPtr += 16;
        }
    _mm_store_si128((__m128i*)tempBuffer,accumulator);

    for(unsigned int i = 0; i<16; ++i){
            returnValue += tempBuffer[i];
    }

    for(unsigned int i = 0; i<(num_points % 16); ++i)
        {
            returnValue += (*aPtr++);
        }

    *result = returnValue;
}
#endif /* LV_HAVE_SSE3 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Accumulates the values in the input buffer
 \param result The accumulated result
 \param inputBuffer The buffer of data to be accumulated
 \param num_points The number of values in inputBuffer to be accumulated
 */
static inline void volk_gnsssdr_8i_accumulator_s8i_a_generic(char* result, const char* inputBuffer, unsigned int num_points)
{
    const char* aPtr = inputBuffer;
    char returnValue = 0;

    for(unsigned int number = 0;number < num_points; number++)
        {
            returnValue += (*aPtr++);
        }
    *result = returnValue;
}
#endif /* LV_HAVE_GENERIC */

#ifdef LV_HAVE_ORC
/*!
 \brief Accumulates the values in the input buffer
 \param result The accumulated result
 \param inputBuffer The buffer of data to be accumulated
 \param num_points The number of values in inputBuffer to be accumulated
 */
extern void volk_gnsssdr_8i_accumulator_s8i_a_orc_impl(short* result, const char* inputBuffer, unsigned int num_points);

static inline void volk_gnsssdr_8i_accumulator_s8i_u_orc(char* result, const char* inputBuffer, unsigned int num_points)
{
    short res = 0;
    char* resc = (char*)&res;
    resc++;

    volk_gnsssdr_8i_accumulator_s8i_a_orc_impl(&res, inputBuffer, num_points);

    *result = *resc;
}
#endif /* LV_HAVE_ORC */

#endif /* INCLUDED_volk_gnsssdr_8i_accumulator_s8i_a_H */

