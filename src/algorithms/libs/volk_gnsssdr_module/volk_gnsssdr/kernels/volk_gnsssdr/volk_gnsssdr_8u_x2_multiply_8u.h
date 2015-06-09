/*!
 * \file volk_gnsssdr_8u_x2_multiply_8u.h
 * \brief Volk protokernel: multiplies unsigned char values
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that multiplies unsigned char values (8 bits data)
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

#ifndef INCLUDED_volk_gnsssdr_8u_x2_multiply_8u_u_H
#define INCLUDED_volk_gnsssdr_8u_x2_multiply_8u_u_H

#include <inttypes.h>
#include <stdio.h>

#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>
/*!
 \brief Multiplies the two input unsigned char values and stores their results in the third unsigned char
 \param cChar The unsigned char where the results will be stored
 \param aChar One of the unsigned char to be multiplied
 \param bChar One of the unsigned char to be multiplied
 \param num_points The number of unsigned char values in aChar and bChar to be multiplied together and stored into cChar
 */
static inline void volk_gnsssdr_8u_x2_multiply_8u_u_sse3(unsigned char* cChar, const unsigned char* aChar, const unsigned char* bChar, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;

    __m128i x, y, x1, x2, y1, y2, mult1, x1_mult_y1, x2_mult_y2, tmp, tmp1, tmp2, totalc;
    unsigned char* c = cChar;
    const unsigned char* a = aChar;
    const unsigned char* b = bChar;

    for(unsigned int number = 0;number < sse_iters; number++)
        {
            x = _mm_lddqu_si128((__m128i*)a);
            y = _mm_lddqu_si128((__m128i*)b);

            mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);
            x1 = _mm_srli_si128 (x, 1);
            x1 = _mm_and_si128 (x1, mult1);
            x2 = _mm_and_si128 (x, mult1);

            y1 = _mm_srli_si128 (y, 1);
            y1 = _mm_and_si128 (y1, mult1);
            y2 = _mm_and_si128 (y, mult1);

            x1_mult_y1 = _mm_mullo_epi16 (x1, y1);
            x2_mult_y2 = _mm_mullo_epi16 (x2, y2);

            tmp = _mm_and_si128 (x1_mult_y1, mult1);
            tmp1 = _mm_slli_si128 (tmp, 1);
            tmp2 = _mm_and_si128 (x2_mult_y2, mult1);
            totalc = _mm_or_si128 (tmp1, tmp2);

            _mm_storeu_si128((__m128i*)c, totalc);

            a += 16;
            b += 16;
            c += 16;
        }

    for (unsigned int i = 0; i<(num_points % 16); ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE3 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Multiplies the two input unsigned char values and stores their results in the third unisgned char
 \param cChar The unsigned char where the results will be stored
 \param aChar One of the unsigned char to be multiplied
 \param bChar One of the unsigned char to be multiplied
 \param num_points The number of unsigned char values in aChar and bChar to be multiplied together and stored into cChar
 */
static inline void volk_gnsssdr_8u_x2_multiply_8u_generic(unsigned char* cChar, const unsigned char* aChar, const unsigned char* bChar, unsigned int num_points)
{
    unsigned char* cPtr = cChar;
    const unsigned char* aPtr = aChar;
    const unsigned char* bPtr = bChar;

    for(unsigned int number = 0; number < num_points; number++)
        {
            *cPtr++ = (*aPtr++) * (*bPtr++);
        }
}
#endif /* LV_HAVE_GENERIC */

#endif /* INCLUDED_volk_gnsssdr_8u_x2_multiply_8u_u_H */


#ifndef INCLUDED_volk_gnsssdr_8u_x2_multiply_8u_a_H
#define INCLUDED_volk_gnsssdr_8u_x2_multiply_8u_a_H

#include <inttypes.h>
#include <stdio.h>

#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

/*!
 \brief Multiplies the two input unsigned char values and stores their results in the third unisgned char
 \param cChar The unsigned char where the results will be stored
 \param aChar One of the unsigned char to be multiplied
 \param bChar One of the unsigned char to be multiplied
 \param num_points The number of unsigned char values in aChar and bChar to be multiplied together and stored into cChar
 */
static inline void volk_gnsssdr_8u_x2_multiply_8u_a_sse3(unsigned char* cChar, const unsigned char* aChar, const unsigned char* bChar, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;

    __m128i x, y, x1, x2, y1, y2, mult1, x1_mult_y1, x2_mult_y2, tmp, tmp1, tmp2, totalc;
    unsigned char* c = cChar;
    const unsigned char* a = aChar;
    const unsigned char* b = bChar;

    for(unsigned int number = 0;number < sse_iters; number++)
        {
            x = _mm_load_si128((__m128i*)a);
            y = _mm_load_si128((__m128i*)b);

            mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);
            x1 = _mm_srli_si128 (x, 1);
            x1 = _mm_and_si128 (x1, mult1);
            x2 = _mm_and_si128 (x, mult1);

            y1 = _mm_srli_si128 (y, 1);
            y1 = _mm_and_si128 (y1, mult1);
            y2 = _mm_and_si128 (y, mult1);

            x1_mult_y1 = _mm_mullo_epi16 (x1, y1);
            x2_mult_y2 = _mm_mullo_epi16 (x2, y2);

            tmp = _mm_and_si128 (x1_mult_y1, mult1);
            tmp1 = _mm_slli_si128 (tmp, 1);
            tmp2 = _mm_and_si128 (x2_mult_y2, mult1);
            totalc = _mm_or_si128 (tmp1, tmp2);

            _mm_store_si128((__m128i*)c, totalc);

            a += 16;
            b += 16;
            c += 16;
        }

    for (unsigned int i = 0; i<(num_points % 16); ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Multiplies the two input unsigned char values and stores their results in the third unisgned char
 \param cChar The unsigned char where the results will be stored
 \param aChar One of the unsigned char to be multiplied
 \param bChar One of the unsigned char to be multiplied
 \param num_points The number of unsigned char values in aChar and bChar to be multiplied together and stored into cChar
 */
static inline void volk_gnsssdr_8u_x2_multiply_8u_a_generic(unsigned char* cChar, const unsigned char* aChar, const unsigned char* bChar, unsigned int num_points)
{
    unsigned char* cPtr = cChar;
    const unsigned char* aPtr = aChar;
    const unsigned char* bPtr = bChar;

    for(unsigned int number = 0; number < num_points; number++)
        {
            *cPtr++ = (*aPtr++) * (*bPtr++);
        }
}
#endif /* LV_HAVE_GENERIC */

#ifdef LV_HAVE_ORC
/*!
 \brief Multiplies the two input unsigned char values and stores their results in the third unisgned char
 \param cChar The unsigned char where the results will be stored
 \param aChar One of the unsigned char to be multiplied
 \param bChar One of the unsigned char to be multiplied
 \param num_points The number of unsigned char values in aChar and bChar to be multiplied together and stored into cChar
 */
extern void volk_gnsssdr_8u_x2_multiply_8u_a_orc_impl(unsigned char* cVector, const unsigned char* aVector, const unsigned char* bVector, unsigned int num_points);
static inline void volk_gnsssdr_8u_x2_multiply_8u_u_orc(unsigned char* cVector, const unsigned char* aVector, const unsigned char* bVector, unsigned int num_points)
{
    volk_gnsssdr_8u_x2_multiply_8u_a_orc_impl(cVector, aVector, bVector, num_points);
}
#endif /* LV_HAVE_ORC */

#endif /* INCLUDED_volk_gnsssdr_8u_x2_multiply_8u_a_H */
