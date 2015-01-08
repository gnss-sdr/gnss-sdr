/*!
 * \file volk_gnsssdr_8i_x2_add_8i.h
 * \brief Volk protokernel: adds pairs of 8 bits (char) scalars
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that adds pairs of 8 bits (char) scalars
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

#ifndef INCLUDED_volk_gnsssdr_8i_x2_add_8i_u_H
#define INCLUDED_volk_gnsssdr_8i_x2_add_8i_u_H

#include <inttypes.h>
#include <stdio.h>

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/*!
 \brief Adds the two input vectors and store their results in the third vector
 \param cVector The vector where the results will be stored
 \param aVector One of the vectors to be added
 \param bVector One of the vectors to be added
 \param num_points The number of values in aVector and bVector to be added together and stored into cVector
 */
static inline void volk_gnsssdr_8i_x2_add_8i_u_sse2(char* cVector, const char* aVector, const char* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;

    char* cPtr = cVector;
    const char* aPtr = aVector;
    const char* bPtr=  bVector;

    __m128i aVal, bVal, cVal;

    for(unsigned int number = 0; number < sse_iters; number++)
        {
            aVal = _mm_loadu_si128((__m128i*)aPtr);
            bVal = _mm_loadu_si128((__m128i*)bPtr);

            cVal = _mm_add_epi8(aVal, bVal);

            _mm_storeu_si128((__m128i*)cPtr,cVal); // Store the results back into the C container

            aPtr += 16;
            bPtr += 16;
            cPtr += 16;
        }

    for(unsigned int i = 0; i<(num_points % 16); ++i)
        {
            *cPtr++ = (*aPtr++) + (*bPtr++);
        }
}
#endif /* LV_HAVE_SSE2 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Adds the two input vectors and store their results in the third vector
 \param cVector The vector where the results will be stored
 \param aVector One of the vectors to be added
 \param bVector One of the vectors to be added
 \param num_points The number of values in aVector and bVector to be added together and stored into cVector
 */
static inline void volk_gnsssdr_8i_x2_add_8i_generic(char* cVector, const char* aVector, const char* bVector, unsigned int num_points)
{
    char* cPtr = cVector;
    const char* aPtr = aVector;
    const char* bPtr=  bVector;
    unsigned int number = 0;

    for(number = 0; number < num_points; number++)
        {
            *cPtr++ = (*aPtr++) + (*bPtr++);
        }
}
#endif /* LV_HAVE_GENERIC */

#endif /* INCLUDED_volk_gnsssdr_8i_x2_add_8i_u_H */


#ifndef INCLUDED_volk_gnsssdr_8i_x2_add_8i_a_H
#define INCLUDED_volk_gnsssdr_8i_x2_add_8i_a_H

#include <inttypes.h>
#include <stdio.h>

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/*!
 \brief Adds the two input vectors and store their results in the third vector
 \param cVector The vector where the results will be stored
 \param aVector One of the vectors to be added
 \param bVector One of the vectors to be added
 \param num_points The number of values in aVector and bVector to be added together and stored into cVector
 */
static inline void volk_gnsssdr_8i_x2_add_8i_a_sse2(char* cVector, const char* aVector, const char* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;

    char* cPtr = cVector;
    const char* aPtr = aVector;
    const char* bPtr=  bVector;

    __m128i aVal, bVal, cVal;

    for(unsigned int number = 0; number < sse_iters; number++)
        {
            aVal = _mm_load_si128((__m128i*)aPtr);
            bVal = _mm_load_si128((__m128i*)bPtr);

            cVal = _mm_add_epi8(aVal, bVal);

            _mm_store_si128((__m128i*)cPtr,cVal); // Store the results back into the C container

            aPtr += 16;
            bPtr += 16;
            cPtr += 16;
        }

    for(unsigned int i = 0; i<(num_points % 16); ++i)
        {
            *cPtr++ = (*aPtr++) + (*bPtr++);
        }
}
#endif /* LV_HAVE_SSE2 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Adds the two input vectors and store their results in the third vector
 \param cVector The vector where the results will be stored
 \param aVector One of the vectors to be added
 \param bVector One of the vectors to be added
 \param num_points The number of values in aVector and bVector to be added together and stored into cVector
 */
static inline void volk_gnsssdr_8i_x2_add_8i_a_generic(char* cVector, const char* aVector, const char* bVector, unsigned int num_points)
{
    char* cPtr = cVector;
    const char* aPtr = aVector;
    const char* bPtr=  bVector;
    unsigned int number = 0;

    for(number = 0; number < num_points; number++)
        {
            *cPtr++ = (*aPtr++) + (*bPtr++);
        }
}
#endif /* LV_HAVE_GENERIC */

#ifdef LV_HAVE_ORC
/*!
 \brief Adds the two input vectors and store their results in the third vector
 \param cVector The vector where the results will be stored
 \param aVector One of the vectors to be added
 \param bVector One of the vectors to be added
 \param num_points The number of values in aVector and bVector to be added together and stored into cVector
 */
extern void volk_gnsssdr_8i_x2_add_8i_a_orc_impl(char* cVector, const char* aVector, const char* bVector, unsigned int num_points);
static inline void volk_gnsssdr_8i_x2_add_8i_u_orc(char* cVector, const char* aVector, const char* bVector, unsigned int num_points)
{
    volk_gnsssdr_8i_x2_add_8i_a_orc_impl(cVector, aVector, bVector, num_points);
}
#endif /* LV_HAVE_ORC */

#endif /* INCLUDED_volk_gnsssdr_8i_x2_add_8i_a_H */
