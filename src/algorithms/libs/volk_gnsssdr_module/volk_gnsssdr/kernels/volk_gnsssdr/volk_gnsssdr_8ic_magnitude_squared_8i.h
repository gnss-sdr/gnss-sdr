/*!
 * \file volk_gnsssdr_8ic_magnitude_squared_8i.h
 * \brief VOLK_GNSSSDR kernel: calculates the magnitude squared of a 16 bits vector.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that calculates the magnitude squared of a
 * 16 bits vector (8 bits the real part and 8 bits the imaginary part)
 * result = (real*real) + (imag*imag)
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
 * \page volk_gnsssdr_8ic_magnitude_squared_8i
 *
 * \b Overview
 *
 * Calculates the magnitude squared of the complex data items in \p complexVector and stores the results in \p magnitudeVector
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_8ic_magnitude_squared_8i(char* magnitudeVector, const lv_8sc_t* complexVector, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li complexVector: The vector containing the complex input values
 * \li num_points: The number of complex data points.
 *
 * \b Outputs
 * \li magnitudeVector: The vector containing the real output values
 *
 */

#ifndef INCLUDED_volk_gnsssdr_8ic_magnitude_squared_8i_H
#define INCLUDED_volk_gnsssdr_8ic_magnitude_squared_8i_H


#ifdef LV_HAVE_SSSE3
#include <tmmintrin.h>

static inline void volk_gnsssdr_8ic_magnitude_squared_8i_u_sse3(char* magnitudeVector, const lv_8sc_t* complexVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;
    unsigned int number;
    unsigned int i;
    const char* complexVectorPtr = (char*)complexVector;
    char* magnitudeVectorPtr = magnitudeVector;

    __m128i zero, result8;
    __m128i avector, avectorhi, avectorlo, avectorlomult, avectorhimult, aadded, maska;
    __m128i bvector, bvectorhi, bvectorlo, bvectorlomult, bvectorhimult, badded, maskb;

    zero = _mm_setzero_si128();
    maska = _mm_set_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 14, 12, 10, 8, 6, 4, 2, 0);
    maskb = _mm_set_epi8(14, 12, 10, 8, 6, 4, 2, 0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);

    for (number = 0; number < sse_iters; number++)
        {
            avector = _mm_lddqu_si128((__m128i*)complexVectorPtr);
            avectorlo = _mm_unpacklo_epi8(avector, zero);
            avectorhi = _mm_unpackhi_epi8(avector, zero);
            avectorlomult = _mm_mullo_epi16(avectorlo, avectorlo);
            avectorhimult = _mm_mullo_epi16(avectorhi, avectorhi);
            aadded = _mm_hadd_epi16(avectorlomult, avectorhimult);

            complexVectorPtr += 16;

            bvector = _mm_lddqu_si128((__m128i*)complexVectorPtr);
            bvectorlo = _mm_unpacklo_epi8(bvector, zero);
            bvectorhi = _mm_unpackhi_epi8(bvector, zero);
            bvectorlomult = _mm_mullo_epi16(bvectorlo, bvectorlo);
            bvectorhimult = _mm_mullo_epi16(bvectorhi, bvectorhi);
            badded = _mm_hadd_epi16(bvectorlomult, bvectorhimult);

            complexVectorPtr += 16;

            result8 = _mm_or_si128(_mm_shuffle_epi8(aadded, maska), _mm_shuffle_epi8(badded, maskb));

            _mm_storeu_si128((__m128i*)magnitudeVectorPtr, result8);

            magnitudeVectorPtr += 16;
        }

    for (i = sse_iters * 16; i < num_points; ++i)
        {
            const char valReal = *complexVectorPtr++;
            const char valImag = *complexVectorPtr++;
            *magnitudeVectorPtr++ = (valReal * valReal) + (valImag * valImag);
        }
}
#endif /* LV_HAVE_SSSE3 */


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_8ic_magnitude_squared_8i_generic(char* magnitudeVector, const lv_8sc_t* complexVector, unsigned int num_points)
{
    const char* complexVectorPtr = (char*)complexVector;
    char* magnitudeVectorPtr = magnitudeVector;
    unsigned int number;
    for (number = 0; number < num_points; number++)
        {
            const char real = *complexVectorPtr++;
            const char imag = *complexVectorPtr++;
            *magnitudeVectorPtr++ = (real * real) + (imag * imag);
        }
}
#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSSE3
#include <tmmintrin.h>

static inline void volk_gnsssdr_8ic_magnitude_squared_8i_a_sse3(char* magnitudeVector, const lv_8sc_t* complexVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;

    const char* complexVectorPtr = (char*)complexVector;
    char* magnitudeVectorPtr = magnitudeVector;
    unsigned int number;
    unsigned int i;

    __m128i zero, result8;
    __m128i avector, avectorhi, avectorlo, avectorlomult, avectorhimult, aadded, maska;
    __m128i bvector, bvectorhi, bvectorlo, bvectorlomult, bvectorhimult, badded, maskb;

    zero = _mm_setzero_si128();
    maska = _mm_set_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 14, 12, 10, 8, 6, 4, 2, 0);
    maskb = _mm_set_epi8(14, 12, 10, 8, 6, 4, 2, 0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);

    for (number = 0; number < sse_iters; number++)
        {
            avector = _mm_load_si128((__m128i*)complexVectorPtr);
            avectorlo = _mm_unpacklo_epi8(avector, zero);
            avectorhi = _mm_unpackhi_epi8(avector, zero);
            avectorlomult = _mm_mullo_epi16(avectorlo, avectorlo);
            avectorhimult = _mm_mullo_epi16(avectorhi, avectorhi);
            aadded = _mm_hadd_epi16(avectorlomult, avectorhimult);

            complexVectorPtr += 16;

            bvector = _mm_load_si128((__m128i*)complexVectorPtr);
            bvectorlo = _mm_unpacklo_epi8(bvector, zero);
            bvectorhi = _mm_unpackhi_epi8(bvector, zero);
            bvectorlomult = _mm_mullo_epi16(bvectorlo, bvectorlo);
            bvectorhimult = _mm_mullo_epi16(bvectorhi, bvectorhi);
            badded = _mm_hadd_epi16(bvectorlomult, bvectorhimult);

            complexVectorPtr += 16;

            result8 = _mm_or_si128(_mm_shuffle_epi8(aadded, maska), _mm_shuffle_epi8(badded, maskb));

            _mm_store_si128((__m128i*)magnitudeVectorPtr, result8);

            magnitudeVectorPtr += 16;
        }

    for (i = sse_iters * 16; i < num_points; ++i)
        {
            const char valReal = *complexVectorPtr++;
            const char valImag = *complexVectorPtr++;
            *magnitudeVectorPtr++ = (valReal * valReal) + (valImag * valImag);
        }
}
#endif /* LV_HAVE_SSSE3 */


#ifdef LV_HAVE_ORC

extern void volk_gnsssdr_8ic_magnitude_squared_8i_a_orc_impl(char* magnitudeVector, const lv_8sc_t* complexVector, unsigned int num_points);
static inline void volk_gnsssdr_8ic_magnitude_squared_8i_u_orc(char* magnitudeVector, const lv_8sc_t* complexVector, unsigned int num_points)
{
    volk_gnsssdr_8ic_magnitude_squared_8i_a_orc_impl(magnitudeVector, complexVector, num_points);
}
#endif /* LV_HAVE_ORC */

#endif /* INCLUDED_volk_gnsssdr_32fc_magnitude_32f_H */
