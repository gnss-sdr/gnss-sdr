/*!
 * \file volk_gnsssdr_16ic_conjugate_16ic.h
 * \brief VOLK_GNSSSDR kernel: returns the conjugate of a 16 bits complex vector.
 * \authors <ul>
 *          <li>  Carles Fernandez Prades 2017 cfernandez at cttc dot cat
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that calculates the conjugate of a
 * 16 bits complex vector (16 bits the real part and 16 bits the imaginary part)
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
 * \page volk_gnsssdr_16ic_conjugate_16ic
 *
 * \b Overview
 *
 * Takes the conjugate of a complex signed 16-bit integer vector.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_16ic_conjugate_16ic(lv_16sc_t* cVector, const lv_16sc_t* aVector, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li aVector: Vector of complex items to be conjugated
 * \li num_points: The number of complex data points.
 *
 * \b Outputs
 * \li cVector: The vector where the result will be stored
 *
 */

#ifndef INCLUDED_volk_gnsssdr_16ic_conjugate_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_conjugate_16ic_H

#include <volk_gnsssdr/volk_gnsssdr_complex.h>


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_conjugate_16ic_generic(lv_16sc_t* cVector, const lv_16sc_t* aVector, unsigned int num_points)
{
    lv_16sc_t* cPtr = cVector;
    const lv_16sc_t* aPtr = aVector;
    unsigned int number;

    for (number = 0; number < num_points; number++)
        {
            *cPtr++ = lv_conj(*aPtr++);
        }
}

#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSSE3
#include <tmmintrin.h>

static inline void volk_gnsssdr_16ic_conjugate_16ic_u_ssse3(lv_16sc_t* cVector, const lv_16sc_t* aVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    unsigned int i;
    lv_16sc_t* c = cVector;
    const lv_16sc_t* a = aVector;
    __m128i tmp;

    __m128i conjugator = _mm_setr_epi16(1, -1, 1, -1, 1, -1, 1, -1);

    for (i = 0; i < sse_iters; ++i)
        {
            tmp = _mm_lddqu_si128((__m128i*)a);
            tmp = _mm_sign_epi16(tmp, conjugator);
            _mm_storeu_si128((__m128i*)c, tmp);
            a += 4;
            c += 4;
        }

    for (i = sse_iters * 4; i < num_points; ++i)
        {
            *c++ = lv_conj(*a++);
        }
}

#endif /* LV_HAVE_SSSE3 */


#ifdef LV_HAVE_SSSE3
#include <tmmintrin.h>

static inline void volk_gnsssdr_16ic_conjugate_16ic_a_ssse3(lv_16sc_t* cVector, const lv_16sc_t* aVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    unsigned int i;
    lv_16sc_t* c = cVector;
    const lv_16sc_t* a = aVector;
    __m128i tmp;
    __m128i conjugator = _mm_setr_epi16(1, -1, 1, -1, 1, -1, 1, -1);

    for (i = 0; i < sse_iters; ++i)
        {
            tmp = _mm_load_si128((__m128i*)a);
            tmp = _mm_sign_epi16(tmp, conjugator);
            _mm_store_si128((__m128i*)c, tmp);
            a += 4;
            c += 4;
        }

    for (i = sse_iters * 4; i < num_points; ++i)
        {
            *c++ = lv_conj(*a++);
        }
}

#endif /* LV_HAVE_SSSE3 */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_conjugate_16ic_a_avx2(lv_16sc_t* cVector, const lv_16sc_t* aVector, unsigned int num_points)
{
    const unsigned int avx2_iters = num_points / 8;
    unsigned int i;
    lv_16sc_t* c = cVector;
    const lv_16sc_t* a = aVector;

    __m256i tmp;
    __m256i conjugator = _mm256_setr_epi16(1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1);

    for (i = 0; i < avx2_iters; ++i)
        {
            tmp = _mm256_load_si256((__m256i*)a);
            tmp = _mm256_sign_epi16(tmp, conjugator);
            _mm256_store_si256((__m256i*)c, tmp);

            a += 8;
            c += 8;
        }

    for (i = avx2_iters * 8; i < num_points; ++i)
        {
            *c++ = lv_conj(*a++);
        }
}

#endif /* LV_HAVE_AVX2 */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_conjugate_16ic_u_avx2(lv_16sc_t* cVector, const lv_16sc_t* aVector, unsigned int num_points)
{
    const unsigned int avx2_iters = num_points / 8;
    unsigned int i;
    lv_16sc_t* c = cVector;
    const lv_16sc_t* a = aVector;

    __m256i tmp;
    __m256i conjugator = _mm256_setr_epi16(1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1);

    for (i = 0; i < avx2_iters; ++i)
        {
            tmp = _mm256_loadu_si256((__m256i*)a);
            tmp = _mm256_sign_epi16(tmp, conjugator);
            _mm256_storeu_si256((__m256i*)c, tmp);

            a += 8;
            c += 8;
        }

    for (i = avx2_iters * 8; i < num_points; ++i)
        {
            *c++ = lv_conj(*a++);
        }
}
#endif /* LV_HAVE_AVX2 */

#endif /* INCLUDED_volk_gnsssdr_16ic_conjugate_16ic_H */
