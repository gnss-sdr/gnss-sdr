/*!
 * \file volk_gnsssdr_8ic_conjugate_8ic.h
 * \brief Volk protokernel: calculates the conjugate of a 16 bits vector
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that calculates the conjugate of a 
 * 16 bits vector (8 bits the real part and 8 bits the imaginary part)
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

#ifndef INCLUDED_volk_gnsssdr_8ic_conjugate_8ic_u_H
#define INCLUDED_volk_gnsssdr_8ic_conjugate_8ic_u_H

#include <inttypes.h>
#include <stdio.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>

#ifdef LV_HAVE_AVX
#include <immintrin.h>
/*!
 \brief Takes the conjugate of an unsigned char vector.
 \param cVector The vector where the results will be stored
 \param aVector Vector to be conjugated
 \param num_points The number of unsigned char values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_conjugate_8ic_u_avx(lv_8sc_t* cVector, const lv_8sc_t* aVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;

    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;

    __m256 tmp;
    __m128i tmp128lo, tmp128hi;
    __m256 conjugator1 = _mm256_castsi256_ps(_mm256_setr_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255));
    __m128i conjugator2 = _mm_setr_epi8(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);

    for (unsigned int i = 0; i < sse_iters; ++i)
        {
            tmp = _mm256_loadu_ps((float*)a);
            tmp = _mm256_xor_ps(tmp, conjugator1);
            tmp128lo = _mm256_castsi256_si128(_mm256_castps_si256(tmp));
            tmp128lo = _mm_add_epi8(tmp128lo, conjugator2);
            tmp128hi = _mm256_extractf128_si256(_mm256_castps_si256(tmp),1);
            tmp128hi = _mm_add_epi8(tmp128hi, conjugator2);
            //tmp = _mm256_set_m128i(tmp128hi , tmp128lo); //not defined in some versions of immintrin.h
            tmp = _mm256_castsi256_ps(_mm256_insertf128_si256(_mm256_castsi128_si256(tmp128lo),(tmp128hi),1));
            _mm256_storeu_ps((float*)c, tmp);

            a += 16;
            c += 16;
        }

    for (unsigned int i = 0; i<(num_points % 16); ++i)
        {
            *c++ = lv_conj(*a++);
        }
}
#endif /* LV_HAVE_AVX */

#ifdef LV_HAVE_SSSE3
#include <tmmintrin.h>
/*!
 \brief Takes the conjugate of an unsigned char vector.
 \param cVector The vector where the results will be stored
 \param aVector Vector to be conjugated
 \param num_points The number of unsigned char values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_conjugate_8ic_u_ssse3(lv_8sc_t* cVector, const lv_8sc_t* aVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    __m128i tmp;

    __m128i conjugator = _mm_setr_epi8(1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1);

    for (unsigned int i = 0; i < sse_iters; ++i)
        {
            tmp = _mm_lddqu_si128((__m128i*)a);
            tmp = _mm_sign_epi8(tmp, conjugator);
            _mm_storeu_si128((__m128i*)c, tmp);
            a += 8;
            c += 8;
        }

    for (unsigned int i = 0; i<(num_points % 8); ++i)
        {
            *c++ = lv_conj(*a++);
        }

}
#endif /* LV_HAVE_SSSE3 */

#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>
/*!
 \brief Takes the conjugate of an unsigned char vector.
 \param cVector The vector where the results will be stored
 \param aVector Vector to be conjugated
 \param num_points The number of unsigned char values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_conjugate_8ic_u_sse3(lv_8sc_t* cVector, const lv_8sc_t* aVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    __m128i tmp;

    __m128i conjugator1 = _mm_setr_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);
    __m128i conjugator2 = _mm_setr_epi8(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);

    for (unsigned int i = 0; i < sse_iters; ++i)
        {
            tmp = _mm_lddqu_si128((__m128i*)a);
            tmp = _mm_xor_si128(tmp, conjugator1);
            tmp = _mm_add_epi8(tmp, conjugator2);
            _mm_storeu_si128((__m128i*)c, tmp);
            a += 8;
            c += 8;
        }

    for (unsigned int i = 0; i<(num_points % 8); ++i)
        {
            *c++ = lv_conj(*a++);
        }

}
#endif /* LV_HAVE_SSE3 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Takes the conjugate of an unsigned char vector.
 \param cVector The vector where the results will be stored
 \param aVector Vector to be conjugated
 \param num_points The number of unsigned char values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_conjugate_8ic_generic(lv_8sc_t* cVector, const lv_8sc_t* aVector, unsigned int num_points)
{
    lv_8sc_t* cPtr = cVector;
    const lv_8sc_t* aPtr = aVector;
    unsigned int number = 0;

    for(number = 0; number < num_points; number++)
        {
            *cPtr++ = lv_conj(*aPtr++);
        }
}
#endif /* LV_HAVE_GENERIC */

#endif /* INCLUDED_volk_gnsssdr_8ic_conjugate_8ic_u_H */


#ifndef INCLUDED_volk_gnsssdr_8ic_conjugate_8ic_a_H
#define INCLUDED_volk_gnsssdr_8ic_conjugate_8ic_a_H

#include <inttypes.h>
#include <stdio.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>

#ifdef LV_HAVE_AVX
#include <immintrin.h>
/*!
 \brief Takes the conjugate of an unsigned char vector.
 \param cVector The vector where the results will be stored
 \param aVector Vector to be conjugated
 \param num_points The number of unsigned char values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_conjugate_8ic_a_avx(lv_8sc_t* cVector, const lv_8sc_t* aVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 16;

    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;

    __m256 tmp;
    __m128i tmp128lo, tmp128hi;
    __m256 conjugator1 = _mm256_castsi256_ps(_mm256_setr_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255));
    __m128i conjugator2 = _mm_setr_epi8(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);

    for (unsigned int i = 0; i < sse_iters; ++i)
        {
            tmp = _mm256_load_ps((float*)a);
            tmp = _mm256_xor_ps(tmp, conjugator1);
            tmp128lo = _mm256_castsi256_si128(_mm256_castps_si256(tmp));
            tmp128lo = _mm_add_epi8(tmp128lo, conjugator2);
            tmp128hi = _mm256_extractf128_si256(_mm256_castps_si256(tmp),1);
            tmp128hi = _mm_add_epi8(tmp128hi, conjugator2);
            //tmp = _mm256_set_m128i(tmp128hi , tmp128lo); //not defined in some versions of immintrin.h
            tmp = _mm256_castsi256_ps(_mm256_insertf128_si256(_mm256_castsi128_si256(tmp128lo),(tmp128hi),1));
            _mm256_store_ps((float*)c, tmp);

            a += 16;
            c += 16;
        }

    for (unsigned int i = 0; i<(num_points % 16); ++i)
        {
            *c++ = lv_conj(*a++);
        }
}
#endif /* LV_HAVE_AVX */

#ifdef LV_HAVE_SSSE3
#include <tmmintrin.h>
/*!
 \brief Takes the conjugate of an unsigned char vector.
 \param cVector The vector where the results will be stored
 \param aVector Vector to be conjugated
 \param num_points The number of unsigned char values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_conjugate_8ic_a_ssse3(lv_8sc_t* cVector, const lv_8sc_t* aVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    __m128i tmp;

    __m128i conjugator = _mm_setr_epi8(1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1);

    for (unsigned int i = 0; i < sse_iters; ++i)
        {
            tmp = _mm_load_si128((__m128i*)a);
            tmp = _mm_sign_epi8(tmp, conjugator);
            _mm_store_si128((__m128i*)c, tmp);
            a += 8;
            c += 8;
        }

    for (unsigned int i = 0; i<(num_points % 8); ++i)
        {
            *c++ = lv_conj(*a++);
        }

}
#endif /* LV_HAVE_SSSE3 */

#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>
/*!
 \brief Takes the conjugate of an unsigned char vector.
 \param cVector The vector where the results will be stored
 \param aVector Vector to be conjugated
 \param num_points The number of unsigned char values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_conjugate_8ic_a_sse3(lv_8sc_t* cVector, const lv_8sc_t* aVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    __m128i tmp;

    __m128i conjugator1 = _mm_setr_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);
    __m128i conjugator2 = _mm_setr_epi8(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);

    for (unsigned int i = 0; i < sse_iters; ++i)
        {
            tmp = _mm_load_si128((__m128i*)a);
            tmp = _mm_xor_si128(tmp, conjugator1);
            tmp = _mm_add_epi8(tmp, conjugator2);
            _mm_store_si128((__m128i*)c, tmp);
            a += 8;
            c += 8;
        }

    for (unsigned int i = 0; i<(num_points % 8); ++i)
        {
            *c++ = lv_conj(*a++);
        }

}
#endif /* LV_HAVE_SSE3 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Takes the conjugate of an unsigned char vector.
 \param cVector The vector where the results will be stored
 \param aVector Vector to be conjugated
 \param num_points The number of unsigned char values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_8ic_conjugate_8ic_a_generic(lv_8sc_t* cVector, const lv_8sc_t* aVector, unsigned int num_points)
{
    lv_8sc_t* cPtr = cVector;
    const lv_8sc_t* aPtr = aVector;
    unsigned int number = 0;

    for(number = 0; number < num_points; number++)
        {
            *cPtr++ = lv_conj(*aPtr++);
        }
}
#endif /* LV_HAVE_GENERIC */

#ifdef LV_HAVE_ORC
/*!
 \brief Takes the conjugate of an unsigned char vector.
 \param cVector The vector where the results will be stored
 \param aVector Vector to be conjugated
 \param num_points The number of unsigned char values in aVector to be conjugated and stored into cVector
 */
extern void volk_gnsssdr_8ic_conjugate_8ic_a_orc_impl(lv_8sc_t* cVector, const lv_8sc_t* aVector, unsigned int num_points);
static inline void volk_gnsssdr_8ic_conjugate_8ic_u_orc(lv_8sc_t* cVector, const lv_8sc_t* aVector, unsigned int num_points)
{
    volk_gnsssdr_8ic_conjugate_8ic_a_orc_impl(cVector, aVector, num_points);
}
#endif /* LV_HAVE_ORC */

#endif /* INCLUDED_volk_gnsssdr_8ic_conjugate_8ic_a_H */
