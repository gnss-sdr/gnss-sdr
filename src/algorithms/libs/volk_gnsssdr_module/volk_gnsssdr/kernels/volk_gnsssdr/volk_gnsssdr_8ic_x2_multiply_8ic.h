/*!
 * \file volk_gnsssdr_8ic_x2_multiply_8ic.h
 * \brief VOLK_GNSSSDR kernel: multiplies two 16 bits vectors.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that multiplies two 16 bits vectors (8 bits the real part
 * and 8 bits the imaginary part)
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
 * \page volk_gnsssdr_8ic_x2_multiply_8ic
 *
 * \b Overview
 *
 * Multiplies two input complex vectors, point-by-point, storing the result in the third vector
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_8ic_x2_multiply_8ic(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li aVector: One of the vectors to be multiplied
 * \li bVector: The other vector to be multiplied
 * \li num_points: The number of complex data points.
 *
 * \b Outputs
 * \li cVector: The vector where the result will be stored
 *
 */

#ifndef INCLUDED_volk_gnsssdr_8ic_x2_multiply_8ic_H
#define INCLUDED_volk_gnsssdr_8ic_x2_multiply_8ic_H

#include <volk_gnsssdr/volk_gnsssdr_complex.h>

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_u_sse2(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    unsigned int number;
    unsigned int i;
    __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc;
    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    const lv_8sc_t* b = bVector;

    mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);

    for (number = 0; number < sse_iters; number++)
        {
            x = _mm_loadu_si128((__m128i*)a);
            y = _mm_loadu_si128((__m128i*)b);

            imagx = _mm_srli_si128(x, 1);
            imagx = _mm_and_si128(imagx, mult1);
            realx = _mm_and_si128(x, mult1);

            imagy = _mm_srli_si128(y, 1);
            imagy = _mm_and_si128(imagy, mult1);
            realy = _mm_and_si128(y, mult1);

            realx_mult_realy = _mm_mullo_epi16(realx, realy);
            imagx_mult_imagy = _mm_mullo_epi16(imagx, imagy);
            realx_mult_imagy = _mm_mullo_epi16(realx, imagy);
            imagx_mult_realy = _mm_mullo_epi16(imagx, realy);

            realc = _mm_sub_epi16(realx_mult_realy, imagx_mult_imagy);
            realc = _mm_and_si128(realc, mult1);
            imagc = _mm_add_epi16(realx_mult_imagy, imagx_mult_realy);
            imagc = _mm_and_si128(imagc, mult1);
            imagc = _mm_slli_si128(imagc, 1);

            totalc = _mm_or_si128(realc, imagc);

            _mm_storeu_si128((__m128i*)c, totalc);

            a += 8;
            b += 8;
            c += 8;
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_u_sse4_1(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    unsigned int number;
    unsigned int i;
    __m128i x, y;
    __m128i mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc;
    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    const lv_8sc_t* b = bVector;

    _mm_setzero_si128();
    mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);

    for (number = 0; number < sse_iters; number++)
        {
            x = _mm_lddqu_si128((__m128i*)a);
            y = _mm_lddqu_si128((__m128i*)b);

            imagx = _mm_srli_si128(x, 1);
            imagx = _mm_and_si128(imagx, mult1);
            realx = _mm_and_si128(x, mult1);

            imagy = _mm_srli_si128(y, 1);
            imagy = _mm_and_si128(imagy, mult1);
            realy = _mm_and_si128(y, mult1);

            realx_mult_realy = _mm_mullo_epi16(realx, realy);
            imagx_mult_imagy = _mm_mullo_epi16(imagx, imagy);
            realx_mult_imagy = _mm_mullo_epi16(realx, imagy);
            imagx_mult_realy = _mm_mullo_epi16(imagx, realy);

            realc = _mm_sub_epi16(realx_mult_realy, imagx_mult_imagy);
            imagc = _mm_add_epi16(realx_mult_imagy, imagx_mult_realy);
            imagc = _mm_slli_si128(imagc, 1);

            totalc = _mm_blendv_epi8(imagc, realc, mult1);

            _mm_storeu_si128((__m128i*)c, totalc);

            a += 8;
            b += 8;
            c += 8;
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE4_1 */


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_generic(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    lv_8sc_t* cPtr = cVector;
    const lv_8sc_t* aPtr = aVector;
    const lv_8sc_t* bPtr = bVector;
    unsigned int number;

    for (number = 0; number < num_points; number++)
        {
            *cPtr++ = (*aPtr++) * (*bPtr++);
        }
}
#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_a_sse2(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    unsigned int number;
    unsigned int i;
    __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc;
    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    const lv_8sc_t* b = bVector;

    mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);

    for (number = 0; number < sse_iters; number++)
        {
            x = _mm_load_si128((__m128i*)a);
            y = _mm_load_si128((__m128i*)b);

            imagx = _mm_srli_si128(x, 1);
            imagx = _mm_and_si128(imagx, mult1);
            realx = _mm_and_si128(x, mult1);

            imagy = _mm_srli_si128(y, 1);
            imagy = _mm_and_si128(imagy, mult1);
            realy = _mm_and_si128(y, mult1);

            realx_mult_realy = _mm_mullo_epi16(realx, realy);
            imagx_mult_imagy = _mm_mullo_epi16(imagx, imagy);
            realx_mult_imagy = _mm_mullo_epi16(realx, imagy);
            imagx_mult_realy = _mm_mullo_epi16(imagx, realy);

            realc = _mm_sub_epi16(realx_mult_realy, imagx_mult_imagy);
            realc = _mm_and_si128(realc, mult1);
            imagc = _mm_add_epi16(realx_mult_imagy, imagx_mult_realy);
            imagc = _mm_and_si128(imagc, mult1);
            imagc = _mm_slli_si128(imagc, 1);

            totalc = _mm_or_si128(realc, imagc);

            _mm_store_si128((__m128i*)c, totalc);

            a += 8;
            b += 8;
            c += 8;
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_a_sse4_1(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    unsigned int number;
    unsigned int i;
    __m128i x, y;
    __m128i mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc;
    lv_8sc_t* c = cVector;
    const lv_8sc_t* a = aVector;
    const lv_8sc_t* b = bVector;

    _mm_setzero_si128();
    mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);

    for (number = 0; number < sse_iters; number++)
        {
            x = _mm_load_si128((__m128i*)a);
            y = _mm_load_si128((__m128i*)b);

            imagx = _mm_srli_si128(x, 1);
            imagx = _mm_and_si128(imagx, mult1);
            realx = _mm_and_si128(x, mult1);

            imagy = _mm_srli_si128(y, 1);
            imagy = _mm_and_si128(imagy, mult1);
            realy = _mm_and_si128(y, mult1);

            realx_mult_realy = _mm_mullo_epi16(realx, realy);
            imagx_mult_imagy = _mm_mullo_epi16(imagx, imagy);
            realx_mult_imagy = _mm_mullo_epi16(realx, imagy);
            imagx_mult_realy = _mm_mullo_epi16(imagx, realy);

            realc = _mm_sub_epi16(realx_mult_realy, imagx_mult_imagy);
            imagc = _mm_add_epi16(realx_mult_imagy, imagx_mult_realy);
            imagc = _mm_slli_si128(imagc, 1);

            totalc = _mm_blendv_epi8(imagc, realc, mult1);

            _mm_store_si128((__m128i*)c, totalc);

            a += 8;
            b += 8;
            c += 8;
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            *c++ = (*a++) * (*b++);
        }
}
#endif /* LV_HAVE_SSE4_1 */


#ifdef LV_HAVE_RVV
#include <riscv_vector.h>

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_rvv(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    size_t n = num_points;

    // Initialize pointers to track progress as stripmine
    // Assuming that intended to use `signed char`
    // as `char`'s signedness is implementation-specific
    signed char* cPtr = (signed char*)cVector;
    const signed char* aPtr = (const signed char*)aVector;
    const signed char* bPtr = (const signed char*)bVector;

    for (size_t vl; n > 0; n -= vl, cPtr += vl * 2, aPtr += vl * 2, bPtr += vl * 2)
        {
            // Record how many elements will actually be processed
            vl = __riscv_vsetvl_e8m4(n);

            // Load aReal[0..vl), aImag[0..vl)
            vint8m4x2_t aVal = __riscv_vlseg2e8_v_i8m4x2(aPtr, vl);
            vint8m4_t aRealVal = __riscv_vget_v_i8m4x2_i8m4(aVal, 0);
            vint8m4_t aImagVal = __riscv_vget_v_i8m4x2_i8m4(aVal, 1);

            // Load bReal[0..vl), bImag[0..vl)
            vint8m4x2_t bVal = __riscv_vlseg2e8_v_i8m4x2(bPtr, vl);
            vint8m4_t bRealVal = __riscv_vget_v_i8m4x2_i8m4(bVal, 0);
            vint8m4_t bImagVal = __riscv_vget_v_i8m4x2_i8m4(bVal, 1);

            // cReal[i] = aReal[i] * bReal[i]
            vint8m4_t cRealVal = __riscv_vmul_vv_i8m4(aRealVal, bRealVal, vl);

            // cReal[i] = -(aImag[i] * bImag[i]) + cReal[i]
            cRealVal = __riscv_vnmsac_vv_i8m4(cRealVal, aImagVal, bImagVal, vl);

            // cImag[i] = aReal[i] * bImag[i]
            vint8m4_t cImagVal = __riscv_vmul_vv_i8m4(aRealVal, bImagVal, vl);

            // cImag[i] = (aImag[i] * bReal[i]) + cImag[i]
            cImagVal = __riscv_vmacc_vv_i8m4(cImagVal, aImagVal, bRealVal, vl);

            // Store cReal[0..vl), cImag[0..vl)
            vint8m4x2_t cVal = __riscv_vcreate_v_i8m4x2(cRealVal, cImagVal);
            __riscv_vsseg2e8_v_i8m4x2(cPtr, cVal, vl);

            // In looping, decrement the number of
            // elements left and increment the pointers
            // by the number of elements processed,
            // taking into account how the `vl` complex
            // numbers are each stored as two 1-byte `char`s
        }
}
#endif /* LV_HAVE_RVV */


#ifdef LV_HAVE_ORC

extern void volk_gnsssdr_8ic_x2_multiply_8ic_a_orc_impl(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points);

static inline void volk_gnsssdr_8ic_x2_multiply_8ic_u_orc(lv_8sc_t* cVector, const lv_8sc_t* aVector, const lv_8sc_t* bVector, unsigned int num_points)
{
    volk_gnsssdr_8ic_x2_multiply_8ic_a_orc_impl(cVector, aVector, bVector, num_points);
}
#endif /* LV_HAVE_ORC */

#endif /* INCLUDED_volk_gnsssdr_8ic_x2_multiply_8ic_H */
