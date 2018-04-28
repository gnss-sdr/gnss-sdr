/*!
 * \file volk_gnsssdr_8ic_x2_dot_prod_8ic.h
 * \brief VOLK_GNSSSDR kernel: multiplies two 16 bits vectors and accumulates them.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that multiplies two 16 bits vectors (8 bits the real part
 * and 8 bits the imaginary part) and accumulates them.
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

/*!
 * \page volk_gnsssdr_8ic_x2_dot_prod_8ic
 *
 * \b Overview
 *
 * Multiplies two input complex vectors (8-bit integer each component) and accumulates them,
 * storing the result.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_16ic_x2_dot_prod_16ic(lv_16sc_t* result, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li in_a:          One of the vectors to be multiplied and accumulated
 * \li in_b:          The other vector to be multiplied and accumulated
 * \li num_points:    The Number of complex values to be multiplied together, accumulated and stored into \p result
 *
 * \b Outputs
 * \li result:        Value of the accumulated result
 *
 */

#ifndef INCLUDED_volk_gnsssdr_8ic_x2_dot_prod_8ic_H
#define INCLUDED_volk_gnsssdr_8ic_x2_dot_prod_8ic_H

#include <string.h>
#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>

#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_generic(lv_8sc_t* result, const lv_8sc_t* in_a, const lv_8sc_t* in_b, unsigned int num_points)
{
    /*lv_8sc_t* cPtr = result;
     const lv_8sc_t* aPtr = in_a;
     const lv_8sc_t* bPtr = in_b;

     for(int number = 0; number < num_points; number++){
     *cPtr += (*aPtr++) * (*bPtr++);
     }*/

    char* res = (char*)result;
    char* in = (char*)in_a;
    char* tp = (char*)in_b;
    unsigned int n_2_ccomplex_blocks = num_points / 2;
    unsigned int isodd = num_points & 1;

    char sum0[2] = {0, 0};
    char sum1[2] = {0, 0};
    unsigned int i = 0;

    for (i = 0; i < n_2_ccomplex_blocks; ++i)
        {
            sum0[0] += in[0] * tp[0] - in[1] * tp[1];
            sum0[1] += in[0] * tp[1] + in[1] * tp[0];
            sum1[0] += in[2] * tp[2] - in[3] * tp[3];
            sum1[1] += in[2] * tp[3] + in[3] * tp[2];

            in += 4;
            tp += 4;
        }

    res[0] = sum0[0] + sum1[0];
    res[1] = sum0[1] + sum1[1];

    // Cleanup if we had an odd number of points
    for (i = 0; i < isodd; ++i)
        {
            *result += in_a[num_points - 1] * in_b[num_points - 1];
        }
}

#endif /*LV_HAVE_GENERIC*/


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_u_sse2(lv_8sc_t* result, const lv_8sc_t* in_a, const lv_8sc_t* in_b, unsigned int num_points)
{
    lv_8sc_t dotProduct;
    memset(&dotProduct, 0x0, 2 * sizeof(char));
    unsigned int number;
    unsigned int i;
    const lv_8sc_t* a = in_a;
    const lv_8sc_t* b = in_b;

    const unsigned int sse_iters = num_points / 8;

    if (sse_iters > 0)
        {
            __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc, realcacc, imagcacc;

            mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);
            realcacc = _mm_setzero_si128();
            imagcacc = _mm_setzero_si128();

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
                    imagc = _mm_add_epi16(realx_mult_imagy, imagx_mult_realy);

                    realcacc = _mm_add_epi16(realcacc, realc);
                    imagcacc = _mm_add_epi16(imagcacc, imagc);

                    a += 8;
                    b += 8;
                }

            realcacc = _mm_and_si128(realcacc, mult1);
            imagcacc = _mm_and_si128(imagcacc, mult1);
            imagcacc = _mm_slli_si128(imagcacc, 1);

            totalc = _mm_or_si128(realcacc, imagcacc);

            __VOLK_ATTR_ALIGNED(16)
            lv_8sc_t dotProductVector[8];

            _mm_storeu_si128((__m128i*)dotProductVector, totalc);  // Store the results back into the dot product vector

            for (i = 0; i < 8; ++i)
                {
                    dotProduct += dotProductVector[i];
                }
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            dotProduct += (*a++) * (*b++);
        }

    *result = dotProduct;
}

#endif /*LV_HAVE_SSE2*/


#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>

static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_u_sse4_1(lv_8sc_t* result, const lv_8sc_t* in_a, const lv_8sc_t* in_b, unsigned int num_points)
{
    lv_8sc_t dotProduct;
    memset(&dotProduct, 0x0, 2 * sizeof(char));
    unsigned int number;
    unsigned int i;
    const lv_8sc_t* a = in_a;
    const lv_8sc_t* b = in_b;

    const unsigned int sse_iters = num_points / 8;

    if (sse_iters > 0)
        {
            __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc, realcacc, imagcacc;

            mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);
            realcacc = _mm_setzero_si128();
            imagcacc = _mm_setzero_si128();

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

                    realcacc = _mm_add_epi16(realcacc, realc);
                    imagcacc = _mm_add_epi16(imagcacc, imagc);

                    a += 8;
                    b += 8;
                }

            imagcacc = _mm_slli_si128(imagcacc, 1);

            totalc = _mm_blendv_epi8(imagcacc, realcacc, mult1);

            __VOLK_ATTR_ALIGNED(16)
            lv_8sc_t dotProductVector[8];

            _mm_storeu_si128((__m128i*)dotProductVector, totalc);  // Store the results back into the dot product vector

            for (i = 0; i < 8; ++i)
                {
                    dotProduct += dotProductVector[i];
                }
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            dotProduct += (*a++) * (*b++);
        }

    *result = dotProduct;
}

#endif /*LV_HAVE_SSE4_1*/


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_a_sse2(lv_8sc_t* result, const lv_8sc_t* in_a, const lv_8sc_t* in_b, unsigned int num_points)
{
    lv_8sc_t dotProduct;
    memset(&dotProduct, 0x0, 2 * sizeof(char));
    unsigned int number;
    unsigned int i;
    const lv_8sc_t* a = in_a;
    const lv_8sc_t* b = in_b;

    const unsigned int sse_iters = num_points / 8;

    if (sse_iters > 0)
        {
            __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc, realcacc, imagcacc;

            mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);
            realcacc = _mm_setzero_si128();
            imagcacc = _mm_setzero_si128();

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

                    realcacc = _mm_add_epi16(realcacc, realc);
                    imagcacc = _mm_add_epi16(imagcacc, imagc);

                    a += 8;
                    b += 8;
                }

            realcacc = _mm_and_si128(realcacc, mult1);
            imagcacc = _mm_and_si128(imagcacc, mult1);
            imagcacc = _mm_slli_si128(imagcacc, 1);

            totalc = _mm_or_si128(realcacc, imagcacc);

            __VOLK_ATTR_ALIGNED(16)
            lv_8sc_t dotProductVector[8];

            _mm_store_si128((__m128i*)dotProductVector, totalc);  // Store the results back into the dot product vector

            for (i = 0; i < 8; ++i)
                {
                    dotProduct += dotProductVector[i];
                }
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            dotProduct += (*a++) * (*b++);
        }

    *result = dotProduct;
}

#endif /*LV_HAVE_SSE2*/

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>

static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_a_sse4_1(lv_8sc_t* result, const lv_8sc_t* in_a, const lv_8sc_t* in_b, unsigned int num_points)
{
    lv_8sc_t dotProduct;
    memset(&dotProduct, 0x0, 2 * sizeof(char));
    unsigned int number;
    unsigned int i;
    const lv_8sc_t* a = in_a;
    const lv_8sc_t* b = in_b;

    const unsigned int sse_iters = num_points / 8;

    if (sse_iters > 0)
        {
            __m128i x, y, mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, realc, imagc, totalc, realcacc, imagcacc;

            mult1 = _mm_set_epi8(0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF);
            realcacc = _mm_setzero_si128();
            imagcacc = _mm_setzero_si128();

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

                    realcacc = _mm_add_epi16(realcacc, realc);
                    imagcacc = _mm_add_epi16(imagcacc, imagc);

                    a += 8;
                    b += 8;
                }

            imagcacc = _mm_slli_si128(imagcacc, 1);

            totalc = _mm_blendv_epi8(imagcacc, realcacc, mult1);

            __VOLK_ATTR_ALIGNED(16)
            lv_8sc_t dotProductVector[8];

            _mm_store_si128((__m128i*)dotProductVector, totalc);  // Store the results back into the dot product vector

            for (i = 0; i < 8; ++i)
                {
                    dotProduct += dotProductVector[i];
                }
        }

    for (i = sse_iters * 8; i < num_points; ++i)
        {
            dotProduct += (*a++) * (*b++);
        }

    *result = dotProduct;
}

#endif /*LV_HAVE_SSE4_1*/


#ifdef LV_HAVE_ORC

extern void volk_gnsssdr_8ic_x2_dot_prod_8ic_a_orc_impl(short* resRealShort, short* resImagShort, const lv_8sc_t* in_a, const lv_8sc_t* in_b, unsigned int num_points);
static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_u_orc(lv_8sc_t* result, const lv_8sc_t* in_a, const lv_8sc_t* in_b, unsigned int num_points)
{
    short resReal = 0;
    char* resRealChar = (char*)&resReal;
    resRealChar++;

    short resImag = 0;
    char* resImagChar = (char*)&resImag;
    resImagChar++;

    volk_gnsssdr_8ic_x2_dot_prod_8ic_a_orc_impl(&resReal, &resImag, in_a, in_b, num_points);

    *result = lv_cmake(*resRealChar, *resImagChar);
}
#endif /* LV_HAVE_ORC */


#ifdef LV_HAVE_NEONV7
#include <arm_neon.h>

static inline void volk_gnsssdr_8ic_x2_dot_prod_8ic_neon(lv_8sc_t* result, const lv_8sc_t* in_a, const lv_8sc_t* in_b, unsigned int num_points)
{
    lv_8sc_t dotProduct;
    dotProduct = lv_cmake(0, 0);
    *result = lv_cmake(0, 0);

    const lv_8sc_t* a = in_a;
    const lv_8sc_t* b = in_b;
    // for 2-lane vectors, 1st lane holds the real part,
    // 2nd lane holds the imaginary part
    int8x8x2_t a_val, b_val, c_val, accumulator, tmp_real, tmp_imag;
    __VOLK_ATTR_ALIGNED(16)
    lv_8sc_t accum_result[8] = {lv_cmake(0, 0)};
    accumulator.val[0] = vdup_n_s8(0);
    accumulator.val[1] = vdup_n_s8(0);
    unsigned int number;

    const unsigned int neon_iters = num_points / 8;

    for (number = 0; number < neon_iters; ++number)
        {
            a_val = vld2_s8((const int8_t*)a);
            b_val = vld2_s8((const int8_t*)b);
            __VOLK_GNSSSDR_PREFETCH(a + 16);
            __VOLK_GNSSSDR_PREFETCH(b + 16);

            // multiply the real*real and imag*imag to get real result
            tmp_real.val[0] = vmul_s8(a_val.val[0], b_val.val[0]);
            tmp_real.val[1] = vmul_s8(a_val.val[1], b_val.val[1]);

            // Multiply cross terms to get the imaginary result
            tmp_imag.val[0] = vmul_s8(a_val.val[0], b_val.val[1]);
            tmp_imag.val[1] = vmul_s8(a_val.val[1], b_val.val[0]);

            c_val.val[0] = vsub_s8(tmp_real.val[0], tmp_real.val[1]);
            c_val.val[1] = vadd_s8(tmp_imag.val[0], tmp_imag.val[1]);

            accumulator.val[0] = vadd_s8(accumulator.val[0], c_val.val[0]);
            accumulator.val[1] = vadd_s8(accumulator.val[1], c_val.val[1]);

            a += 8;
            b += 8;
        }
    vst2_s8((int8_t*)accum_result, accumulator);
    for (number = 0; number < 8; ++number)
        {
            *result += accum_result[number];
        }

    for (number = neon_iters * 8; number < num_points; ++number)
        {
            dotProduct += (*a++) * (*b++);
        }

    *result += dotProduct;
}
#endif /* LV_HAVE_NEONV7 */

#endif /*INCLUDED_volk_gnsssdr_8ic_x2_dot_prod_8ic_H*/
