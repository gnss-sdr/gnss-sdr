/*!
 * \file volk_gnsssdr_16ic_x2_dot_prod_16ic.h
 * \brief VOLK_GNSSSDR kernel: multiplies two 16 bits vectors and accumulates them.
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that multiplies two 16 bits vectors (8 bits the real part
 * and 8 bits the imaginary part) and accumulates them
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
 * \page volk_gnsssdr_16ic_x2_dot_prod_16ic
 *
 * \b Overview
 *
 * Multiplies two input complex vectors (16-bit integer each component) and accumulates them,
 * storing the result. Results are saturated so never go beyond the limits of the data type.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_16ic_x2_dot_prod_16ic(lv_16sc_t* result, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li in_a:          One of the vectors to be multiplied and accumulated.
 * \li in_b:          The other vector to be multiplied and accumulated.
 * \li num_points:    Number of complex values to be multiplied together, accumulated and stored into \p result
 *
 * \b Outputs
 * \li result:        Value of the accumulated result.
 *
 */

#ifndef INCLUDED_volk_gnsssdr_16ic_x2_dot_prod_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_x2_dot_prod_16ic_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/saturation_arithmetic.h>


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_generic(lv_16sc_t* result, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points)
{
    result[0] = lv_cmake((int16_t)0, (int16_t)0);
    unsigned int n;
    for (n = 0; n < num_points; n++)
        {
            lv_16sc_t tmp = in_a[n] * in_b[n];
            result[0] = lv_cmake(sat_adds16i(lv_creal(result[0]), lv_creal(tmp)), sat_adds16i(lv_cimag(result[0]), lv_cimag(tmp)));
        }
}

#endif /*LV_HAVE_GENERIC*/


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_a_sse2(lv_16sc_t* out, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake((int16_t)0, (int16_t)0);

    const unsigned int sse_iters = num_points / 4;
    unsigned int number;
    const lv_16sc_t* _in_a = in_a;
    const lv_16sc_t* _in_b = in_b;
    lv_16sc_t* _out = out;

    if (sse_iters > 0)
        {
            __m128i a, b, c, c_sr, mask_imag, mask_real, real, imag, imag1, imag2, b_sl, a_sl, realcacc, imagcacc;
            __VOLK_ATTR_ALIGNED(16)
            lv_16sc_t dotProductVector[4];

            realcacc = _mm_setzero_si128();
            imagcacc = _mm_setzero_si128();

            mask_imag = _mm_set_epi8(0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0);
            mask_real = _mm_set_epi8(0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF);

            for (number = 0; number < sse_iters; number++)
                {
                    // a[127:0]=[a3.i,a3.r,a2.i,a2.r,a1.i,a1.r,a0.i,a0.r]
                    a = _mm_load_si128((__m128i*)_in_a);  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
                    __VOLK_GNSSSDR_PREFETCH(_in_a + 8);
                    b = _mm_load_si128((__m128i*)_in_b);
                    __VOLK_GNSSSDR_PREFETCH(_in_b + 8);
                    c = _mm_mullo_epi16(a, b);  // a3.i*b3.i, a3.r*b3.r, ....

                    c_sr = _mm_srli_si128(c, 2);  // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
                    real = _mm_subs_epi16(c, c_sr);

                    b_sl = _mm_slli_si128(b, 2);  // b3.r, b2.i ....
                    a_sl = _mm_slli_si128(a, 2);  // a3.r, a2.i ....

                    imag1 = _mm_mullo_epi16(a, b_sl);  // a3.i*b3.r, ....
                    imag2 = _mm_mullo_epi16(b, a_sl);  // b3.i*a3.r, ....

                    imag = _mm_adds_epi16(imag1, imag2);  //with saturation arithmetic!

                    realcacc = _mm_adds_epi16(realcacc, real);
                    imagcacc = _mm_adds_epi16(imagcacc, imag);

                    _in_a += 4;
                    _in_b += 4;
                }

            realcacc = _mm_and_si128(realcacc, mask_real);
            imagcacc = _mm_and_si128(imagcacc, mask_imag);

            a = _mm_or_si128(realcacc, imagcacc);

            _mm_store_si128((__m128i*)dotProductVector, a);  // Store the results back into the dot product vector

            for (number = 0; number < 4; ++number)
                {
                    dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[number])), sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[number])));
                }
        }

    for (number = 0; number < (num_points % 4); ++number)
        {
            lv_16sc_t tmp = (*_in_a++) * (*_in_b++);
            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(tmp)), sat_adds16i(lv_cimag(dotProduct), lv_cimag(tmp)));
        }

    *_out = dotProduct;
}

#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_u_sse2(lv_16sc_t* out, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake((int16_t)0, (int16_t)0);

    const unsigned int sse_iters = num_points / 4;

    const lv_16sc_t* _in_a = in_a;
    const lv_16sc_t* _in_b = in_b;
    lv_16sc_t* _out = out;
    unsigned int i;
    unsigned int number;

    if (sse_iters > 0)
        {
            __m128i a, b, c, c_sr, mask_imag, mask_real, real, imag, imag1, imag2, b_sl, a_sl, realcacc, imagcacc, result;
            __VOLK_ATTR_ALIGNED(16)
            lv_16sc_t dotProductVector[4];

            realcacc = _mm_setzero_si128();
            imagcacc = _mm_setzero_si128();

            mask_imag = _mm_set_epi8(0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0);
            mask_real = _mm_set_epi8(0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF);

            for (number = 0; number < sse_iters; number++)
                {
                    //std::complex<T> memory structure: real part -> reinterpret_cast<cv T*>(a)[2*i]
                    //imaginery part -> reinterpret_cast<cv T*>(a)[2*i + 1]
                    // a[127:0]=[a3.i,a3.r,a2.i,a2.r,a1.i,a1.r,a0.i,a0.r]
                    a = _mm_loadu_si128((__m128i*)_in_a);  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
                    __VOLK_GNSSSDR_PREFETCH(_in_a + 8);
                    b = _mm_loadu_si128((__m128i*)_in_b);
                    __VOLK_GNSSSDR_PREFETCH(_in_b + 8);
                    c = _mm_mullo_epi16(a, b);  // a3.i*b3.i, a3.r*b3.r, ....

                    c_sr = _mm_srli_si128(c, 2);  // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
                    real = _mm_subs_epi16(c, c_sr);

                    b_sl = _mm_slli_si128(b, 2);  // b3.r, b2.i ....
                    a_sl = _mm_slli_si128(a, 2);  // a3.r, a2.i ....

                    imag1 = _mm_mullo_epi16(a, b_sl);  // a3.i*b3.r, ....
                    imag2 = _mm_mullo_epi16(b, a_sl);  // b3.i*a3.r, ....

                    imag = _mm_adds_epi16(imag1, imag2);  //with saturation arithmetic!

                    realcacc = _mm_adds_epi16(realcacc, real);
                    imagcacc = _mm_adds_epi16(imagcacc, imag);

                    _in_a += 4;
                    _in_b += 4;
                }

            realcacc = _mm_and_si128(realcacc, mask_real);
            imagcacc = _mm_and_si128(imagcacc, mask_imag);

            result = _mm_or_si128(realcacc, imagcacc);

            _mm_storeu_si128((__m128i*)dotProductVector, result);  // Store the results back into the dot product vector

            for (i = 0; i < 4; ++i)
                {
                    dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])), sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
                }
        }

    for (i = 0; i < (num_points % 4); ++i)
        {
            lv_16sc_t tmp = (*_in_a++) * (*_in_b++);
            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(tmp)), sat_adds16i(lv_cimag(dotProduct), lv_cimag(tmp)));
        }

    *_out = dotProduct;
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_u_axv2(lv_16sc_t* out, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake((int16_t)0, (int16_t)0);

    const unsigned int avx_iters = num_points / 8;

    const lv_16sc_t* _in_a = in_a;
    const lv_16sc_t* _in_b = in_b;
    lv_16sc_t* _out = out;
    unsigned int i;
    unsigned int number;

    if (avx_iters > 0)
        {
            __m256i a, b, c, c_sr, mask_imag, mask_real, real, imag, imag1, imag2, b_sl, a_sl, realcacc, imagcacc, result;
            __VOLK_ATTR_ALIGNED(32)
            lv_16sc_t dotProductVector[8];

            realcacc = _mm256_setzero_si256();
            imagcacc = _mm256_setzero_si256();

            mask_imag = _mm256_set_epi8(0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0);
            mask_real = _mm256_set_epi8(0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF);

            for (number = 0; number < avx_iters; number++)
                {
                    a = _mm256_loadu_si256((__m256i*)_in_a);
                    __VOLK_GNSSSDR_PREFETCH(_in_a + 16);
                    b = _mm256_loadu_si256((__m256i*)_in_b);
                    __VOLK_GNSSSDR_PREFETCH(_in_b + 16);
                    c = _mm256_mullo_epi16(a, b);

                    c_sr = _mm256_srli_si256(c, 2);  // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
                    real = _mm256_subs_epi16(c, c_sr);

                    b_sl = _mm256_slli_si256(b, 2);
                    a_sl = _mm256_slli_si256(a, 2);

                    imag1 = _mm256_mullo_epi16(a, b_sl);
                    imag2 = _mm256_mullo_epi16(b, a_sl);

                    imag = _mm256_adds_epi16(imag1, imag2);  //with saturation arithmetic!

                    realcacc = _mm256_adds_epi16(realcacc, real);
                    imagcacc = _mm256_adds_epi16(imagcacc, imag);

                    _in_a += 8;
                    _in_b += 8;
                }

            realcacc = _mm256_and_si256(realcacc, mask_real);
            imagcacc = _mm256_and_si256(imagcacc, mask_imag);

            result = _mm256_or_si256(realcacc, imagcacc);

            _mm256_storeu_si256((__m256i*)dotProductVector, result);  // Store the results back into the dot product vector
            _mm256_zeroupper();

            for (i = 0; i < 8; ++i)
                {
                    dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])), sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
                }
        }

    for (i = 0; i < (num_points % 8); ++i)
        {
            lv_16sc_t tmp = (*_in_a++) * (*_in_b++);
            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(tmp)), sat_adds16i(lv_cimag(dotProduct), lv_cimag(tmp)));
        }

    *_out = dotProduct;
}
#endif /* LV_HAVE_AVX2 */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_a_axv2(lv_16sc_t* out, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake((int16_t)0, (int16_t)0);

    const unsigned int avx_iters = num_points / 8;

    const lv_16sc_t* _in_a = in_a;
    const lv_16sc_t* _in_b = in_b;
    lv_16sc_t* _out = out;
    unsigned int i;
    unsigned int number;

    if (avx_iters > 0)
        {
            __m256i a, b, c, c_sr, mask_imag, mask_real, real, imag, imag1, imag2, b_sl, a_sl, realcacc, imagcacc, result;
            __VOLK_ATTR_ALIGNED(32)
            lv_16sc_t dotProductVector[8];

            realcacc = _mm256_setzero_si256();
            imagcacc = _mm256_setzero_si256();

            mask_imag = _mm256_set_epi8(0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0);
            mask_real = _mm256_set_epi8(0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF);

            for (number = 0; number < avx_iters; number++)
                {
                    a = _mm256_load_si256((__m256i*)_in_a);
                    __VOLK_GNSSSDR_PREFETCH(_in_a + 16);
                    b = _mm256_load_si256((__m256i*)_in_b);
                    __VOLK_GNSSSDR_PREFETCH(_in_b + 16);
                    c = _mm256_mullo_epi16(a, b);

                    c_sr = _mm256_srli_si256(c, 2);  // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
                    real = _mm256_subs_epi16(c, c_sr);

                    b_sl = _mm256_slli_si256(b, 2);
                    a_sl = _mm256_slli_si256(a, 2);

                    imag1 = _mm256_mullo_epi16(a, b_sl);
                    imag2 = _mm256_mullo_epi16(b, a_sl);

                    imag = _mm256_adds_epi16(imag1, imag2);  //with saturation arithmetic!

                    realcacc = _mm256_adds_epi16(realcacc, real);
                    imagcacc = _mm256_adds_epi16(imagcacc, imag);

                    _in_a += 8;
                    _in_b += 8;
                }

            realcacc = _mm256_and_si256(realcacc, mask_real);
            imagcacc = _mm256_and_si256(imagcacc, mask_imag);

            result = _mm256_or_si256(realcacc, imagcacc);

            _mm256_store_si256((__m256i*)dotProductVector, result);  // Store the results back into the dot product vector
            _mm256_zeroupper();

            for (i = 0; i < 8; ++i)
                {
                    dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])), sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
                }
        }

    for (i = 0; i < (num_points % 8); ++i)
        {
            lv_16sc_t tmp = (*_in_a++) * (*_in_b++);
            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(tmp)), sat_adds16i(lv_cimag(dotProduct), lv_cimag(tmp)));
        }

    *_out = dotProduct;
}
#endif /* LV_HAVE_AVX2 */


#ifdef LV_HAVE_NEON
#include <arm_neon.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_neon(lv_16sc_t* out, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points)
{
    unsigned int quarter_points = num_points / 4;
    unsigned int number;

    lv_16sc_t* a_ptr = (lv_16sc_t*)in_a;
    lv_16sc_t* b_ptr = (lv_16sc_t*)in_b;
    *out = lv_cmake((int16_t)0, (int16_t)0);

    if (quarter_points > 0)
        {
            // for 2-lane vectors, 1st lane holds the real part,
            // 2nd lane holds the imaginary part
            int16x4x2_t a_val, b_val, c_val, accumulator;
            int16x4x2_t tmp_real, tmp_imag;
            __VOLK_ATTR_ALIGNED(16)
            lv_16sc_t accum_result[4];
            accumulator.val[0] = vdup_n_s16(0);
            accumulator.val[1] = vdup_n_s16(0);
            lv_16sc_t dotProduct = lv_cmake((int16_t)0, (int16_t)0);

            for (number = 0; number < quarter_points; ++number)
                {
                    a_val = vld2_s16((int16_t*)a_ptr);  // a0r|a1r|a2r|a3r || a0i|a1i|a2i|a3i
                    b_val = vld2_s16((int16_t*)b_ptr);  // b0r|b1r|b2r|b3r || b0i|b1i|b2i|b3i
                    __VOLK_GNSSSDR_PREFETCH(a_ptr + 8);
                    __VOLK_GNSSSDR_PREFETCH(b_ptr + 8);

                    // multiply the real*real and imag*imag to get real result
                    // a0r*b0r|a1r*b1r|a2r*b2r|a3r*b3r
                    tmp_real.val[0] = vmul_s16(a_val.val[0], b_val.val[0]);
                    // a0i*b0i|a1i*b1i|a2i*b2i|a3i*b3i
                    tmp_real.val[1] = vmul_s16(a_val.val[1], b_val.val[1]);

                    // Multiply cross terms to get the imaginary result
                    // a0r*b0i|a1r*b1i|a2r*b2i|a3r*b3i
                    tmp_imag.val[0] = vmul_s16(a_val.val[0], b_val.val[1]);
                    // a0i*b0r|a1i*b1r|a2i*b2r|a3i*b3r
                    tmp_imag.val[1] = vmul_s16(a_val.val[1], b_val.val[0]);

                    c_val.val[0] = vqsub_s16(tmp_real.val[0], tmp_real.val[1]);
                    c_val.val[1] = vqadd_s16(tmp_imag.val[0], tmp_imag.val[1]);

                    accumulator.val[0] = vqadd_s16(accumulator.val[0], c_val.val[0]);
                    accumulator.val[1] = vqadd_s16(accumulator.val[1], c_val.val[1]);

                    a_ptr += 4;
                    b_ptr += 4;
                }

            vst2_s16((int16_t*)accum_result, accumulator);
            for (number = 0; number < 4; ++number)
                {
                    dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(accum_result[number])), sat_adds16i(lv_cimag(dotProduct), lv_cimag(accum_result[number])));
                }

            *out = dotProduct;
        }

    // tail case
    for (number = quarter_points * 4; number < num_points; ++number)
        {
            *out += (*a_ptr++) * (*b_ptr++);
        }
}

#endif /* LV_HAVE_NEON */


#ifdef LV_HAVE_NEON
#include <arm_neon.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_neon_vma(lv_16sc_t* out, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points)
{
    unsigned int quarter_points = num_points / 4;
    unsigned int number;

    lv_16sc_t* a_ptr = (lv_16sc_t*)in_a;
    lv_16sc_t* b_ptr = (lv_16sc_t*)in_b;
    // for 2-lane vectors, 1st lane holds the real part,
    // 2nd lane holds the imaginary part
    int16x4x2_t a_val, b_val, accumulator;
    int16x4x2_t tmp;
    __VOLK_ATTR_ALIGNED(16)
    lv_16sc_t accum_result[4];
    accumulator.val[0] = vdup_n_s16(0);
    accumulator.val[1] = vdup_n_s16(0);

    for (number = 0; number < quarter_points; ++number)
        {
            a_val = vld2_s16((int16_t*)a_ptr);  // a0r|a1r|a2r|a3r || a0i|a1i|a2i|a3i
            b_val = vld2_s16((int16_t*)b_ptr);  // b0r|b1r|b2r|b3r || b0i|b1i|b2i|b3i
            __VOLK_GNSSSDR_PREFETCH(a_ptr + 8);
            __VOLK_GNSSSDR_PREFETCH(b_ptr + 8);

            tmp.val[0] = vmul_s16(a_val.val[0], b_val.val[0]);
            tmp.val[1] = vmul_s16(a_val.val[1], b_val.val[0]);

            // use multiply accumulate/subtract to get result
            tmp.val[0] = vmls_s16(tmp.val[0], a_val.val[1], b_val.val[1]);
            tmp.val[1] = vmla_s16(tmp.val[1], a_val.val[0], b_val.val[1]);

            accumulator.val[0] = vqadd_s16(accumulator.val[0], tmp.val[0]);
            accumulator.val[1] = vqadd_s16(accumulator.val[1], tmp.val[1]);

            a_ptr += 4;
            b_ptr += 4;
        }

    vst2_s16((int16_t*)accum_result, accumulator);
    *out = accum_result[0] + accum_result[1] + accum_result[2] + accum_result[3];

    // tail case
    for (number = quarter_points * 4; number < num_points; ++number)
        {
            *out += (*a_ptr++) * (*b_ptr++);
        }
}

#endif /* LV_HAVE_NEON */


#ifdef LV_HAVE_NEON
#include <arm_neon.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_neon_optvma(lv_16sc_t* out, const lv_16sc_t* in_a, const lv_16sc_t* in_b, unsigned int num_points)
{
    unsigned int quarter_points = num_points / 4;
    unsigned int number;

    lv_16sc_t* a_ptr = (lv_16sc_t*)in_a;
    lv_16sc_t* b_ptr = (lv_16sc_t*)in_b;
    // for 2-lane vectors, 1st lane holds the real part,
    // 2nd lane holds the imaginary part
    int16x4x2_t a_val, b_val, accumulator1, accumulator2;

    __VOLK_ATTR_ALIGNED(16)
    lv_16sc_t accum_result[4];
    accumulator1.val[0] = vdup_n_s16(0);
    accumulator1.val[1] = vdup_n_s16(0);
    accumulator2.val[0] = vdup_n_s16(0);
    accumulator2.val[1] = vdup_n_s16(0);

    for (number = 0; number < quarter_points; ++number)
        {
            a_val = vld2_s16((int16_t*)a_ptr);  // a0r|a1r|a2r|a3r || a0i|a1i|a2i|a3i
            b_val = vld2_s16((int16_t*)b_ptr);  // b0r|b1r|b2r|b3r || b0i|b1i|b2i|b3i
            __VOLK_GNSSSDR_PREFETCH(a_ptr + 8);
            __VOLK_GNSSSDR_PREFETCH(b_ptr + 8);

            // use 2 accumulators to remove inter-instruction data dependencies
            accumulator1.val[0] = vmla_s16(accumulator1.val[0], a_val.val[0], b_val.val[0]);
            accumulator2.val[0] = vmls_s16(accumulator2.val[0], a_val.val[1], b_val.val[1]);
            accumulator1.val[1] = vmla_s16(accumulator1.val[1], a_val.val[0], b_val.val[1]);
            accumulator2.val[1] = vmla_s16(accumulator2.val[1], a_val.val[1], b_val.val[0]);

            a_ptr += 4;
            b_ptr += 4;
        }

    accumulator1.val[0] = vqadd_s16(accumulator1.val[0], accumulator2.val[0]);
    accumulator1.val[1] = vqadd_s16(accumulator1.val[1], accumulator2.val[1]);

    vst2_s16((int16_t*)accum_result, accumulator1);
    *out = accum_result[0] + accum_result[1] + accum_result[2] + accum_result[3];

    // tail case
    for (number = quarter_points * 4; number < num_points; ++number)
        {
            *out += (*a_ptr++) * (*b_ptr++);
        }
}

#endif /* LV_HAVE_NEON */

#endif /*INCLUDED_volk_gnsssdr_16ic_x2_dot_prod_16ic_H*/
