/*!
 * \file volk_gnsssdr_16ic_x2_dot_prod_16ic_xn.h
 * \brief VOLK_GNSSSDR kernel: multiplies N 16 bits vectors by a common vector and accumulates the results in N 16 bits short complex outputs.
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that multiplies N 16 bits vectors by a common vector and accumulates the results in N 16 bits short complex outputs.
 * It is optimized to perform the N tap correlation process in GNSS receivers.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

/*!
 * \page volk_gnsssdr_16ic_x2_dot_prod_16ic_xn
 *
 * \b Overview
 *
 * Multiplies a reference complex vector by an arbitrary number of other complex vectors, accumulates the results and stores them in the output vector.
 * This function can be used as a multiple correlator.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_16ic_x2_dot_prod_16ic_xn(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_16sc_t** in_a, int num_a_vectors, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li in_common:     Pointer to one of the vectors to be multiplied and accumulated (reference vector)
 * \li in_a:          Pointer to an array of pointers to other vectors to be multiplied by \p in_common and accumulated.
 * \li num_a_vectors: Number of vectors to be multiplied by the reference vector \p in_common and accumulated.
 * \li num_points:    Number of complex values to be multiplied together, accumulated and stored into \p result
 *
 * \b Outputs
 * \li result:        Vector of \p num_a_vectors components with vector \p in_common multiplied by the vectors in \p in_a and accumulated.
 *
 */

#ifndef INCLUDED_volk_gnsssdr_16ic_xn_dot_prod_16ic_xn_H
#define INCLUDED_volk_gnsssdr_16ic_xn_dot_prod_16ic_xn_H


#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/saturation_arithmetic.h>

#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_generic(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_16sc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    int n_vec;
    unsigned int n;
    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            result[n_vec] = lv_cmake(0, 0);
            for (n = 0; n < num_points; n++)
                {
                    //r*a.r - i*a.i, i*a.r + r*a.i
                    //result[n_vec]+=in_common[n]*in_a[n_vec][n];
                    lv_16sc_t tmp = in_common[n] * in_a[n_vec][n];
                    result[n_vec] = lv_cmake(sat_adds16i(lv_creal(result[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(result[n_vec]), lv_cimag(tmp)));
                }
        }
}

#endif /*LV_HAVE_GENERIC*/


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_generic_sat(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_16sc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    int n_vec;
    unsigned int n;
    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            result[n_vec] = lv_cmake(0, 0);
            for (n = 0; n < num_points; n++)
                {
                    lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(in_common[n]), lv_creal(in_a[n_vec][n])), -sat_muls16i(lv_cimag(in_common[n]), lv_cimag(in_a[n_vec][n]))),
                        sat_adds16i(sat_muls16i(lv_creal(in_common[n]), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(in_common[n]), lv_creal(in_a[n_vec][n]))));
                    result[n_vec] = lv_cmake(sat_adds16i(lv_creal(result[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(result[n_vec]), lv_cimag(tmp)));
                }
        }
}

#endif /*LV_HAVE_GENERIC*/


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_a_sse2(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_16sc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake(0, 0);
    int n_vec;
    unsigned int index;
    const unsigned int sse_iters = num_points / 4;

    const lv_16sc_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;

    if (sse_iters > 0)
        {
            __VOLK_ATTR_ALIGNED(16)
            lv_16sc_t dotProductVector[4];

            __m128i* realcacc = (__m128i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m128i), volk_gnsssdr_get_alignment());
            __m128i* imagcacc = (__m128i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m128i), volk_gnsssdr_get_alignment());

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    realcacc[n_vec] = _mm_setzero_si128();
                    imagcacc[n_vec] = _mm_setzero_si128();
                }

            __m128i a, b, c, c_sr, mask_imag, mask_real, real, imag;

            mask_imag = _mm_set_epi8(0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0);
            mask_real = _mm_set_epi8(0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF);

            for (index = 0; index < sse_iters; index++)
                {
                    // b[127:0]=[a3.i,a3.r,a2.i,a2.r,a1.i,a1.r,a0.i,a0.r]
                    b = _mm_load_si128((__m128i*)_in_common);  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
                    __VOLK_GNSSSDR_PREFETCH(_in_common + 8);
                    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            a = _mm_load_si128((__m128i*)&(_in_a[n_vec][index * 4]));  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg

                            c = _mm_mullo_epi16(a, b);  // a3.i*b3.i, a3.r*b3.r, ....

                            c_sr = _mm_srli_si128(c, 2);  // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
                            real = _mm_subs_epi16(c, c_sr);

                            c_sr = _mm_slli_si128(b, 2);   // b3.r, b2.i ....
                            c = _mm_mullo_epi16(a, c_sr);  // a3.i*b3.r, ....

                            c_sr = _mm_slli_si128(a, 2);      // a3.r, a2.i ....
                            imag = _mm_mullo_epi16(b, c_sr);  // b3.i*a3.r, ....

                            imag = _mm_adds_epi16(c, imag);

                            realcacc[n_vec] = _mm_adds_epi16(realcacc[n_vec], real);
                            imagcacc[n_vec] = _mm_adds_epi16(imagcacc[n_vec], imag);
                        }
                    _in_common += 4;
                }

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    realcacc[n_vec] = _mm_and_si128(realcacc[n_vec], mask_real);
                    imagcacc[n_vec] = _mm_and_si128(imagcacc[n_vec], mask_imag);

                    a = _mm_or_si128(realcacc[n_vec], imagcacc[n_vec]);

                    _mm_store_si128((__m128i*)dotProductVector, a);  // Store the results back into the dot product vector
                    dotProduct = lv_cmake(0, 0);
                    for (index = 0; index < 4; ++index)
                        {
                            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[index])),
                                sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[index])));
                        }
                    _out[n_vec] = dotProduct;
                }
            volk_gnsssdr_free(realcacc);
            volk_gnsssdr_free(imagcacc);
        }

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            for (index = sse_iters * 4; index < num_points; index++)
                {
                    lv_16sc_t tmp = in_common[index] * in_a[n_vec][index];

                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_u_sse2(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_16sc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake(0, 0);
    int n_vec;
    unsigned int index;
    const unsigned int sse_iters = num_points / 4;

    const lv_16sc_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;

    if (sse_iters > 0)
        {
            __VOLK_ATTR_ALIGNED(16)
            lv_16sc_t dotProductVector[4];

            __m128i* realcacc = (__m128i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m128i), volk_gnsssdr_get_alignment());
            __m128i* imagcacc = (__m128i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m128i), volk_gnsssdr_get_alignment());

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    realcacc[n_vec] = _mm_setzero_si128();
                    imagcacc[n_vec] = _mm_setzero_si128();
                }

            __m128i a, b, c, c_sr, mask_imag, mask_real, real, imag;

            mask_imag = _mm_set_epi8(0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0);
            mask_real = _mm_set_epi8(0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF);

            for (index = 0; index < sse_iters; index++)
                {
                    // b[127:0]=[a3.i,a3.r,a2.i,a2.r,a1.i,a1.r,a0.i,a0.r]
                    b = _mm_loadu_si128((__m128i*)_in_common);  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
                    __VOLK_GNSSSDR_PREFETCH(_in_common + 8);
                    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            a = _mm_loadu_si128((__m128i*)&(_in_a[n_vec][index * 4]));  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg

                            c = _mm_mullo_epi16(a, b);  // a3.i*b3.i, a3.r*b3.r, ....

                            c_sr = _mm_srli_si128(c, 2);  // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
                            real = _mm_subs_epi16(c, c_sr);

                            c_sr = _mm_slli_si128(b, 2);   // b3.r, b2.i ....
                            c = _mm_mullo_epi16(a, c_sr);  // a3.i*b3.r, ....

                            c_sr = _mm_slli_si128(a, 2);      // a3.r, a2.i ....
                            imag = _mm_mullo_epi16(b, c_sr);  // b3.i*a3.r, ....

                            imag = _mm_adds_epi16(c, imag);

                            realcacc[n_vec] = _mm_adds_epi16(realcacc[n_vec], real);
                            imagcacc[n_vec] = _mm_adds_epi16(imagcacc[n_vec], imag);
                        }
                    _in_common += 4;
                }

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    realcacc[n_vec] = _mm_and_si128(realcacc[n_vec], mask_real);
                    imagcacc[n_vec] = _mm_and_si128(imagcacc[n_vec], mask_imag);

                    a = _mm_or_si128(realcacc[n_vec], imagcacc[n_vec]);

                    _mm_store_si128((__m128i*)dotProductVector, a);  // Store the results back into the dot product vector
                    dotProduct = lv_cmake(0, 0);
                    for (index = 0; index < 4; ++index)
                        {
                            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[index])),
                                sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[index])));
                        }
                    _out[n_vec] = dotProduct;
                }
            volk_gnsssdr_free(realcacc);
            volk_gnsssdr_free(imagcacc);
        }

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            for (index = sse_iters * 4; index < num_points; index++)
                {
                    lv_16sc_t tmp = in_common[index] * in_a[n_vec][index];

                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_a_avx2(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_16sc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake(0, 0);
    int n_vec;
    unsigned int index;
    const unsigned int sse_iters = num_points / 8;

    const lv_16sc_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;

    if (sse_iters > 0)
        {
            __VOLK_ATTR_ALIGNED(32)
            lv_16sc_t dotProductVector[8];

            __m256i* realcacc = (__m256i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m256i), volk_gnsssdr_get_alignment());
            __m256i* imagcacc = (__m256i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m256i), volk_gnsssdr_get_alignment());

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    realcacc[n_vec] = _mm256_setzero_si256();
                    imagcacc[n_vec] = _mm256_setzero_si256();
                }

            __m256i a, b, c, c_sr, mask_imag, mask_real, real, imag;

            mask_imag = _mm256_set_epi8(0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0);
            mask_real = _mm256_set_epi8(0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF);

            for (index = 0; index < sse_iters; index++)
                {
                    b = _mm256_load_si256((__m256i*)_in_common);
                    __VOLK_GNSSSDR_PREFETCH(_in_common + 16);
                    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            a = _mm256_load_si256((__m256i*)&(_in_a[n_vec][index * 8]));

                            c = _mm256_mullo_epi16(a, b);

                            c_sr = _mm256_srli_si256(c, 2);  // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
                            real = _mm256_subs_epi16(c, c_sr);

                            c_sr = _mm256_slli_si256(b, 2);   // b3.r, b2.i ....
                            c = _mm256_mullo_epi16(a, c_sr);  // a3.i*b3.r, ....

                            c_sr = _mm256_slli_si256(a, 2);      // a3.r, a2.i ....
                            imag = _mm256_mullo_epi16(b, c_sr);  // b3.i*a3.r, ....

                            imag = _mm256_adds_epi16(c, imag);

                            realcacc[n_vec] = _mm256_adds_epi16(realcacc[n_vec], real);
                            imagcacc[n_vec] = _mm256_adds_epi16(imagcacc[n_vec], imag);
                        }
                    _in_common += 8;
                }

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    realcacc[n_vec] = _mm256_and_si256(realcacc[n_vec], mask_real);
                    imagcacc[n_vec] = _mm256_and_si256(imagcacc[n_vec], mask_imag);

                    a = _mm256_or_si256(realcacc[n_vec], imagcacc[n_vec]);

                    _mm256_store_si256((__m256i*)dotProductVector, a);  // Store the results back into the dot product vector
                    dotProduct = lv_cmake(0, 0);
                    for (index = 0; index < 8; ++index)
                        {
                            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[index])),
                                sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[index])));
                        }
                    _out[n_vec] = dotProduct;
                }
            volk_gnsssdr_free(realcacc);
            volk_gnsssdr_free(imagcacc);
        }
    _mm256_zeroupper();

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            for (index = sse_iters * 8; index < num_points; index++)
                {
                    lv_16sc_t tmp = in_common[index] * in_a[n_vec][index];

                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_AVX2 */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_u_avx2(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_16sc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake(0, 0);

    const unsigned int sse_iters = num_points / 8;
    int n_vec;
    unsigned int index;
    const lv_16sc_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;

    if (sse_iters > 0)
        {
            __VOLK_ATTR_ALIGNED(32)
            lv_16sc_t dotProductVector[8];

            __m256i* realcacc = (__m256i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m256i), volk_gnsssdr_get_alignment());
            __m256i* imagcacc = (__m256i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m256i), volk_gnsssdr_get_alignment());

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    realcacc[n_vec] = _mm256_setzero_si256();
                    imagcacc[n_vec] = _mm256_setzero_si256();
                }

            __m256i a, b, c, c_sr, mask_imag, mask_real, real, imag;

            mask_imag = _mm256_set_epi8(0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0);
            mask_real = _mm256_set_epi8(0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF, 0, 0, 0xFF, 0xFF);

            for (index = 0; index < sse_iters; index++)
                {
                    b = _mm256_loadu_si256((__m256i*)_in_common);
                    __VOLK_GNSSSDR_PREFETCH(_in_common + 16);
                    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            a = _mm256_loadu_si256((__m256i*)&(_in_a[n_vec][index * 8]));

                            c = _mm256_mullo_epi16(a, b);

                            c_sr = _mm256_srli_si256(c, 2);  // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
                            real = _mm256_subs_epi16(c, c_sr);

                            c_sr = _mm256_slli_si256(b, 2);   // b3.r, b2.i ....
                            c = _mm256_mullo_epi16(a, c_sr);  // a3.i*b3.r, ....

                            c_sr = _mm256_slli_si256(a, 2);      // a3.r, a2.i ....
                            imag = _mm256_mullo_epi16(b, c_sr);  // b3.i*a3.r, ....

                            imag = _mm256_adds_epi16(c, imag);

                            realcacc[n_vec] = _mm256_adds_epi16(realcacc[n_vec], real);
                            imagcacc[n_vec] = _mm256_adds_epi16(imagcacc[n_vec], imag);
                        }
                    _in_common += 8;
                }

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    realcacc[n_vec] = _mm256_and_si256(realcacc[n_vec], mask_real);
                    imagcacc[n_vec] = _mm256_and_si256(imagcacc[n_vec], mask_imag);

                    a = _mm256_or_si256(realcacc[n_vec], imagcacc[n_vec]);

                    _mm256_store_si256((__m256i*)dotProductVector, a);  // Store the results back into the dot product vector
                    dotProduct = lv_cmake(0, 0);
                    for (index = 0; index < 8; ++index)
                        {
                            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[index])),
                                sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[index])));
                        }
                    _out[n_vec] = dotProduct;
                }
            volk_gnsssdr_free(realcacc);
            volk_gnsssdr_free(imagcacc);
        }
    _mm256_zeroupper();

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            for (index = sse_iters * 8; index < num_points; index++)
                {
                    lv_16sc_t tmp = in_common[index] * in_a[n_vec][index];

                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_AVX2 */


#ifdef LV_HAVE_NEONV7
#include <arm_neon.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_neon(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_16sc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake(0, 0);
    int n_vec;
    unsigned int index;
    const unsigned int neon_iters = num_points / 4;

    const lv_16sc_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;

    if (neon_iters > 0)
        {
            __VOLK_ATTR_ALIGNED(16)
            lv_16sc_t dotProductVector[4];

            int16x4x2_t a_val, b_val, c_val;

            int16x4x2_t* accumulator = (int16x4x2_t*)volk_gnsssdr_malloc(num_a_vectors * sizeof(int16x4x2_t), volk_gnsssdr_get_alignment());

            int16x4x2_t tmp_real, tmp_imag;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    accumulator[n_vec].val[0] = vdup_n_s16(0);
                    accumulator[n_vec].val[1] = vdup_n_s16(0);
                }

            for (index = 0; index < neon_iters; index++)
                {
                    b_val = vld2_s16((int16_t*)_in_common);  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
                    __VOLK_GNSSSDR_PREFETCH(_in_common + 8);
                    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            a_val = vld2_s16((int16_t*)&(_in_a[n_vec][index * 4]));  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
                            //__VOLK_GNSSSDR_PREFETCH(&_in_a[n_vec][index*4] + 8);

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

                            accumulator[n_vec].val[0] = vqadd_s16(accumulator[n_vec].val[0], c_val.val[0]);
                            accumulator[n_vec].val[1] = vqadd_s16(accumulator[n_vec].val[1], c_val.val[1]);
                        }
                    _in_common += 4;
                }

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    vst2_s16((int16_t*)dotProductVector, accumulator[n_vec]);  // Store the results back into the dot product vector
                    dotProduct = lv_cmake(0, 0);
                    for (index = 0; index < 4; ++index)
                        {
                            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[index])),
                                sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[index])));
                        }
                    _out[n_vec] = dotProduct;
                }
            volk_gnsssdr_free(accumulator);
        }

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            for (index = neon_iters * 4; index < num_points; index++)
                {
                    lv_16sc_t tmp = in_common[index] * in_a[n_vec][index];

                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_NEONV7 */


#ifdef LV_HAVE_NEONV7
#include <arm_neon.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_neon_vma(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_16sc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake(0, 0);

    const unsigned int neon_iters = num_points / 4;
    int n_vec;
    unsigned int index;
    const lv_16sc_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;

    if (neon_iters > 0)
        {
            __VOLK_ATTR_ALIGNED(16)
            lv_16sc_t dotProductVector[4];

            int16x4x2_t a_val, b_val, tmp;

            int16x4x2_t* accumulator = (int16x4x2_t*)volk_gnsssdr_malloc(num_a_vectors * sizeof(int16x4x2_t), volk_gnsssdr_get_alignment());

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    accumulator[n_vec].val[0] = vdup_n_s16(0);
                    accumulator[n_vec].val[1] = vdup_n_s16(0);
                }

            for (index = 0; index < neon_iters; index++)
                {
                    b_val = vld2_s16((int16_t*)_in_common);  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
                    __VOLK_GNSSSDR_PREFETCH(_in_common + 8);
                    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            a_val = vld2_s16((int16_t*)&(_in_a[n_vec][index * 4]));

                            tmp.val[0] = vmul_s16(a_val.val[0], b_val.val[0]);
                            tmp.val[1] = vmul_s16(a_val.val[1], b_val.val[0]);

                            // use multiply accumulate/subtract to get result
                            tmp.val[0] = vmls_s16(tmp.val[0], a_val.val[1], b_val.val[1]);
                            tmp.val[1] = vmla_s16(tmp.val[1], a_val.val[0], b_val.val[1]);

                            accumulator[n_vec].val[0] = vqadd_s16(accumulator[n_vec].val[0], tmp.val[0]);
                            accumulator[n_vec].val[1] = vqadd_s16(accumulator[n_vec].val[1], tmp.val[1]);
                        }
                    _in_common += 4;
                }

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    vst2_s16((int16_t*)dotProductVector, accumulator[n_vec]);  // Store the results back into the dot product vector
                    dotProduct = lv_cmake(0, 0);
                    for (index = 0; index < 4; ++index)
                        {
                            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[index])),
                                sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[index])));
                        }
                    _out[n_vec] = dotProduct;
                }
            volk_gnsssdr_free(accumulator);
        }

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            for (index = neon_iters * 4; index < num_points; index++)
                {
                    lv_16sc_t tmp = in_common[index] * in_a[n_vec][index];

                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_NEONV7 */


#ifdef LV_HAVE_NEONV7
#include <arm_neon.h>

static inline void volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_neon_optvma(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_16sc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake(0, 0);

    const unsigned int neon_iters = num_points / 4;
    int n_vec;
    unsigned int index;
    const lv_16sc_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;

    if (neon_iters > 0)
        {
            __VOLK_ATTR_ALIGNED(16)
            lv_16sc_t dotProductVector[4];

            int16x4x2_t a_val, b_val;

            int16x4x2_t* accumulator1 = (int16x4x2_t*)volk_gnsssdr_malloc(num_a_vectors * sizeof(int16x4x2_t), volk_gnsssdr_get_alignment());
            int16x4x2_t* accumulator2 = (int16x4x2_t*)volk_gnsssdr_malloc(num_a_vectors * sizeof(int16x4x2_t), volk_gnsssdr_get_alignment());

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    accumulator1[n_vec].val[0] = vdup_n_s16(0);
                    accumulator1[n_vec].val[1] = vdup_n_s16(0);
                    accumulator2[n_vec].val[0] = vdup_n_s16(0);
                    accumulator2[n_vec].val[1] = vdup_n_s16(0);
                }

            for (index = 0; index < neon_iters; index++)
                {
                    b_val = vld2_s16((int16_t*)_in_common);  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
                    __VOLK_GNSSSDR_PREFETCH(_in_common + 8);
                    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            a_val = vld2_s16((int16_t*)&(_in_a[n_vec][index * 4]));

                            accumulator1[n_vec].val[0] = vmla_s16(accumulator1[n_vec].val[0], a_val.val[0], b_val.val[0]);
                            accumulator1[n_vec].val[1] = vmla_s16(accumulator1[n_vec].val[1], a_val.val[0], b_val.val[1]);
                            accumulator2[n_vec].val[0] = vmls_s16(accumulator2[n_vec].val[0], a_val.val[1], b_val.val[1]);
                            accumulator2[n_vec].val[1] = vmla_s16(accumulator2[n_vec].val[1], a_val.val[1], b_val.val[0]);
                        }
                    _in_common += 4;
                }

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    accumulator1[n_vec].val[0] = vqadd_s16(accumulator1[n_vec].val[0], accumulator2[n_vec].val[0]);
                    accumulator1[n_vec].val[1] = vqadd_s16(accumulator1[n_vec].val[1], accumulator2[n_vec].val[1]);
                }

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    vst2_s16((int16_t*)dotProductVector, accumulator1[n_vec]);  // Store the results back into the dot product vector
                    dotProduct = lv_cmake(0, 0);
                    for (index = 0; index < 4; ++index)
                        {
                            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[index])),
                                sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[index])));
                        }
                    _out[n_vec] = dotProduct;
                }
            volk_gnsssdr_free(accumulator1);
            volk_gnsssdr_free(accumulator2);
        }

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            for (index = neon_iters * 4; index < num_points; index++)
                {
                    lv_16sc_t tmp = in_common[index] * in_a[n_vec][index];

                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_NEONV7 */

#endif /*INCLUDED_volk_gnsssdr_16ic_xn_dot_prod_16ic_xn_H*/
