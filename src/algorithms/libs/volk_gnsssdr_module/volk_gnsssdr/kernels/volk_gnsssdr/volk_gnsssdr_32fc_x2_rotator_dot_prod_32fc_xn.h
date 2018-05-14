/*!
 * \file volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn.h
 * \brief VOLK_GNSSSDR kernel: multiplies N complex (32-bit float per component) vectors
 * by a common vector, phase rotated and accumulates the results in N float complex outputs.
 * \authors <ul>
 *          <li> Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that multiplies N 32 bits complex vectors by a common vector, which is
 * phase-rotated by phase offset and phase increment, and accumulates the results
 * in N 32 bits float complex outputs.
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
 * \page volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn
 *
 * \b Overview
 *
 * Rotates and multiplies the reference complex vector with an arbitrary number of other complex vectors,
 * accumulates the results and stores them in the output vector.
 * The rotation is done at a fixed rate per sample, from an initial \p phase offset.
 * This function can be used for Doppler wipe-off and multiple correlator.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_32fc_t** in_a, int num_a_vectors, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li in_common:     Pointer to one of the vectors to be rotated, multiplied and accumulated (reference vector).
 * \li phase_inc:     Phase increment = lv_cmake(cos(phase_step_rad), sin(phase_step_rad))
 * \li phase:         Initial phase = lv_cmake(cos(initial_phase_rad), sin(initial_phase_rad))
 * \li in_a:          Pointer to an array of pointers to multiple vectors to be multiplied and accumulated.
 * \li num_a_vectors: Number of vectors to be multiplied by the reference vector and accumulated.
 * \li num_points:    Number of complex values to be multiplied together, accumulated and stored into \p result.
 *
 * \b Outputs
 * \li phase:         Final phase.
 * \li result:        Vector of \p num_a_vectors components with the multiple vectors of \p in_a rotated, multiplied by \p in_common and accumulated.
 *
 */

#ifndef INCLUDED_volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_H
#define INCLUDED_volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_H


#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/saturation_arithmetic.h>
#include <math.h>


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_generic(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_32fc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_32fc_t tmp32_1, tmp32_2;
    int n_vec;
    unsigned int n;
    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            result[n_vec] = lv_cmake(0, 0);
        }
    for (n = 0; n < num_points; n++)
        {
            tmp32_1 = *in_common++ * (*phase);  //if(n<10 || n >= 8108) printf("generic phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));

            // Regenerate phase
            if (n % 256 == 0)
                {
                    //printf("Phase before regeneration %i: %f,%f  Modulus: %f\n", n,lv_creal(*phase),lv_cimag(*phase), cabsf(*phase));
#ifdef __cplusplus
                    (*phase) /= std::abs((*phase));
#else
                    (*phase) /= hypotf(lv_creal(*phase), lv_cimag(*phase));
#endif
                    //printf("Phase after regeneration %i: %f,%f  Modulus: %f\n", n,lv_creal(*phase),lv_cimag(*phase), cabsf(*phase));
                }

            (*phase) *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    tmp32_2 = tmp32_1 * in_a[n_vec][n];
                    result[n_vec] += tmp32_2;
                }
        }
}

#endif /*LV_HAVE_GENERIC*/


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_generic_reload(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_32fc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_32fc_t tmp32_1, tmp32_2;
    const unsigned int ROTATOR_RELOAD = 256;
    int n_vec;
    unsigned int n;
    unsigned int j;
    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            result[n_vec] = lv_cmake(0, 0);
        }

    for (n = 0; n < num_points / ROTATOR_RELOAD; n++)
        {
            for (j = 0; j < ROTATOR_RELOAD; j++)
                {
                    tmp32_1 = *in_common++ * (*phase);
                    (*phase) *= phase_inc;
                    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            tmp32_2 = tmp32_1 * in_a[n_vec][n * ROTATOR_RELOAD + j];
                            result[n_vec] += tmp32_2;
                        }
                }
                /* Regenerate phase */
#ifdef __cplusplus
            (*phase) /= std::abs((*phase));
#else
            //(*phase) /= cabsf((*phase));
            (*phase) /= hypotf(lv_creal(*phase), lv_cimag(*phase));
#endif
        }

    for (j = 0; j < num_points % ROTATOR_RELOAD; j++)
        {
            tmp32_1 = *in_common++ * (*phase);
            (*phase) *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    tmp32_2 = tmp32_1 * in_a[n_vec][(num_points / ROTATOR_RELOAD) * ROTATOR_RELOAD + j];
                    result[n_vec] += tmp32_2;
                }
        }
}

#endif /*LV_HAVE_GENERIC*/


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>
static inline void volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_u_sse3(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_32fc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_32fc_t dotProduct = lv_cmake(0, 0);
    lv_32fc_t tmp32_1, tmp32_2;
    const unsigned int sse_iters = num_points / 2;
    int n_vec;
    int i;
    unsigned int number;
    unsigned int n;
    const lv_32fc_t** _in_a = in_a;
    const lv_32fc_t* _in_common = in_common;

    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t dotProductVector[2];

    __m128* acc = (__m128*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m128), volk_gnsssdr_get_alignment());

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            acc[n_vec] = _mm_setzero_ps();
        }

    // phase rotation registers
    __m128 a, two_phase_acc_reg, two_phase_inc_reg, yl, yh, tmp1, tmp1p, tmp2, tmp2p, z1;

    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_inc[2];
    two_phase_inc[0] = phase_inc * phase_inc;
    two_phase_inc[1] = phase_inc * phase_inc;
    two_phase_inc_reg = _mm_load_ps((float*)two_phase_inc);
    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_acc[2];
    two_phase_acc[0] = (*phase);
    two_phase_acc[1] = (*phase) * phase_inc;
    two_phase_acc_reg = _mm_load_ps((float*)two_phase_acc);

    const __m128 ylp = _mm_moveldup_ps(two_phase_inc_reg);
    const __m128 yhp = _mm_movehdup_ps(two_phase_inc_reg);

    for (number = 0; number < sse_iters; number++)
        {
            // Phase rotation on operand in_common starts here:
            a = _mm_loadu_ps((float*)_in_common);
            // __VOLK_GNSSSDR_PREFETCH(_in_common + 4);
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);
            tmp1 = _mm_mul_ps(a, yl);
            tmp1p = _mm_mul_ps(two_phase_acc_reg, ylp);
            a = _mm_shuffle_ps(a, a, 0xB1);
            two_phase_acc_reg = _mm_shuffle_ps(two_phase_acc_reg, two_phase_acc_reg, 0xB1);
            tmp2 = _mm_mul_ps(a, yh);
            tmp2p = _mm_mul_ps(two_phase_acc_reg, yhp);
            z1 = _mm_addsub_ps(tmp1, tmp2);
            two_phase_acc_reg = _mm_addsub_ps(tmp1p, tmp2p);

            yl = _mm_moveldup_ps(z1);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(z1);

            //next two samples
            _in_common += 2;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    a = _mm_loadu_ps((float*)&(_in_a[n_vec][number * 2]));
                    tmp1 = _mm_mul_ps(a, yl);
                    a = _mm_shuffle_ps(a, a, 0xB1);
                    tmp2 = _mm_mul_ps(a, yh);
                    z1 = _mm_addsub_ps(tmp1, tmp2);
                    acc[n_vec] = _mm_add_ps(acc[n_vec], z1);
                }
            // Regenerate phase
            if ((number % 128) == 0)
                {
                    tmp1 = _mm_mul_ps(two_phase_acc_reg, two_phase_acc_reg);
                    tmp2 = _mm_hadd_ps(tmp1, tmp1);
                    tmp1 = _mm_shuffle_ps(tmp2, tmp2, 0xD8);
                    tmp2 = _mm_sqrt_ps(tmp1);
                    two_phase_acc_reg = _mm_div_ps(two_phase_acc_reg, tmp2);
                }
        }

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            _mm_store_ps((float*)dotProductVector, acc[n_vec]);  // Store the results back into the dot product vector
            dotProduct = lv_cmake(0, 0);
            for (i = 0; i < 2; ++i)
                {
                    dotProduct = dotProduct + dotProductVector[i];
                }
            result[n_vec] = dotProduct;
        }
    volk_gnsssdr_free(acc);

    _mm_store_ps((float*)two_phase_acc, two_phase_acc_reg);
    (*phase) = two_phase_acc[0];

    for (n = sse_iters * 2; n < num_points; n++)
        {
            tmp32_1 = in_common[n] * (*phase);
            (*phase) *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    tmp32_2 = tmp32_1 * in_a[n_vec][n];
                    result[n_vec] += tmp32_2;
                }
        }
}
#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>
static inline void volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_a_sse3(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_32fc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_32fc_t dotProduct = lv_cmake(0, 0);
    lv_32fc_t tmp32_1, tmp32_2;
    const unsigned int sse_iters = num_points / 2;
    int n_vec;
    int i;
    unsigned int n;
    unsigned int number;
    const lv_32fc_t** _in_a = in_a;
    const lv_32fc_t* _in_common = in_common;

    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t dotProductVector[2];

    __m128* acc = (__m128*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m128), volk_gnsssdr_get_alignment());

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            acc[n_vec] = _mm_setzero_ps();
        }

    // phase rotation registers
    __m128 a, two_phase_acc_reg, two_phase_inc_reg, yl, yh, tmp1, tmp1p, tmp2, tmp2p, z1;

    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_inc[2];
    two_phase_inc[0] = phase_inc * phase_inc;
    two_phase_inc[1] = phase_inc * phase_inc;
    two_phase_inc_reg = _mm_load_ps((float*)two_phase_inc);
    __VOLK_ATTR_ALIGNED(16)
    lv_32fc_t two_phase_acc[2];
    two_phase_acc[0] = (*phase);
    two_phase_acc[1] = (*phase) * phase_inc;
    two_phase_acc_reg = _mm_load_ps((float*)two_phase_acc);

    const __m128 ylp = _mm_moveldup_ps(two_phase_inc_reg);
    const __m128 yhp = _mm_movehdup_ps(two_phase_inc_reg);

    for (number = 0; number < sse_iters; number++)
        {
            // Phase rotation on operand in_common starts here:
            a = _mm_load_ps((float*)_in_common);
            // __VOLK_GNSSSDR_PREFETCH(_in_common + 4);
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);
            tmp1 = _mm_mul_ps(a, yl);
            tmp1p = _mm_mul_ps(two_phase_acc_reg, ylp);
            a = _mm_shuffle_ps(a, a, 0xB1);
            two_phase_acc_reg = _mm_shuffle_ps(two_phase_acc_reg, two_phase_acc_reg, 0xB1);
            tmp2 = _mm_mul_ps(a, yh);
            tmp2p = _mm_mul_ps(two_phase_acc_reg, yhp);
            z1 = _mm_addsub_ps(tmp1, tmp2);
            two_phase_acc_reg = _mm_addsub_ps(tmp1p, tmp2p);

            yl = _mm_moveldup_ps(z1);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(z1);

            //next two samples
            _in_common += 2;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    a = _mm_load_ps((float*)&(_in_a[n_vec][number * 2]));
                    tmp1 = _mm_mul_ps(a, yl);
                    a = _mm_shuffle_ps(a, a, 0xB1);
                    tmp2 = _mm_mul_ps(a, yh);
                    z1 = _mm_addsub_ps(tmp1, tmp2);
                    acc[n_vec] = _mm_add_ps(acc[n_vec], z1);
                }
            // Regenerate phase
            if ((number % 128) == 0)
                {
                    tmp1 = _mm_mul_ps(two_phase_acc_reg, two_phase_acc_reg);
                    tmp2 = _mm_hadd_ps(tmp1, tmp1);
                    tmp1 = _mm_shuffle_ps(tmp2, tmp2, 0xD8);
                    tmp2 = _mm_sqrt_ps(tmp1);
                    two_phase_acc_reg = _mm_div_ps(two_phase_acc_reg, tmp2);
                }
        }

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            _mm_store_ps((float*)dotProductVector, acc[n_vec]);  // Store the results back into the dot product vector
            dotProduct = lv_cmake(0, 0);
            for (i = 0; i < 2; ++i)
                {
                    dotProduct = dotProduct + dotProductVector[i];
                }
            result[n_vec] = dotProduct;
        }
    volk_gnsssdr_free(acc);

    _mm_store_ps((float*)two_phase_acc, two_phase_acc_reg);
    (*phase) = two_phase_acc[0];

    for (n = sse_iters * 2; n < num_points; n++)
        {
            tmp32_1 = in_common[n] * (*phase);
            (*phase) *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    tmp32_2 = tmp32_1 * in_a[n_vec][n];
                    result[n_vec] += tmp32_2;
                }
        }
}
#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_AVX
#include <immintrin.h>
static inline void volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_u_avx(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_32fc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_32fc_t dotProduct = lv_cmake(0, 0);
    lv_32fc_t tmp32_1, tmp32_2;
    const unsigned int avx_iters = num_points / 4;
    int n_vec;
    int i;
    unsigned int number;
    unsigned int n;
    const lv_32fc_t** _in_a = in_a;
    const lv_32fc_t* _in_common = in_common;
    lv_32fc_t _phase = (*phase);

    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t dotProductVector[4];

    __m256* acc = (__m256*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m256), volk_gnsssdr_get_alignment());

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            acc[n_vec] = _mm256_setzero_ps();
            result[n_vec] = lv_cmake(0, 0);
        }

    // phase rotation registers
    __m256 a, four_phase_acc_reg, yl, yh, tmp1, tmp1p, tmp2, tmp2p, z;

    __attribute__((aligned(32))) lv_32fc_t four_phase_inc[4];
    const lv_32fc_t phase_inc2 = phase_inc * phase_inc;
    const lv_32fc_t phase_inc3 = phase_inc2 * phase_inc;
    const lv_32fc_t phase_inc4 = phase_inc3 * phase_inc;
    four_phase_inc[0] = phase_inc4;
    four_phase_inc[1] = phase_inc4;
    four_phase_inc[2] = phase_inc4;
    four_phase_inc[3] = phase_inc4;
    const __m256 four_phase_inc_reg = _mm256_load_ps((float*)four_phase_inc);

    __attribute__((aligned(32))) lv_32fc_t four_phase_acc[4];
    four_phase_acc[0] = _phase;
    four_phase_acc[1] = _phase * phase_inc;
    four_phase_acc[2] = _phase * phase_inc2;
    four_phase_acc[3] = _phase * phase_inc3;
    four_phase_acc_reg = _mm256_load_ps((float*)four_phase_acc);

    const __m256 ylp = _mm256_moveldup_ps(four_phase_inc_reg);
    const __m256 yhp = _mm256_movehdup_ps(four_phase_inc_reg);

    for (number = 0; number < avx_iters; number++)
        {
            // Phase rotation on operand in_common starts here:
            a = _mm256_loadu_ps((float*)_in_common);
            __VOLK_GNSSSDR_PREFETCH(_in_common + 16);
            yl = _mm256_moveldup_ps(four_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm256_movehdup_ps(four_phase_acc_reg);
            tmp1 = _mm256_mul_ps(a, yl);
            tmp1p = _mm256_mul_ps(four_phase_acc_reg, ylp);
            a = _mm256_shuffle_ps(a, a, 0xB1);
            four_phase_acc_reg = _mm256_shuffle_ps(four_phase_acc_reg, four_phase_acc_reg, 0xB1);
            tmp2 = _mm256_mul_ps(a, yh);
            tmp2p = _mm256_mul_ps(four_phase_acc_reg, yhp);
            z = _mm256_addsub_ps(tmp1, tmp2);
            four_phase_acc_reg = _mm256_addsub_ps(tmp1p, tmp2p);

            yl = _mm256_moveldup_ps(z);  // Load yl with cr,cr,dr,dr
            yh = _mm256_movehdup_ps(z);

            //next two samples
            _in_common += 4;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    a = _mm256_loadu_ps((float*)&(_in_a[n_vec][number * 4]));
                    tmp1 = _mm256_mul_ps(a, yl);
                    a = _mm256_shuffle_ps(a, a, 0xB1);
                    tmp2 = _mm256_mul_ps(a, yh);
                    z = _mm256_addsub_ps(tmp1, tmp2);
                    acc[n_vec] = _mm256_add_ps(acc[n_vec], z);
                }
            // Regenerate phase
            if ((number % 128) == 0)
                {
                    tmp1 = _mm256_mul_ps(four_phase_acc_reg, four_phase_acc_reg);
                    tmp2 = _mm256_hadd_ps(tmp1, tmp1);
                    tmp1 = _mm256_shuffle_ps(tmp2, tmp2, 0xD8);
                    tmp2 = _mm256_sqrt_ps(tmp1);
                    four_phase_acc_reg = _mm256_div_ps(four_phase_acc_reg, tmp2);
                }
        }

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            _mm256_store_ps((float*)dotProductVector, acc[n_vec]);  // Store the results back into the dot product vector
            dotProduct = lv_cmake(0, 0);
            for (i = 0; i < 4; ++i)
                {
                    dotProduct = dotProduct + dotProductVector[i];
                }
            result[n_vec] = dotProduct;
        }
    volk_gnsssdr_free(acc);

    tmp1 = _mm256_mul_ps(four_phase_acc_reg, four_phase_acc_reg);
    tmp2 = _mm256_hadd_ps(tmp1, tmp1);
    tmp1 = _mm256_shuffle_ps(tmp2, tmp2, 0xD8);
    tmp2 = _mm256_sqrt_ps(tmp1);
    four_phase_acc_reg = _mm256_div_ps(four_phase_acc_reg, tmp2);

    _mm256_store_ps((float*)four_phase_acc, four_phase_acc_reg);
    _phase = four_phase_acc[0];
    _mm256_zeroupper();

    for (n = avx_iters * 4; n < num_points; n++)
        {
            tmp32_1 = *_in_common++ * _phase;
            _phase *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    tmp32_2 = tmp32_1 * _in_a[n_vec][n];
                    result[n_vec] += tmp32_2;
                }
        }
    (*phase) = _phase;
}
#endif /* LV_HAVE_AVX */


#ifdef LV_HAVE_AVX
#include <immintrin.h>
static inline void volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_a_avx(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_32fc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_32fc_t dotProduct = lv_cmake(0, 0);
    lv_32fc_t tmp32_1, tmp32_2;
    const unsigned int avx_iters = num_points / 4;
    int n_vec;
    int i;
    unsigned int number;
    unsigned int n;
    const lv_32fc_t** _in_a = in_a;
    const lv_32fc_t* _in_common = in_common;
    lv_32fc_t _phase = (*phase);

    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t dotProductVector[4];

    __m256* acc = (__m256*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m256), volk_gnsssdr_get_alignment());

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            acc[n_vec] = _mm256_setzero_ps();
            result[n_vec] = lv_cmake(0, 0);
        }

    // phase rotation registers
    __m256 a, four_phase_acc_reg, yl, yh, tmp1, tmp1p, tmp2, tmp2p, z;

    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t four_phase_inc[4];
    const lv_32fc_t phase_inc2 = phase_inc * phase_inc;
    const lv_32fc_t phase_inc3 = phase_inc2 * phase_inc;
    const lv_32fc_t phase_inc4 = phase_inc3 * phase_inc;
    four_phase_inc[0] = phase_inc4;
    four_phase_inc[1] = phase_inc4;
    four_phase_inc[2] = phase_inc4;
    four_phase_inc[3] = phase_inc4;
    const __m256 four_phase_inc_reg = _mm256_load_ps((float*)four_phase_inc);

    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t four_phase_acc[4];
    four_phase_acc[0] = _phase;
    four_phase_acc[1] = _phase * phase_inc;
    four_phase_acc[2] = _phase * phase_inc2;
    four_phase_acc[3] = _phase * phase_inc3;
    four_phase_acc_reg = _mm256_load_ps((float*)four_phase_acc);

    const __m256 ylp = _mm256_moveldup_ps(four_phase_inc_reg);
    const __m256 yhp = _mm256_movehdup_ps(four_phase_inc_reg);

    for (number = 0; number < avx_iters; number++)
        {
            // Phase rotation on operand in_common starts here:
            a = _mm256_load_ps((float*)_in_common);
            __VOLK_GNSSSDR_PREFETCH(_in_common + 16);
            yl = _mm256_moveldup_ps(four_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm256_movehdup_ps(four_phase_acc_reg);
            tmp1 = _mm256_mul_ps(a, yl);
            tmp1p = _mm256_mul_ps(four_phase_acc_reg, ylp);
            a = _mm256_shuffle_ps(a, a, 0xB1);
            four_phase_acc_reg = _mm256_shuffle_ps(four_phase_acc_reg, four_phase_acc_reg, 0xB1);
            tmp2 = _mm256_mul_ps(a, yh);
            tmp2p = _mm256_mul_ps(four_phase_acc_reg, yhp);
            z = _mm256_addsub_ps(tmp1, tmp2);
            four_phase_acc_reg = _mm256_addsub_ps(tmp1p, tmp2p);

            yl = _mm256_moveldup_ps(z);  // Load yl with cr,cr,dr,dr
            yh = _mm256_movehdup_ps(z);

            //next two samples
            _in_common += 4;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    a = _mm256_load_ps((float*)&(_in_a[n_vec][number * 4]));
                    tmp1 = _mm256_mul_ps(a, yl);
                    a = _mm256_shuffle_ps(a, a, 0xB1);
                    tmp2 = _mm256_mul_ps(a, yh);
                    z = _mm256_addsub_ps(tmp1, tmp2);
                    acc[n_vec] = _mm256_add_ps(acc[n_vec], z);
                }
            // Regenerate phase
            if ((number % 128) == 0)
                {
                    tmp1 = _mm256_mul_ps(four_phase_acc_reg, four_phase_acc_reg);
                    tmp2 = _mm256_hadd_ps(tmp1, tmp1);
                    tmp1 = _mm256_shuffle_ps(tmp2, tmp2, 0xD8);
                    tmp2 = _mm256_sqrt_ps(tmp1);
                    four_phase_acc_reg = _mm256_div_ps(four_phase_acc_reg, tmp2);
                }
        }

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            _mm256_store_ps((float*)dotProductVector, acc[n_vec]);  // Store the results back into the dot product vector
            dotProduct = lv_cmake(0, 0);
            for (i = 0; i < 4; ++i)
                {
                    dotProduct = dotProduct + dotProductVector[i];
                }
            result[n_vec] = dotProduct;
        }
    volk_gnsssdr_free(acc);

    tmp1 = _mm256_mul_ps(four_phase_acc_reg, four_phase_acc_reg);
    tmp2 = _mm256_hadd_ps(tmp1, tmp1);
    tmp1 = _mm256_shuffle_ps(tmp2, tmp2, 0xD8);
    tmp2 = _mm256_sqrt_ps(tmp1);
    four_phase_acc_reg = _mm256_div_ps(four_phase_acc_reg, tmp2);

    _mm256_store_ps((float*)four_phase_acc, four_phase_acc_reg);
    _phase = four_phase_acc[0];
    _mm256_zeroupper();

    for (n = avx_iters * 4; n < num_points; n++)
        {
            tmp32_1 = *_in_common++ * _phase;
            _phase *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    tmp32_2 = tmp32_1 * _in_a[n_vec][n];
                    result[n_vec] += tmp32_2;
                }
        }
    (*phase) = _phase;
}
#endif /* LV_HAVE_AVX */


#ifdef LV_HAVE_NEONV7
#include <arm_neon.h>

static inline void volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_neon(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_32fc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    const unsigned int neon_iters = num_points / 4;
    int n_vec;
    int i;
    unsigned int number;
    unsigned int n;
    const lv_32fc_t** _in_a = in_a;
    const lv_32fc_t* _in_common = in_common;
    lv_32fc_t* _out = result;

    lv_32fc_t _phase = (*phase);
    lv_32fc_t tmp32_1, tmp32_2;

    if (neon_iters > 0)
        {
            lv_32fc_t dotProduct = lv_cmake(0, 0);
            float32_t arg_phase0 = cargf(_phase);
            float32_t arg_phase_inc = cargf(phase_inc);
            float32_t phase_est;

            lv_32fc_t ___phase4 = phase_inc * phase_inc * phase_inc * phase_inc;
            __VOLK_ATTR_ALIGNED(16)
            float32_t __phase4_real[4] = {lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4)};
            __VOLK_ATTR_ALIGNED(16)
            float32_t __phase4_imag[4] = {lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4)};

            float32x4_t _phase4_real = vld1q_f32(__phase4_real);
            float32x4_t _phase4_imag = vld1q_f32(__phase4_imag);

            lv_32fc_t phase2 = (lv_32fc_t)(_phase)*phase_inc;
            lv_32fc_t phase3 = phase2 * phase_inc;
            lv_32fc_t phase4 = phase3 * phase_inc;

            __VOLK_ATTR_ALIGNED(16)
            float32_t __phase_real[4] = {lv_creal((_phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4)};
            __VOLK_ATTR_ALIGNED(16)
            float32_t __phase_imag[4] = {lv_cimag((_phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4)};

            float32x4_t _phase_real = vld1q_f32(__phase_real);
            float32x4_t _phase_imag = vld1q_f32(__phase_imag);

            __VOLK_ATTR_ALIGNED(32)
            lv_32fc_t dotProductVector[4];

            float32x4x2_t a_val, b_val, tmp32_real, tmp32_imag;

            float32x4x2_t* accumulator1 = (float32x4x2_t*)volk_gnsssdr_malloc(num_a_vectors * sizeof(float32x4x2_t), volk_gnsssdr_get_alignment());
            float32x4x2_t* accumulator2 = (float32x4x2_t*)volk_gnsssdr_malloc(num_a_vectors * sizeof(float32x4x2_t), volk_gnsssdr_get_alignment());

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    accumulator1[n_vec].val[0] = vdupq_n_f32(0.0f);
                    accumulator1[n_vec].val[1] = vdupq_n_f32(0.0f);
                    accumulator2[n_vec].val[0] = vdupq_n_f32(0.0f);
                    accumulator2[n_vec].val[1] = vdupq_n_f32(0.0f);
                }

            for (number = 0; number < neon_iters; number++)
                {
                    /* load 4 complex numbers (float 32 bits each component) */
                    b_val = vld2q_f32((float32_t*)_in_common);
                    __VOLK_GNSSSDR_PREFETCH(_in_common + 8);
                    _in_common += 4;

                    /* complex multiplication of four complex samples (float 32 bits each component) */
                    tmp32_real.val[0] = vmulq_f32(b_val.val[0], _phase_real);
                    tmp32_real.val[1] = vmulq_f32(b_val.val[1], _phase_imag);
                    tmp32_imag.val[0] = vmulq_f32(b_val.val[0], _phase_imag);
                    tmp32_imag.val[1] = vmulq_f32(b_val.val[1], _phase_real);

                    b_val.val[0] = vsubq_f32(tmp32_real.val[0], tmp32_real.val[1]);
                    b_val.val[1] = vaddq_f32(tmp32_imag.val[0], tmp32_imag.val[1]);

                    /* compute next four phases */
                    tmp32_real.val[0] = vmulq_f32(_phase_real, _phase4_real);
                    tmp32_real.val[1] = vmulq_f32(_phase_imag, _phase4_imag);
                    tmp32_imag.val[0] = vmulq_f32(_phase_real, _phase4_imag);
                    tmp32_imag.val[1] = vmulq_f32(_phase_imag, _phase4_real);

                    _phase_real = vsubq_f32(tmp32_real.val[0], tmp32_real.val[1]);
                    _phase_imag = vaddq_f32(tmp32_imag.val[0], tmp32_imag.val[1]);

                    // Regenerate phase
                    if ((number % 128) == 0)
                        {
                            phase_est = arg_phase0 + (number + 1) * 4 * arg_phase_inc;

                            _phase = lv_cmake(cos(phase_est), sin(phase_est));
                            phase2 = _phase * phase_inc;
                            phase3 = phase2 * phase_inc;
                            phase4 = phase3 * phase_inc;

                            __VOLK_ATTR_ALIGNED(16)
                            float32_t ____phase_real[4] = {lv_creal((_phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4)};
                            __VOLK_ATTR_ALIGNED(16)
                            float32_t ____phase_imag[4] = {lv_cimag((_phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4)};

                            _phase_real = vld1q_f32(____phase_real);
                            _phase_imag = vld1q_f32(____phase_imag);
                        }

                    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            a_val = vld2q_f32((float32_t*)&(_in_a[n_vec][number * 4]));

                            // use 2 accumulators to remove inter-instruction data dependencies
                            accumulator1[n_vec].val[0] = vmlaq_f32(accumulator1[n_vec].val[0], a_val.val[0], b_val.val[0]);
                            accumulator2[n_vec].val[0] = vmlsq_f32(accumulator2[n_vec].val[0], a_val.val[1], b_val.val[1]);
                            accumulator1[n_vec].val[1] = vmlaq_f32(accumulator1[n_vec].val[1], a_val.val[0], b_val.val[1]);
                            accumulator2[n_vec].val[1] = vmlaq_f32(accumulator2[n_vec].val[1], a_val.val[1], b_val.val[0]);
                        }
                }
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    accumulator1[n_vec].val[0] = vaddq_f32(accumulator1[n_vec].val[0], accumulator2[n_vec].val[0]);
                    accumulator1[n_vec].val[1] = vaddq_f32(accumulator1[n_vec].val[1], accumulator2[n_vec].val[1]);
                }
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    vst2q_f32((float32_t*)dotProductVector, accumulator1[n_vec]);  // Store the results back into the dot product vector
                    dotProduct = lv_cmake(0, 0);
                    for (i = 0; i < 4; ++i)
                        {
                            dotProduct = dotProduct + dotProductVector[i];
                        }
                    _out[n_vec] = dotProduct;
                }
            volk_gnsssdr_free(accumulator1);
            volk_gnsssdr_free(accumulator2);

            vst1q_f32((float32_t*)__phase_real, _phase_real);
            vst1q_f32((float32_t*)__phase_imag, _phase_imag);

            _phase = lv_cmake((float32_t)__phase_real[0], (float32_t)__phase_imag[0]);
        }

    for (n = neon_iters * 4; n < num_points; n++)
        {
            tmp32_1 = in_common[n] * _phase;
            _phase *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    tmp32_2 = tmp32_1 * in_a[n_vec][n];
                    _out[n_vec] += tmp32_2;
                }
        }
    (*phase) = _phase;
}

#endif /* LV_HAVE_NEONV7 */

#endif /* INCLUDED_volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_H */
