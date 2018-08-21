/*!
 * \file volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn.h
 * \brief VOLK_GNSSSDR kernel: multiplies N complex (32-bit float per component) vectors
 * by a common vector, phase rotated and accumulates the results in N float complex outputs.
 * \authors <ul>
 *          <li> Cillian O'Driscoll 2016. cillian.odriscoll(at)gmail.com
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
 * \page volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn
 *
 * \b Overview
 *
 * Rotates and multiplies the reference complex vector with an arbitrary number of other real vectors,
 * accumulates the results and stores them in the output vector.
 * The rotation is done at a fixed rate per sample, from an initial \p phase offset.
 * This function can be used for Doppler wipe-off and multiple correlator.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const float** in_a, int num_a_vectors, unsigned int num_points);
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

#ifndef INCLUDED_volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn_H
#define INCLUDED_volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn_H


#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/saturation_arithmetic.h>
#include <math.h>
//#include <stdio.h>

#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn_generic(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const float** in_a, int num_a_vectors, unsigned int num_points)
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

static inline void volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn_generic_reload(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const float** in_a, int num_a_vectors, unsigned int num_points)
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

#ifdef LV_HAVE_AVX
#include <immintrin.h>
#include <volk_gnsssdr/volk_gnsssdr_avx_intrinsics.h>
static inline void volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn_u_avx(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const float** in_a, int num_a_vectors, unsigned int num_points)
{
    unsigned int number = 0;
    int vec_ind = 0;
    unsigned int i = 0;
    const unsigned int sixteenthPoints = num_points / 16;

    const float* aPtr = (float*)in_common;
    const float* bPtr[num_a_vectors];
    for (vec_ind = 0; vec_ind < num_a_vectors; ++vec_ind)
        {
            bPtr[vec_ind] = in_a[vec_ind];
        }

    lv_32fc_t _phase = (*phase);
    lv_32fc_t wo;

    __m256 a0Val, a1Val, a2Val, a3Val;
    __m256 b0Val[num_a_vectors], b1Val[num_a_vectors], b2Val[num_a_vectors], b3Val[num_a_vectors];
    __m256 x0Val[num_a_vectors], x1Val[num_a_vectors], x0loVal[num_a_vectors], x0hiVal[num_a_vectors], x1loVal[num_a_vectors], x1hiVal[num_a_vectors];
    __m256 c0Val[num_a_vectors], c1Val[num_a_vectors], c2Val[num_a_vectors], c3Val[num_a_vectors];

    __m256 dotProdVal0[num_a_vectors];
    __m256 dotProdVal1[num_a_vectors];
    __m256 dotProdVal2[num_a_vectors];
    __m256 dotProdVal3[num_a_vectors];

    for (vec_ind = 0; vec_ind < num_a_vectors; vec_ind++)
        {
            dotProdVal0[vec_ind] = _mm256_setzero_ps();
            dotProdVal1[vec_ind] = _mm256_setzero_ps();
            dotProdVal2[vec_ind] = _mm256_setzero_ps();
            dotProdVal3[vec_ind] = _mm256_setzero_ps();
        }

    // Set up the complex rotator
    __m256 z0, z1, z2, z3;
    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t phase_vec[16];
    for (vec_ind = 0; vec_ind < 16; ++vec_ind)
        {
            phase_vec[vec_ind] = _phase;
            _phase *= phase_inc;
        }

    z0 = _mm256_load_ps((float*)phase_vec);
    z1 = _mm256_load_ps((float*)(phase_vec + 4));
    z2 = _mm256_load_ps((float*)(phase_vec + 8));
    z3 = _mm256_load_ps((float*)(phase_vec + 12));

    lv_32fc_t dz = phase_inc;
    dz *= dz;
    dz *= dz;
    dz *= dz;
    dz *= dz;  // dz = phase_inc^16;

    for (vec_ind = 0; vec_ind < 4; ++vec_ind)
        {
            phase_vec[vec_ind] = dz;
        }

    __m256 dz_reg = _mm256_load_ps((float*)phase_vec);
    dz_reg = _mm256_complexnormalise_ps(dz_reg);

    for (; number < sixteenthPoints; number++)
        {
            a0Val = _mm256_loadu_ps(aPtr);
            a1Val = _mm256_loadu_ps(aPtr + 8);
            a2Val = _mm256_loadu_ps(aPtr + 16);
            a3Val = _mm256_loadu_ps(aPtr + 24);

            a0Val = _mm256_complexmul_ps(a0Val, z0);
            a1Val = _mm256_complexmul_ps(a1Val, z1);
            a2Val = _mm256_complexmul_ps(a2Val, z2);
            a3Val = _mm256_complexmul_ps(a3Val, z3);

            z0 = _mm256_complexmul_ps(z0, dz_reg);
            z1 = _mm256_complexmul_ps(z1, dz_reg);
            z2 = _mm256_complexmul_ps(z2, dz_reg);
            z3 = _mm256_complexmul_ps(z3, dz_reg);

            for (vec_ind = 0; vec_ind < num_a_vectors; ++vec_ind)
                {
                    x0Val[vec_ind] = _mm256_loadu_ps(bPtr[vec_ind]);  // t0|t1|t2|t3|t4|t5|t6|t7
                    x1Val[vec_ind] = _mm256_loadu_ps(bPtr[vec_ind] + 8);
                    x0loVal[vec_ind] = _mm256_unpacklo_ps(x0Val[vec_ind], x0Val[vec_ind]);  // t0|t0|t1|t1|t4|t4|t5|t5
                    x0hiVal[vec_ind] = _mm256_unpackhi_ps(x0Val[vec_ind], x0Val[vec_ind]);  // t2|t2|t3|t3|t6|t6|t7|t7
                    x1loVal[vec_ind] = _mm256_unpacklo_ps(x1Val[vec_ind], x1Val[vec_ind]);
                    x1hiVal[vec_ind] = _mm256_unpackhi_ps(x1Val[vec_ind], x1Val[vec_ind]);

                    // TODO: it may be possible to rearrange swizzling to better pipeline data
                    b0Val[vec_ind] = _mm256_permute2f128_ps(x0loVal[vec_ind], x0hiVal[vec_ind], 0x20);  // t0|t0|t1|t1|t2|t2|t3|t3
                    b1Val[vec_ind] = _mm256_permute2f128_ps(x0loVal[vec_ind], x0hiVal[vec_ind], 0x31);  // t4|t4|t5|t5|t6|t6|t7|t7
                    b2Val[vec_ind] = _mm256_permute2f128_ps(x1loVal[vec_ind], x1hiVal[vec_ind], 0x20);
                    b3Val[vec_ind] = _mm256_permute2f128_ps(x1loVal[vec_ind], x1hiVal[vec_ind], 0x31);

                    c0Val[vec_ind] = _mm256_mul_ps(a0Val, b0Val[vec_ind]);
                    c1Val[vec_ind] = _mm256_mul_ps(a1Val, b1Val[vec_ind]);
                    c2Val[vec_ind] = _mm256_mul_ps(a2Val, b2Val[vec_ind]);
                    c3Val[vec_ind] = _mm256_mul_ps(a3Val, b3Val[vec_ind]);

                    dotProdVal0[vec_ind] = _mm256_add_ps(c0Val[vec_ind], dotProdVal0[vec_ind]);
                    dotProdVal1[vec_ind] = _mm256_add_ps(c1Val[vec_ind], dotProdVal1[vec_ind]);
                    dotProdVal2[vec_ind] = _mm256_add_ps(c2Val[vec_ind], dotProdVal2[vec_ind]);
                    dotProdVal3[vec_ind] = _mm256_add_ps(c3Val[vec_ind], dotProdVal3[vec_ind]);

                    bPtr[vec_ind] += 16;
                }

            // Force the rotators back onto the unit circle
            if ((number % 64) == 0)
                {
                    z0 = _mm256_complexnormalise_ps(z0);
                    z1 = _mm256_complexnormalise_ps(z1);
                    z2 = _mm256_complexnormalise_ps(z2);
                    z3 = _mm256_complexnormalise_ps(z3);
                }

            aPtr += 32;
        }
    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t dotProductVector[4];

    for (vec_ind = 0; vec_ind < num_a_vectors; ++vec_ind)
        {
            dotProdVal0[vec_ind] = _mm256_add_ps(dotProdVal0[vec_ind], dotProdVal1[vec_ind]);
            dotProdVal0[vec_ind] = _mm256_add_ps(dotProdVal0[vec_ind], dotProdVal2[vec_ind]);
            dotProdVal0[vec_ind] = _mm256_add_ps(dotProdVal0[vec_ind], dotProdVal3[vec_ind]);

            _mm256_store_ps((float*)dotProductVector, dotProdVal0[vec_ind]);  // Store the results back into the dot product vector

            result[vec_ind] = lv_cmake(0, 0);
            for (i = 0; i < 4; ++i)
                {
                    result[vec_ind] += dotProductVector[i];
                }
        }

    z0 = _mm256_complexnormalise_ps(z0);
    _mm256_store_ps((float*)phase_vec, z0);
    _phase = phase_vec[0];
    _mm256_zeroupper();

    number = sixteenthPoints * 16;
    for (; number < num_points; number++)
        {
            wo = (*aPtr++) * _phase;
            _phase *= phase_inc;

            for (vec_ind = 0; vec_ind < num_a_vectors; ++vec_ind)
                {
                    result[vec_ind] += wo * in_a[vec_ind][number];
                }
        }

    *phase = _phase;
}

#endif /* LV_HAVE_AVX */


#ifdef LV_HAVE_AVX
#include <immintrin.h>
#include <volk_gnsssdr/volk_gnsssdr_avx_intrinsics.h>
static inline void volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn_a_avx(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const float** in_a, int num_a_vectors, unsigned int num_points)
{
    unsigned int number = 0;
    int vec_ind = 0;
    unsigned int i = 0;
    const unsigned int sixteenthPoints = num_points / 16;

    const float* aPtr = (float*)in_common;
    const float* bPtr[num_a_vectors];
    for (vec_ind = 0; vec_ind < num_a_vectors; ++vec_ind)
        {
            bPtr[vec_ind] = in_a[vec_ind];
        }

    lv_32fc_t _phase = (*phase);
    lv_32fc_t wo;

    __m256 a0Val, a1Val, a2Val, a3Val;
    __m256 b0Val[num_a_vectors], b1Val[num_a_vectors], b2Val[num_a_vectors], b3Val[num_a_vectors];
    __m256 x0Val[num_a_vectors], x1Val[num_a_vectors], x0loVal[num_a_vectors], x0hiVal[num_a_vectors], x1loVal[num_a_vectors], x1hiVal[num_a_vectors];
    __m256 c0Val[num_a_vectors], c1Val[num_a_vectors], c2Val[num_a_vectors], c3Val[num_a_vectors];

    __m256 dotProdVal0[num_a_vectors];
    __m256 dotProdVal1[num_a_vectors];
    __m256 dotProdVal2[num_a_vectors];
    __m256 dotProdVal3[num_a_vectors];

    for (vec_ind = 0; vec_ind < num_a_vectors; vec_ind++)
        {
            dotProdVal0[vec_ind] = _mm256_setzero_ps();
            dotProdVal1[vec_ind] = _mm256_setzero_ps();
            dotProdVal2[vec_ind] = _mm256_setzero_ps();
            dotProdVal3[vec_ind] = _mm256_setzero_ps();
        }

    // Set up the complex rotator
    __m256 z0, z1, z2, z3;
    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t phase_vec[16];
    for (vec_ind = 0; vec_ind < 16; ++vec_ind)
        {
            phase_vec[vec_ind] = _phase;
            _phase *= phase_inc;
        }

    z0 = _mm256_load_ps((float*)phase_vec);
    z1 = _mm256_load_ps((float*)(phase_vec + 4));
    z2 = _mm256_load_ps((float*)(phase_vec + 8));
    z3 = _mm256_load_ps((float*)(phase_vec + 12));

    lv_32fc_t dz = phase_inc;
    dz *= dz;
    dz *= dz;
    dz *= dz;
    dz *= dz;  // dz = phase_inc^16;

    for (vec_ind = 0; vec_ind < 4; ++vec_ind)
        {
            phase_vec[vec_ind] = dz;
        }

    __m256 dz_reg = _mm256_load_ps((float*)phase_vec);
    dz_reg = _mm256_complexnormalise_ps(dz_reg);

    for (; number < sixteenthPoints; number++)
        {
            a0Val = _mm256_load_ps(aPtr);
            a1Val = _mm256_load_ps(aPtr + 8);
            a2Val = _mm256_load_ps(aPtr + 16);
            a3Val = _mm256_load_ps(aPtr + 24);

            a0Val = _mm256_complexmul_ps(a0Val, z0);
            a1Val = _mm256_complexmul_ps(a1Val, z1);
            a2Val = _mm256_complexmul_ps(a2Val, z2);
            a3Val = _mm256_complexmul_ps(a3Val, z3);

            z0 = _mm256_complexmul_ps(z0, dz_reg);
            z1 = _mm256_complexmul_ps(z1, dz_reg);
            z2 = _mm256_complexmul_ps(z2, dz_reg);
            z3 = _mm256_complexmul_ps(z3, dz_reg);

            for (vec_ind = 0; vec_ind < num_a_vectors; ++vec_ind)
                {
                    x0Val[vec_ind] = _mm256_loadu_ps(bPtr[vec_ind]);  // t0|t1|t2|t3|t4|t5|t6|t7
                    x1Val[vec_ind] = _mm256_loadu_ps(bPtr[vec_ind] + 8);
                    x0loVal[vec_ind] = _mm256_unpacklo_ps(x0Val[vec_ind], x0Val[vec_ind]);  // t0|t0|t1|t1|t4|t4|t5|t5
                    x0hiVal[vec_ind] = _mm256_unpackhi_ps(x0Val[vec_ind], x0Val[vec_ind]);  // t2|t2|t3|t3|t6|t6|t7|t7
                    x1loVal[vec_ind] = _mm256_unpacklo_ps(x1Val[vec_ind], x1Val[vec_ind]);
                    x1hiVal[vec_ind] = _mm256_unpackhi_ps(x1Val[vec_ind], x1Val[vec_ind]);

                    // TODO: it may be possible to rearrange swizzling to better pipeline data
                    b0Val[vec_ind] = _mm256_permute2f128_ps(x0loVal[vec_ind], x0hiVal[vec_ind], 0x20);  // t0|t0|t1|t1|t2|t2|t3|t3
                    b1Val[vec_ind] = _mm256_permute2f128_ps(x0loVal[vec_ind], x0hiVal[vec_ind], 0x31);  // t4|t4|t5|t5|t6|t6|t7|t7
                    b2Val[vec_ind] = _mm256_permute2f128_ps(x1loVal[vec_ind], x1hiVal[vec_ind], 0x20);
                    b3Val[vec_ind] = _mm256_permute2f128_ps(x1loVal[vec_ind], x1hiVal[vec_ind], 0x31);

                    c0Val[vec_ind] = _mm256_mul_ps(a0Val, b0Val[vec_ind]);
                    c1Val[vec_ind] = _mm256_mul_ps(a1Val, b1Val[vec_ind]);
                    c2Val[vec_ind] = _mm256_mul_ps(a2Val, b2Val[vec_ind]);
                    c3Val[vec_ind] = _mm256_mul_ps(a3Val, b3Val[vec_ind]);

                    dotProdVal0[vec_ind] = _mm256_add_ps(c0Val[vec_ind], dotProdVal0[vec_ind]);
                    dotProdVal1[vec_ind] = _mm256_add_ps(c1Val[vec_ind], dotProdVal1[vec_ind]);
                    dotProdVal2[vec_ind] = _mm256_add_ps(c2Val[vec_ind], dotProdVal2[vec_ind]);
                    dotProdVal3[vec_ind] = _mm256_add_ps(c3Val[vec_ind], dotProdVal3[vec_ind]);

                    bPtr[vec_ind] += 16;
                }

            // Force the rotators back onto the unit circle
            if ((number % 64) == 0)
                {
                    z0 = _mm256_complexnormalise_ps(z0);
                    z1 = _mm256_complexnormalise_ps(z1);
                    z2 = _mm256_complexnormalise_ps(z2);
                    z3 = _mm256_complexnormalise_ps(z3);
                }

            aPtr += 32;
        }
    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t dotProductVector[4];

    for (vec_ind = 0; vec_ind < num_a_vectors; ++vec_ind)
        {
            dotProdVal0[vec_ind] = _mm256_add_ps(dotProdVal0[vec_ind], dotProdVal1[vec_ind]);
            dotProdVal0[vec_ind] = _mm256_add_ps(dotProdVal0[vec_ind], dotProdVal2[vec_ind]);
            dotProdVal0[vec_ind] = _mm256_add_ps(dotProdVal0[vec_ind], dotProdVal3[vec_ind]);

            _mm256_store_ps((float*)dotProductVector, dotProdVal0[vec_ind]);  // Store the results back into the dot product vector

            result[vec_ind] = lv_cmake(0, 0);
            for (i = 0; i < 4; ++i)
                {
                    result[vec_ind] += dotProductVector[i];
                }
        }

    z0 = _mm256_complexnormalise_ps(z0);
    _mm256_store_ps((float*)phase_vec, z0);
    _phase = phase_vec[0];
    _mm256_zeroupper();

    number = sixteenthPoints * 16;
    for (; number < num_points; number++)
        {
            wo = (*aPtr++) * _phase;
            _phase *= phase_inc;

            for (vec_ind = 0; vec_ind < num_a_vectors; ++vec_ind)
                {
                    result[vec_ind] += wo * in_a[vec_ind][number];
                }
        }

    *phase = _phase;
}


#endif /* LV_HAVE_AVX */

#endif /* INCLUDED_volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn_H */
