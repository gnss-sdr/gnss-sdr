/*!
 * \file volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn.h
 * \brief VOLK_GNSSSDR kernel: multiplies N 16 bits vectors by a common vector
 * phase rotated and accumulates the results in N 16 bits short complex outputs.
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that multiplies N 16 bits vectors by a common vector, which is
 * phase-rotated by phase offset and phase increment, and accumulates the results
 * in N 16 bits short complex outputs.
 * It is optimized to perform the N tap correlation process in GNSS receivers.
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
 * \page volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn
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
 * void volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn((lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const int16_t** in_a, int num_a_vectors, unsigned int num_points);
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

#ifndef INCLUDED_volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_H
#define INCLUDED_volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_H


#include <volk_gnsssdr/saturation_arithmetic.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <math.h>
// #include <stdio.h>

#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_generic(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const int16_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;
    int n_vec;
    unsigned int n;
    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            result[n_vec] = lv_cmake(0, 0);
        }
    for (n = 0; n < num_points; n++)
        {
            tmp16 = *in_common++;  // if(n<10 || n >= 8108) printf("generic phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));

            // Regenerate phase
            if (n % 256 == 0)
                {
                    // printf("Phase before regeneration %i: %f,%f  Modulus: %f\n", n,lv_creal(*phase),lv_cimag(*phase), cabsf(*phase));
#ifdef __cplusplus
                    (*phase) /= std::abs((*phase));
#else
                    (*phase) /= hypotf(lv_creal(*phase), lv_cimag(*phase));
#endif
                    // printf("Phase after regeneration %i: %f,%f  Modulus: %f\n", n,lv_creal(*phase),lv_cimag(*phase), cabsf(*phase));
                }

            (*phase) *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
                    // lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                    result[n_vec] = lv_cmake(sat_adds16i(lv_creal(result[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(result[n_vec]), lv_cimag(tmp)));
                }
        }
}

#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_generic_reload(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const int16_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;
    int n_vec;
    unsigned int n;
    unsigned int j;
    const unsigned int ROTATOR_RELOAD = 256;
    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            result[n_vec] = lv_cmake(0, 0);
        }

    for (n = 0; n < num_points / ROTATOR_RELOAD; n++)
        {
            for (j = 0; j < ROTATOR_RELOAD; j++)
                {
                    tmp16 = *in_common++;  // if(n<10 || n >= 8108) printf("generic phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
                    tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
                    tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
                    (*phase) *= phase_inc;
                    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            lv_16sc_t tmp = tmp16 * in_a[n_vec][n * ROTATOR_RELOAD + j];
                            // lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                            result[n_vec] = lv_cmake(sat_adds16i(lv_creal(result[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(result[n_vec]), lv_cimag(tmp)));
                        }
                }
                /* Regenerate phase */
#ifdef __cplusplus
            (*phase) /= std::abs((*phase));
#else
            // (*phase) /= cabsf((*phase));
            (*phase) /= hypotf(lv_creal(*phase), lv_cimag(*phase));
#endif
        }

    for (j = 0; j < num_points % ROTATOR_RELOAD; j++)
        {
            tmp16 = *in_common++;  // if(n<10 || n >= 8108) printf("generic phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][(num_points / ROTATOR_RELOAD) * ROTATOR_RELOAD + j];
                    // lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                    result[n_vec] = lv_cmake(sat_adds16i(lv_creal(result[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(result[n_vec]), lv_cimag(tmp)));
                }
        }
}

#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

static inline void volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_a_sse3(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const int16_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake(0, 0);

    const unsigned int sse_iters = num_points / 4;
    int n_vec;
    int i;
    unsigned int number;
    unsigned int n;
    const int16_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;

    __VOLK_ATTR_ALIGNED(16)
    lv_16sc_t dotProductVector[4];

    __m128i* cacc = (__m128i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m128i), volk_gnsssdr_get_alignment());

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            cacc[n_vec] = _mm_setzero_si128();
        }

    __m128i a, b, c;

    // phase rotation registers
    __m128 pa, pb, two_phase_acc_reg, two_phase_inc_reg;
    __m128i pc1, pc2;
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
    __m128 yl, yh, tmp1, tmp2, tmp3;
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;

    for (number = 0; number < sse_iters; number++)
        {
            // Phase rotation on operand in_common starts here:
            // printf("generic phase %i: %f,%f\n", n*4,lv_creal(*phase),lv_cimag(*phase));
            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl);                // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1);        // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh);                // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2);           // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc1 = _mm_cvtps_epi32(pb);                // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // next two samples
            _in_common += 2;
            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            __VOLK_GNSSSDR_PREFETCH(_in_common + 8);
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl);                // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1);        // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh);                // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2);           // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc2 = _mm_cvtps_epi32(pb);                // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // store four rotated in_common samples in the register b
            b = _mm_packs_epi32(pc1, pc2);  // convert from 32ic to 16ic

            // next two samples
            _in_common += 2;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    a = _mm_loadl_epi64((__m128i*)&(_in_a[n_vec][number * 4]));  // load (2 byte imag, 2 byte real) x 4 into 128 bits reg

                    a = _mm_unpacklo_epi16(a, a);

                    c = _mm_mullo_epi16(a, b);  // a3.i*b3.i, a3.r*b3.r, ....

                    cacc[n_vec] = _mm_adds_epi16(cacc[n_vec], c);
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
            a = cacc[n_vec];
            _mm_store_si128((__m128i*)dotProductVector, a);  // Store the results back into the dot product vector
            dotProduct = lv_cmake(0, 0);
            for (i = 0; i < 4; ++i)
                {
                    dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])),
                        sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
                }
            _out[n_vec] = dotProduct;
        }
    volk_gnsssdr_free(cacc);

    tmp1 = _mm_mul_ps(two_phase_acc_reg, two_phase_acc_reg);
    tmp2 = _mm_hadd_ps(tmp1, tmp1);
    tmp1 = _mm_shuffle_ps(tmp2, tmp2, 0xD8);
    tmp2 = _mm_sqrt_ps(tmp1);
    two_phase_acc_reg = _mm_div_ps(two_phase_acc_reg, tmp2);

    _mm_store_ps((float*)two_phase_acc, two_phase_acc_reg);
    // (*phase) = lv_cmake((float*)two_phase_acc[0], (float*)two_phase_acc[1]);
    (*phase) = two_phase_acc[0];

    for (n = sse_iters * 4; n < num_points; n++)
        {
            tmp16 = in_common[n];  // printf("a_sse phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
                    // lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

static inline void volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_u_sse3(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const int16_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake(0, 0);

    const unsigned int sse_iters = num_points / 4;
    int n_vec;
    int i;
    unsigned int number;
    unsigned int n;
    const int16_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;

    __VOLK_ATTR_ALIGNED(16)
    lv_16sc_t dotProductVector[4];

    __m128i* cacc = (__m128i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m128i), volk_gnsssdr_get_alignment());

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            cacc[n_vec] = _mm_setzero_si128();
        }

    __m128i a, b, c;

    // phase rotation registers
    __m128 pa, pb, two_phase_acc_reg, two_phase_inc_reg;
    __m128i pc1, pc2;
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
    __m128 yl, yh, tmp1, tmp2, tmp3;
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;

    for (number = 0; number < sse_iters; number++)
        {
            // Phase rotation on operand in_common starts here:
            // printf("generic phase %i: %f,%f\n", n*4,lv_creal(*phase),lv_cimag(*phase));
            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl);                // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1);        // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh);                // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2);           // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc1 = _mm_cvtps_epi32(pb);                // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // next two samples
            _in_common += 2;
            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0])));  // // load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            __VOLK_GNSSSDR_PREFETCH(_in_common + 8);
            // complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl);                // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1);        // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh);                // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2);           // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc2 = _mm_cvtps_epi32(pb);                // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // store four rotated in_common samples in the register b
            b = _mm_packs_epi32(pc1, pc2);  // convert from 32ic to 16ic

            // next two samples
            _in_common += 2;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    a = _mm_loadl_epi64((__m128i*)&(_in_a[n_vec][number * 4]));  // load (2 byte imag, 2 byte real) x 4 into 128 bits reg

                    a = _mm_unpacklo_epi16(a, a);

                    c = _mm_mullo_epi16(a, b);  // a3.i*b3.i, a3.r*b3.r, ....

                    cacc[n_vec] = _mm_adds_epi16(cacc[n_vec], c);
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
            a = cacc[n_vec];
            _mm_store_si128((__m128i*)dotProductVector, a);  // Store the results back into the dot product vector
            dotProduct = lv_cmake(0, 0);
            for (i = 0; i < 4; ++i)
                {
                    dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])),
                        sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
                }
            _out[n_vec] = dotProduct;
        }
    volk_gnsssdr_free(cacc);

    tmp1 = _mm_mul_ps(two_phase_acc_reg, two_phase_acc_reg);
    tmp2 = _mm_hadd_ps(tmp1, tmp1);
    tmp1 = _mm_shuffle_ps(tmp2, tmp2, 0xD8);
    tmp2 = _mm_sqrt_ps(tmp1);
    two_phase_acc_reg = _mm_div_ps(two_phase_acc_reg, tmp2);

    _mm_store_ps((float*)two_phase_acc, two_phase_acc_reg);
    // (*phase) = lv_cmake((float*)two_phase_acc[0], (float*)two_phase_acc[1]);
    (*phase) = two_phase_acc[0];

    for (n = sse_iters * 4; n < num_points; n++)
        {
            tmp16 = in_common[n];  // printf("a_sse phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
                    // lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_AVX2
#include <volk_gnsssdr/volk_gnsssdr_avx_intrinsics.h>
#include <volk_gnsssdr/volk_gnsssdr_sse3_intrinsics.h>
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_a_avx2(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const int16_t** in_a, int num_a_vectors, unsigned int num_points)
{
    const unsigned int avx2_iters = num_points / 8;
    const int16_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;
    int n_vec;
    unsigned int number;
    unsigned int n;

    lv_16sc_t tmp16;
    lv_32fc_t tmp32;

    __VOLK_ATTR_ALIGNED(32)
    lv_16sc_t dotProductVector[8];
    lv_16sc_t dotProduct = lv_cmake(0, 0);

    __m256i* cacc = (__m256i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m256i), volk_gnsssdr_get_alignment());

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            cacc[n_vec] = _mm256_setzero_si256();
        }

    __m128i a128, ain_128, ain_128_lo, ain_128_hi;
    __m256i ai;
    __m256 a, b;

    __m256 four_phase_acc_reg, four_phase_inc_reg;

    lv_32fc_t _phase_inc = phase_inc * phase_inc * phase_inc * phase_inc;

    // Normalise the 4*phase increment
#ifdef __cplusplus
    _phase_inc /= std::abs(_phase_inc);
#else
    _phase_inc /= hypotf(lv_creal(_phase_inc), lv_cimag(_phase_inc));
#endif

    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t four_phase_inc[4];
    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t four_phase_acc[4];
    for (n = 0; n < 4; ++n)
        {
            four_phase_inc[n] = _phase_inc;
            four_phase_acc[n] = *phase;
            *phase *= phase_inc;
        }
    four_phase_acc_reg = _mm256_load_ps((float*)four_phase_acc);
    four_phase_inc_reg = _mm256_load_ps((float*)four_phase_inc);

    __m256i a2, b2, c, c1, c2, perm_idx;

    perm_idx = _mm256_set_epi32(7, 6, 3, 2, 5, 4, 1, 0);
    // perm_idx = _mm256_set_epi32( 0, 1, 4, 5, 2, 3, 6, 7);

    for (number = 0; number < avx2_iters; number++)
        {
            a128 = _mm_load_si128((__m128i*)_in_common);
            ai = _mm256_cvtepi16_epi32(a128);
            a = _mm256_cvtepi32_ps(ai);

            // complex 32fc multiplication b=a*two_phase_acc_reg
            b = _mm256_complexmul_ps(a, four_phase_acc_reg);
            c1 = _mm256_cvtps_epi32(b);  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            four_phase_acc_reg = _mm256_complexmul_ps(four_phase_inc_reg, four_phase_acc_reg);

            // next four samples
            _in_common += 4;
            a128 = _mm_load_si128((__m128i*)_in_common);
            ai = _mm256_cvtepi16_epi32(a128);
            a = _mm256_cvtepi32_ps(ai);

            // complex 32fc multiplication b=a*two_phase_acc_reg
            b = _mm256_complexmul_ps(a, four_phase_acc_reg);
            c2 = _mm256_cvtps_epi32(b);  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            four_phase_acc_reg = _mm256_complexmul_ps(four_phase_inc_reg, four_phase_acc_reg);

            __VOLK_GNSSSDR_PREFETCH(_in_common + 16);


            // Store and convert 32ic to 16ic:
            b2 = _mm256_packs_epi32(c1, c2);

            b2 = _mm256_permutevar8x32_epi32(b2, perm_idx);


            _in_common += 4;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    ain_128 = _mm_load_si128((__m128i*)&(_in_a[n_vec][number * 8]));

                    ain_128_lo = _mm_unpacklo_epi16(ain_128, ain_128);
                    ain_128_hi = _mm_unpackhi_epi16(ain_128, ain_128);

                    a2 = _mm256_insertf128_si256(_mm256_castsi128_si256(ain_128_lo), ain_128_hi, 1);

                    c = _mm256_mullo_epi16(a2, b2);

                    cacc[n_vec] = _mm256_adds_epi16(cacc[n_vec], c);
                }
            // Regenerate phase
            if ((number % 128) == 0)
                {
                    four_phase_acc_reg = _mm256_complexnormalise_ps(four_phase_acc_reg);
                }
        }

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            a2 = cacc[n_vec];

            _mm256_store_si256((__m256i*)dotProductVector, a2);  // Store the results back into the dot product vector
            dotProduct = lv_cmake(0, 0);
            for (number = 0; number < 8; ++number)
                {
                    dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[number])),
                        sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[number])));
                }
            _out[n_vec] = dotProduct;
        }

    volk_gnsssdr_free(cacc);

    _mm256_store_ps((float*)four_phase_acc, four_phase_acc_reg);
    (*phase) = four_phase_acc[0];

    for (n = avx2_iters * 8; n < num_points; n++)
        {
            tmp16 = in_common[n];
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_AVX2 */

#ifdef LV_HAVE_AVX2
#include <volk_gnsssdr/volk_gnsssdr_avx_intrinsics.h>
#include <volk_gnsssdr/volk_gnsssdr_sse3_intrinsics.h>
#include <immintrin.h>

static inline void volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_u_avx2(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const int16_t** in_a, int num_a_vectors, unsigned int num_points)
{
    const unsigned int avx2_iters = num_points / 8;
    const int16_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;
    int n_vec;
    unsigned int number;
    unsigned int n;

    lv_16sc_t tmp16;
    lv_32fc_t tmp32;

    __VOLK_ATTR_ALIGNED(32)
    lv_16sc_t dotProductVector[8];
    lv_16sc_t dotProduct = lv_cmake(0, 0);

    __m256i* cacc = (__m256i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m256i), volk_gnsssdr_get_alignment());

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            cacc[n_vec] = _mm256_setzero_si256();
        }

    __m128i a128, ain_128, ain_128_lo, ain_128_hi;
    __m256i ai;
    __m256 a, b;

    __m256 four_phase_acc_reg, four_phase_inc_reg;

    lv_32fc_t _phase_inc = phase_inc * phase_inc * phase_inc * phase_inc;

    // Normalise the 4*phase increment
#ifdef __cplusplus
    _phase_inc /= std::abs(_phase_inc);
#else
    _phase_inc /= hypotf(lv_creal(_phase_inc), lv_cimag(_phase_inc));
#endif

    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t four_phase_inc[4];
    __VOLK_ATTR_ALIGNED(32)
    lv_32fc_t four_phase_acc[4];
    for (n = 0; n < 4; ++n)
        {
            four_phase_inc[n] = _phase_inc;
            four_phase_acc[n] = *phase;
            *phase *= phase_inc;
        }
    four_phase_acc_reg = _mm256_load_ps((float*)four_phase_acc);
    four_phase_inc_reg = _mm256_load_ps((float*)four_phase_inc);

    __m256i a2, b2, c, c1, c2, perm_idx;

    perm_idx = _mm256_set_epi32(7, 6, 3, 2, 5, 4, 1, 0);
    // perm_idx = _mm256_set_epi32( 0, 1, 4, 5, 2, 3, 6, 7);

    for (number = 0; number < avx2_iters; number++)
        {
            a128 = _mm_loadu_si128((__m128i*)_in_common);
            ai = _mm256_cvtepi16_epi32(a128);
            a = _mm256_cvtepi32_ps(ai);

            // complex 32fc multiplication b=a*two_phase_acc_reg
            b = _mm256_complexmul_ps(a, four_phase_acc_reg);
            c1 = _mm256_cvtps_epi32(b);  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            four_phase_acc_reg = _mm256_complexmul_ps(four_phase_inc_reg, four_phase_acc_reg);

            // next four samples
            _in_common += 4;
            a128 = _mm_loadu_si128((__m128i*)_in_common);
            ai = _mm256_cvtepi16_epi32(a128);
            a = _mm256_cvtepi32_ps(ai);

            // complex 32fc multiplication b=a*two_phase_acc_reg
            b = _mm256_complexmul_ps(a, four_phase_acc_reg);
            c2 = _mm256_cvtps_epi32(b);  // convert from 32fc to 32ic

            // complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            four_phase_acc_reg = _mm256_complexmul_ps(four_phase_inc_reg, four_phase_acc_reg);

            __VOLK_GNSSSDR_PREFETCH(_in_common + 16);

            // Store and convert 32ic to 16ic:
            b2 = _mm256_packs_epi32(c1, c2);

            b2 = _mm256_permutevar8x32_epi32(b2, perm_idx);

            _in_common += 4;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    ain_128 = _mm_loadu_si128((__m128i*)&(_in_a[n_vec][number * 8]));

                    ain_128_lo = _mm_unpacklo_epi16(ain_128, ain_128);
                    ain_128_hi = _mm_unpackhi_epi16(ain_128, ain_128);

                    a2 = _mm256_insertf128_si256(_mm256_castsi128_si256(ain_128_lo), ain_128_hi, 1);

                    c = _mm256_mullo_epi16(a2, b2);

                    cacc[n_vec] = _mm256_adds_epi16(cacc[n_vec], c);
                }
            // Regenerate phase
            if ((number % 128) == 0)
                {
                    four_phase_acc_reg = _mm256_complexnormalise_ps(four_phase_acc_reg);
                }
        }

    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            a2 = cacc[n_vec];

            _mm256_store_si256((__m256i*)dotProductVector, a2);  // Store the results back into the dot product vector
            dotProduct = lv_cmake(0, 0);
            for (number = 0; number < 8; ++number)
                {
                    dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[number])),
                        sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[number])));
                }
            _out[n_vec] = dotProduct;
        }

    volk_gnsssdr_free(cacc);

    _mm256_store_ps((float*)four_phase_acc, four_phase_acc_reg);
    (*phase) = four_phase_acc[0];

    for (n = avx2_iters * 8; n < num_points; n++)
        {
            tmp16 = in_common[n];
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_AVX2 */

#endif /* INCLUDED_volk_gnsssdr_16ic_16i_dot_prod_16ic_xn_H */
