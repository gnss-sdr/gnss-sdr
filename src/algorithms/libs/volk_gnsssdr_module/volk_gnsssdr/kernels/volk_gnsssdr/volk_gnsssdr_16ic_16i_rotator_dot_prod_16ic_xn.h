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


#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/saturation_arithmetic.h>
#include <math.h>
//#include <stdio.h>

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
            tmp16 = *in_common++;  //if(n<10 || n >= 8108) printf("generic phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));

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
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
                    //lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                    result[n_vec] = lv_cmake(sat_adds16i(lv_creal(result[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(result[n_vec]), lv_cimag(tmp)));
                }
        }
}

#endif /*LV_HAVE_GENERIC*/


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
                    tmp16 = *in_common++;  //if(n<10 || n >= 8108) printf("generic phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
                    tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
                    tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
                    (*phase) *= phase_inc;
                    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            lv_16sc_t tmp = tmp16 * in_a[n_vec][n * ROTATOR_RELOAD + j];
                            //lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                            result[n_vec] = lv_cmake(sat_adds16i(lv_creal(result[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(result[n_vec]), lv_cimag(tmp)));
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
            tmp16 = *in_common++;  //if(n<10 || n >= 8108) printf("generic phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;
            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][(num_points / ROTATOR_RELOAD) * ROTATOR_RELOAD + j];
                    //lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                    result[n_vec] = lv_cmake(sat_adds16i(lv_creal(result[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(result[n_vec]), lv_cimag(tmp)));
                }
        }
}

#endif /*LV_HAVE_GENERIC*/


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
            //printf("generic phase %i: %f,%f\n", n*4,lv_creal(*phase),lv_cimag(*phase));
            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0])));  // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            //complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl);                // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1);        // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh);                // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2);           // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc1 = _mm_cvtps_epi32(pb);                // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            //next two samples
            _in_common += 2;
            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0])));  // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            __VOLK_GNSSSDR_PREFETCH(_in_common + 8);
            //complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl);                // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1);        // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh);                // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2);           // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc2 = _mm_cvtps_epi32(pb);                // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // store four rotated in_common samples in the register b
            b = _mm_packs_epi32(pc1, pc2);  // convert from 32ic to 16ic

            //next two samples
            _in_common += 2;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    a = _mm_loadl_epi64((__m128i*)&(_in_a[n_vec][number * 4]));  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg

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
    //(*phase) = lv_cmake((float*)two_phase_acc[0], (float*)two_phase_acc[1]);
    (*phase) = two_phase_acc[0];

    for (n = sse_iters * 4; n < num_points; n++)
        {
            tmp16 = in_common[n];  //printf("a_sse phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
                    //lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_SSE3 */


//#ifdef LV_HAVE_SSE3
//#include <pmmintrin.h>

//static inline void volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_a_sse3_reload(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const int16_t** in_a,  int num_a_vectors, unsigned int num_points)
//{
//lv_16sc_t dotProduct = lv_cmake(0,0);

//const unsigned int sse_iters = num_points / 4;
//const unsigned int ROTATOR_RELOAD = 128;
//int n_vec;
//int i;
//unsigned int number;
//unsigned int j;
//unsigned int n;

//const int16_t** _in_a = in_a;
//const lv_16sc_t* _in_common = in_common;
//lv_16sc_t* _out = result;

//__VOLK_ATTR_ALIGNED(16) lv_16sc_t dotProductVector[4];

//__m128i* realcacc = (__m128i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m128i), volk_gnsssdr_get_alignment());
//__m128i* imagcacc = (__m128i*)volk_gnsssdr_malloc(num_a_vectors * sizeof(__m128i), volk_gnsssdr_get_alignment());

//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//realcacc[n_vec] = _mm_setzero_si128();
//imagcacc[n_vec] = _mm_setzero_si128();
//}

//__m128i a, b, c, c_sr, mask_imag, mask_real, real, imag, imag1, imag2, b_sl, a_sl;

//mask_imag = _mm_set_epi8(255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0);
//mask_real = _mm_set_epi8(0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255);

//// phase rotation registers
//__m128 pa, pb, two_phase_acc_reg, two_phase_inc_reg;
//__m128i pc1, pc2;
//__VOLK_ATTR_ALIGNED(16) lv_32fc_t two_phase_inc[2];
//two_phase_inc[0] = phase_inc * phase_inc;
//two_phase_inc[1] = phase_inc * phase_inc;
//two_phase_inc_reg = _mm_load_ps((float*) two_phase_inc);
//__VOLK_ATTR_ALIGNED(16) lv_32fc_t two_phase_acc[2];
//two_phase_acc[0] = (*phase);
//two_phase_acc[1] = (*phase) * phase_inc;
//two_phase_acc_reg = _mm_load_ps((float*)two_phase_acc);
//__m128 yl, yh, tmp1, tmp2, tmp3;
//lv_16sc_t tmp16;
//lv_32fc_t tmp32;

//for (number = 0; number <  sse_iters / ROTATOR_RELOAD; ++number)
//{
//for (j = 0; j < ROTATOR_RELOAD; j++)
//{
//// Phase rotation on operand in_common starts here:
////printf("generic phase %i: %f,%f\n", n*4,lv_creal(*phase),lv_cimag(*phase));
//pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0]))); // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
////complex 32fc multiplication b=a*two_phase_acc_reg
//yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
//yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
//tmp1 = _mm_mul_ps(pa, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
//pa = _mm_shuffle_ps(pa, pa, 0xB1); // Re-arrange x to be ai,ar,bi,br
//tmp2 = _mm_mul_ps(pa, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
//pb = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
//pc1 = _mm_cvtps_epi32(pb); // convert from 32fc to 32ic

////complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
//yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
//yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
//tmp1 = _mm_mul_ps(two_phase_inc_reg, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
//tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1); // Re-arrange x to be ai,ar,bi,br
//tmp2 = _mm_mul_ps(tmp3, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
//two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

////next two samples
//_in_common += 2;
//pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0]))); // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
//__VOLK_GNSSSDR_PREFETCH(_in_common + 8);
////complex 32fc multiplication b=a*two_phase_acc_reg
//yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
//yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
//tmp1 = _mm_mul_ps(pa, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
//pa = _mm_shuffle_ps(pa, pa, 0xB1); // Re-arrange x to be ai,ar,bi,br
//tmp2 = _mm_mul_ps(pa, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
//pb = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
//pc2 = _mm_cvtps_epi32(pb); // convert from 32fc to 32ic

////complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
//yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
//yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
//tmp1 = _mm_mul_ps(two_phase_inc_reg, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
//tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1); // Re-arrange x to be ai,ar,bi,br
//tmp2 = _mm_mul_ps(tmp3, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
//two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

//// store four rotated in_common samples in the register b
//b = _mm_packs_epi32(pc1, pc2);// convert from 32ic to 16ic

////next two samples
//_in_common += 2;

//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//a = _mm_load_si128((__m128i*)&(_in_a[n_vec][(number * ROTATOR_RELOAD + j) * 4])); //load (2 byte imag, 2 byte real) x 4 into 128 bits reg

//c = _mm_mullo_epi16(a, b); // a3.i*b3.i, a3.r*b3.r, ....

//c_sr = _mm_srli_si128(c, 2); // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
//real = _mm_subs_epi16(c, c_sr);

//b_sl = _mm_slli_si128(b, 2); // b3.r, b2.i ....
//a_sl = _mm_slli_si128(a, 2); // a3.r, a2.i ....

//imag1 = _mm_mullo_epi16(a, b_sl); // a3.i*b3.r, ....
//imag2 = _mm_mullo_epi16(b, a_sl); // b3.i*a3.r, ....

//imag = _mm_adds_epi16(imag1, imag2);

//realcacc[n_vec] = _mm_adds_epi16(realcacc[n_vec], real);
//imagcacc[n_vec] = _mm_adds_epi16(imagcacc[n_vec], imag);
//}
//}
//// regenerate phase
//tmp1 = _mm_mul_ps(two_phase_acc_reg, two_phase_acc_reg);
//tmp2 = _mm_hadd_ps(tmp1, tmp1);
//tmp1 = _mm_shuffle_ps(tmp2, tmp2, 0xD8);
//tmp2 = _mm_sqrt_ps(tmp1);
//two_phase_acc_reg = _mm_div_ps(two_phase_acc_reg, tmp2);
//}

//for (j = 0; j < sse_iters % ROTATOR_RELOAD; j++)
//{
//pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0]))); // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
////complex 32fc multiplication b=a*two_phase_acc_reg
//yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
//yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
//tmp1 = _mm_mul_ps(pa, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
//pa = _mm_shuffle_ps(pa, pa, 0xB1); // Re-arrange x to be ai,ar,bi,br
//tmp2 = _mm_mul_ps(pa, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
//pb = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
//pc1 = _mm_cvtps_epi32(pb); // convert from 32fc to 32ic

////complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
//yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
//yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
//tmp1 = _mm_mul_ps(two_phase_inc_reg, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
//tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1); // Re-arrange x to be ai,ar,bi,br
//tmp2 = _mm_mul_ps(tmp3, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
//two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

////next two samples
//_in_common += 2;
//pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0]))); // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
//__VOLK_GNSSSDR_PREFETCH(_in_common + 8);
////complex 32fc multiplication b=a*two_phase_acc_reg
//yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
//yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
//tmp1 = _mm_mul_ps(pa, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
//pa = _mm_shuffle_ps(pa, pa, 0xB1); // Re-arrange x to be ai,ar,bi,br
//tmp2 = _mm_mul_ps(pa, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
//pb = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
//pc2 = _mm_cvtps_epi32(pb); // convert from 32fc to 32ic

////complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
//yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
//yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
//tmp1 = _mm_mul_ps(two_phase_inc_reg, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
//tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1); // Re-arrange x to be ai,ar,bi,br
//tmp2 = _mm_mul_ps(tmp3, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
//two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

//// store four rotated in_common samples in the register b
//b = _mm_packs_epi32(pc1, pc2);// convert from 32ic to 16ic

////next two samples
//_in_common += 2;

//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//a = _mm_load_si128((__m128i*)&(_in_a[n_vec][((sse_iters / ROTATOR_RELOAD)  * ROTATOR_RELOAD + j) * 4])); //load (2 byte imag, 2 byte real) x 4 into 128 bits reg

//c = _mm_mullo_epi16(a, b); // a3.i*b3.i, a3.r*b3.r, ....

//c_sr = _mm_srli_si128(c, 2); // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
//real = _mm_subs_epi16(c, c_sr);

//b_sl = _mm_slli_si128(b, 2); // b3.r, b2.i ....
//a_sl = _mm_slli_si128(a, 2); // a3.r, a2.i ....

//imag1 = _mm_mullo_epi16(a, b_sl); // a3.i*b3.r, ....
//imag2 = _mm_mullo_epi16(b, a_sl); // b3.i*a3.r, ....

//imag = _mm_adds_epi16(imag1, imag2);

//realcacc[n_vec] = _mm_adds_epi16(realcacc[n_vec], real);
//imagcacc[n_vec] = _mm_adds_epi16(imagcacc[n_vec], imag);
//}
//}

//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//realcacc[n_vec] = _mm_and_si128(realcacc[n_vec], mask_real);
//imagcacc[n_vec] = _mm_and_si128(imagcacc[n_vec], mask_imag);

//a = _mm_or_si128(realcacc[n_vec], imagcacc[n_vec]);

//_mm_store_si128((__m128i*)dotProductVector, a); // Store the results back into the dot product vector
//dotProduct = lv_cmake(0,0);
//for (i = 0; i < 4; ++i)
//{
//dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])),
//sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
//}
//_out[n_vec] = dotProduct;
//}

//volk_gnsssdr_free(realcacc);
//volk_gnsssdr_free(imagcacc);

//tmp1 = _mm_mul_ps(two_phase_acc_reg, two_phase_acc_reg);
//tmp2 = _mm_hadd_ps(tmp1, tmp1);
//tmp1 = _mm_shuffle_ps(tmp2, tmp2, 0xD8);
//tmp2 = _mm_sqrt_ps(tmp1);
//two_phase_acc_reg = _mm_div_ps(two_phase_acc_reg, tmp2);

//_mm_store_ps((float*)two_phase_acc, two_phase_acc_reg);
////(*phase) = lv_cmake((float*)two_phase_acc[0], (float*)two_phase_acc[1]);
//(*phase) = two_phase_acc[0];

//for(n = sse_iters * 4; n < num_points; n++)
//{
//tmp16 = in_common[n];  //printf("a_sse phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
//tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
//tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
//(*phase) *= phase_inc;

//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
////lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
//_out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
//sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
//}
//}

//}
//#endif [> LV_HAVE_SSE3 <]


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
            //printf("generic phase %i: %f,%f\n", n*4,lv_creal(*phase),lv_cimag(*phase));
            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0])));  // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            //complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl);                // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1);        // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh);                // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2);           // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc1 = _mm_cvtps_epi32(pb);                // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            //next two samples
            _in_common += 2;
            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0])));  // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            __VOLK_GNSSSDR_PREFETCH(_in_common + 8);
            //complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);  // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);  // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl);                // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1);        // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh);                // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2);           // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc2 = _mm_cvtps_epi32(pb);                // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg);                            // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg);                            // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl);                           // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1);  // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh);                                        // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2);                      // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // store four rotated in_common samples in the register b
            b = _mm_packs_epi32(pc1, pc2);  // convert from 32ic to 16ic

            //next two samples
            _in_common += 2;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    a = _mm_loadl_epi64((__m128i*)&(_in_a[n_vec][number * 4]));  //load (2 byte imag, 2 byte real) x 4 into 128 bits reg

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
    //(*phase) = lv_cmake((float*)two_phase_acc[0], (float*)two_phase_acc[1]);
    (*phase) = two_phase_acc[0];

    for (n = sse_iters * 4; n < num_points; n++)
        {
            tmp16 = in_common[n];  //printf("a_sse phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
                    //lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                        sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>
#include <volk_gnsssdr/volk_gnsssdr_sse3_intrinsics.h>
#include <volk_gnsssdr/volk_gnsssdr_avx_intrinsics.h>

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
    //perm_idx = _mm256_set_epi32( 0, 1, 4, 5, 2, 3, 6, 7);

    for (number = 0; number < avx2_iters; number++)
        {
            a128 = _mm_load_si128((__m128i*)_in_common);
            ai = _mm256_cvtepi16_epi32(a128);
            a = _mm256_cvtepi32_ps(ai);

            //complex 32fc multiplication b=a*two_phase_acc_reg
            b = _mm256_complexmul_ps(a, four_phase_acc_reg);
            c1 = _mm256_cvtps_epi32(b);  // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            four_phase_acc_reg = _mm256_complexmul_ps(four_phase_inc_reg, four_phase_acc_reg);

            //next four samples
            _in_common += 4;
            a128 = _mm_load_si128((__m128i*)_in_common);
            ai = _mm256_cvtepi16_epi32(a128);
            a = _mm256_cvtepi32_ps(ai);

            //complex 32fc multiplication b=a*two_phase_acc_reg
            b = _mm256_complexmul_ps(a, four_phase_acc_reg);
            c2 = _mm256_cvtps_epi32(b);  // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
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
    _mm256_zeroupper();

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
#include <immintrin.h>
#include <volk_gnsssdr/volk_gnsssdr_sse3_intrinsics.h>
#include <volk_gnsssdr/volk_gnsssdr_avx_intrinsics.h>

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
    //perm_idx = _mm256_set_epi32( 0, 1, 4, 5, 2, 3, 6, 7);

    for (number = 0; number < avx2_iters; number++)
        {
            a128 = _mm_loadu_si128((__m128i*)_in_common);
            ai = _mm256_cvtepi16_epi32(a128);
            a = _mm256_cvtepi32_ps(ai);

            //complex 32fc multiplication b=a*two_phase_acc_reg
            b = _mm256_complexmul_ps(a, four_phase_acc_reg);
            c1 = _mm256_cvtps_epi32(b);  // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            four_phase_acc_reg = _mm256_complexmul_ps(four_phase_inc_reg, four_phase_acc_reg);

            //next four samples
            _in_common += 4;
            a128 = _mm_loadu_si128((__m128i*)_in_common);
            ai = _mm256_cvtepi16_epi32(a128);
            a = _mm256_cvtepi32_ps(ai);

            //complex 32fc multiplication b=a*two_phase_acc_reg
            b = _mm256_complexmul_ps(a, four_phase_acc_reg);
            c2 = _mm256_cvtps_epi32(b);  // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
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
    _mm256_zeroupper();

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

//#ifdef LV_HAVE_NEON
//#include <arm_neon.h>

//static inline void volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_neon(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const int16_t** in_a,  int num_a_vectors, unsigned int num_points)
//{
//const unsigned int neon_iters = num_points / 4;

//const int16_t** _in_a = in_a;
//const lv_16sc_t* _in_common = in_common;
//lv_16sc_t* _out = result;
//int n_vec;
//int i;
//unsigned int number;
//unsigned int n;
//lv_16sc_t tmp16_, tmp;
//lv_32fc_t tmp32_;

//if (neon_iters > 0)
//{
//lv_16sc_t dotProduct = lv_cmake(0,0);
//float arg_phase0 = cargf(*phase);
//float arg_phase_inc = cargf(phase_inc);
//float phase_est;

//lv_32fc_t ___phase4 = phase_inc * phase_inc * phase_inc * phase_inc;
//__VOLK_ATTR_ALIGNED(16) float32_t __phase4_real[4] = { lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4) };
//__VOLK_ATTR_ALIGNED(16) float32_t __phase4_imag[4] = { lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4) };

//float32x4_t _phase4_real = vld1q_f32(__phase4_real);
//float32x4_t _phase4_imag = vld1q_f32(__phase4_imag);

//lv_32fc_t phase2 = (lv_32fc_t)(*phase) * phase_inc;
//lv_32fc_t phase3 = phase2 * phase_inc;
//lv_32fc_t phase4 = phase3 * phase_inc;

//__VOLK_ATTR_ALIGNED(16) float32_t __phase_real[4] = { lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4) };
//__VOLK_ATTR_ALIGNED(16) float32_t __phase_imag[4] = { lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4) };

//float32x4_t _phase_real = vld1q_f32(__phase_real);
//float32x4_t _phase_imag = vld1q_f32(__phase_imag);

//int16x4x2_t a_val, b_val, c_val;
//__VOLK_ATTR_ALIGNED(16) lv_16sc_t dotProductVector[4];
//float32x4_t half = vdupq_n_f32(0.5f);
//int16x4x2_t tmp16;
//int32x4x2_t tmp32i;

//float32x4x2_t tmp32f, tmp32_real, tmp32_imag;
//float32x4_t sign, PlusHalf, Round;

//int16x4x2_t* accumulator = (int16x4x2_t*)volk_gnsssdr_malloc(num_a_vectors * sizeof(int16x4x2_t), volk_gnsssdr_get_alignment());

//for(n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//accumulator[n_vec].val[0] = vdup_n_s16(0);
//accumulator[n_vec].val[1] = vdup_n_s16(0);
//}

//for(number = 0; number < neon_iters; number++)
//{
//[> load 4 complex numbers (int 16 bits each component) <]
//tmp16 = vld2_s16((int16_t*)_in_common);
//__VOLK_GNSSSDR_PREFETCH(_in_common + 8);
//_in_common += 4;

//[> promote them to int 32 bits <]
//tmp32i.val[0] = vmovl_s16(tmp16.val[0]);
//tmp32i.val[1] = vmovl_s16(tmp16.val[1]);

//[> promote them to float 32 bits <]
//tmp32f.val[0] = vcvtq_f32_s32(tmp32i.val[0]);
//tmp32f.val[1] = vcvtq_f32_s32(tmp32i.val[1]);

//[> complex multiplication of four complex samples (float 32 bits each component) <]
//tmp32_real.val[0] = vmulq_f32(tmp32f.val[0], _phase_real);
//tmp32_real.val[1] = vmulq_f32(tmp32f.val[1], _phase_imag);
//tmp32_imag.val[0] = vmulq_f32(tmp32f.val[0], _phase_imag);
//tmp32_imag.val[1] = vmulq_f32(tmp32f.val[1], _phase_real);

//tmp32f.val[0] = vsubq_f32(tmp32_real.val[0], tmp32_real.val[1]);
//tmp32f.val[1] = vaddq_f32(tmp32_imag.val[0], tmp32_imag.val[1]);

//[> downcast results to int32 <]
//[> in __aarch64__ we can do that with vcvtaq_s32_f32(ret1); vcvtaq_s32_f32(ret2); <]
//sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[0]), 31)));
//PlusHalf = vaddq_f32(tmp32f.val[0], half);
//Round = vsubq_f32(PlusHalf, sign);
//tmp32i.val[0] = vcvtq_s32_f32(Round);

//sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[1]), 31)));
//PlusHalf = vaddq_f32(tmp32f.val[1], half);
//Round = vsubq_f32(PlusHalf, sign);
//tmp32i.val[1] = vcvtq_s32_f32(Round);

//[> downcast results to int16 <]
//tmp16.val[0] = vqmovn_s32(tmp32i.val[0]);
//tmp16.val[1] = vqmovn_s32(tmp32i.val[1]);

//[> compute next four phases <]
//tmp32_real.val[0] = vmulq_f32(_phase_real, _phase4_real);
//tmp32_real.val[1] = vmulq_f32(_phase_imag, _phase4_imag);
//tmp32_imag.val[0] = vmulq_f32(_phase_real, _phase4_imag);
//tmp32_imag.val[1] = vmulq_f32(_phase_imag, _phase4_real);

//_phase_real = vsubq_f32(tmp32_real.val[0], tmp32_real.val[1]);
//_phase_imag = vaddq_f32(tmp32_imag.val[0], tmp32_imag.val[1]);

//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//a_val = vld2_s16((int16_t*)&(_in_a[n_vec][number*4])); //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
////__VOLK_GNSSSDR_PREFETCH(&_in_a[n_vec][number*4] + 8);

//// multiply the real*real and imag*imag to get real result
//// a0r*b0r|a1r*b1r|a2r*b2r|a3r*b3r
//b_val.val[0] = vmul_s16(a_val.val[0], tmp16.val[0]);
//// a0i*b0i|a1i*b1i|a2i*b2i|a3i*b3i
//b_val.val[1] = vmul_s16(a_val.val[1], tmp16.val[1]);
//c_val.val[0] = vqsub_s16(b_val.val[0], b_val.val[1]);

//// Multiply cross terms to get the imaginary result
//// a0r*b0i|a1r*b1i|a2r*b2i|a3r*b3i
//b_val.val[0] = vmul_s16(a_val.val[0], tmp16.val[1]);
//// a0i*b0r|a1i*b1r|a2i*b2r|a3i*b3r
//b_val.val[1] = vmul_s16(a_val.val[1], tmp16.val[0]);
//c_val.val[1] = vqadd_s16(b_val.val[0], b_val.val[1]);

//accumulator[n_vec].val[0] = vqadd_s16(accumulator[n_vec].val[0], c_val.val[0]);
//accumulator[n_vec].val[1] = vqadd_s16(accumulator[n_vec].val[1], c_val.val[1]);
//}
//// Regenerate phase
//if ((number % 256) == 0)
//{
//phase_est = arg_phase0 + (number + 1) * 4 * arg_phase_inc;

//*phase = lv_cmake(cos(phase_est), sin(phase_est));
//phase2 = (lv_32fc_t)(*phase) * phase_inc;
//phase3 = phase2 * phase_inc;
//phase4 = phase3 * phase_inc;

//__VOLK_ATTR_ALIGNED(16) float32_t ____phase_real[4] = { lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4) };
//__VOLK_ATTR_ALIGNED(16) float32_t ____phase_imag[4] = { lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4) };

//_phase_real = vld1q_f32(____phase_real);
//_phase_imag = vld1q_f32(____phase_imag);
//}
//}

//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//vst2_s16((int16_t*)dotProductVector, accumulator[n_vec]); // Store the results back into the dot product vector
//dotProduct = lv_cmake(0,0);
//for (i = 0; i < 4; ++i)
//{
//dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])),
//sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
//}
//_out[n_vec] = dotProduct;
//}
//volk_gnsssdr_free(accumulator);
//vst1q_f32((float32_t*)__phase_real, _phase_real);
//vst1q_f32((float32_t*)__phase_imag, _phase_imag);

//(*phase) = lv_cmake((float32_t)__phase_real[0], (float32_t)__phase_imag[0]);
//}

//for (n = neon_iters * 4; n < num_points; n++)
//{
//tmp16_ = in_common[n];  //printf("neon phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
//tmp32_ = lv_cmake((float32_t)lv_creal(tmp16_), (float32_t)lv_cimag(tmp16_)) * (*phase);
//tmp16_ = lv_cmake((int16_t)rintf(lv_creal(tmp32_)), (int16_t)rintf(lv_cimag(tmp32_)));
//(*phase) *= phase_inc;
//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//tmp = tmp16_ * in_a[n_vec][n];
//_out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
//}
//}
//}

//#endif [> LV_HAVE_NEON <]


//#ifdef LV_HAVE_NEON
//#include <arm_neon.h>
//#include <volk_gnsssdr/volk_gnsssdr_neon_intrinsics.h>

//static inline void volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_neon_vma(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const int16_t** in_a,  int num_a_vectors, unsigned int num_points)
//{
//const unsigned int neon_iters = num_points / 4;

//const int16_t** _in_a = in_a;
//const lv_16sc_t* _in_common = in_common;
//lv_16sc_t* _out = result;
//int n_vec;
//int i;
//unsigned int number;
//unsigned int n;
//lv_16sc_t tmp16_, tmp;
//lv_32fc_t tmp32_;

//if (neon_iters > 0)
//{
//lv_16sc_t dotProduct = lv_cmake(0,0);
//float arg_phase0 = cargf(*phase);
//float arg_phase_inc = cargf(phase_inc);
//float phase_est;
////printf("arg phase0: %f", arg_phase0);
//lv_32fc_t ___phase4 = phase_inc * phase_inc * phase_inc * phase_inc;
//__VOLK_ATTR_ALIGNED(16) float32_t __phase4_real[4] = { lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4) };
//__VOLK_ATTR_ALIGNED(16) float32_t __phase4_imag[4] = { lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4) };

//float32x4_t _phase4_real = vld1q_f32(__phase4_real);
//float32x4_t _phase4_imag = vld1q_f32(__phase4_imag);

//lv_32fc_t phase2 = (lv_32fc_t)(*phase) * phase_inc;
//lv_32fc_t phase3 = phase2 * phase_inc;
//lv_32fc_t phase4 = phase3 * phase_inc;

//__VOLK_ATTR_ALIGNED(16) float32_t __phase_real[4] = { lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4) };
//__VOLK_ATTR_ALIGNED(16) float32_t __phase_imag[4] = { lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4) };

//float32x4_t _phase_real = vld1q_f32(__phase_real);
//float32x4_t _phase_imag = vld1q_f32(__phase_imag);

//int16x4x2_t a_val, b_val;
//__VOLK_ATTR_ALIGNED(16) lv_16sc_t dotProductVector[4];
//float32x4_t half = vdupq_n_f32(0.5f);
//int16x4x2_t tmp16;
//int32x4x2_t tmp32i;

//float32x4x2_t tmp32f, tmp32_real, tmp32_imag;
//float32x4_t sign, PlusHalf, Round;

//int16x4x2_t* accumulator = (int16x4x2_t*)volk_gnsssdr_malloc(num_a_vectors * sizeof(int16x4x2_t), volk_gnsssdr_get_alignment());

//for(n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//accumulator[n_vec].val[0] = vdup_n_s16(0);
//accumulator[n_vec].val[1] = vdup_n_s16(0);
//}

//for(number = 0; number < neon_iters; number++)
//{
//[> load 4 complex numbers (int 16 bits each component) <]
//tmp16 = vld2_s16((int16_t*)_in_common);
//__VOLK_GNSSSDR_PREFETCH(_in_common + 8);
//_in_common += 4;

//[> promote them to int 32 bits <]
//tmp32i.val[0] = vmovl_s16(tmp16.val[0]);
//tmp32i.val[1] = vmovl_s16(tmp16.val[1]);

//[> promote them to float 32 bits <]
//tmp32f.val[0] = vcvtq_f32_s32(tmp32i.val[0]);
//tmp32f.val[1] = vcvtq_f32_s32(tmp32i.val[1]);

//[> complex multiplication of four complex samples (float 32 bits each component) <]
//tmp32_real.val[0] = vmulq_f32(tmp32f.val[0], _phase_real);
//tmp32_real.val[1] = vmulq_f32(tmp32f.val[1], _phase_imag);
//tmp32_imag.val[0] = vmulq_f32(tmp32f.val[0], _phase_imag);
//tmp32_imag.val[1] = vmulq_f32(tmp32f.val[1], _phase_real);

//tmp32f.val[0] = vsubq_f32(tmp32_real.val[0], tmp32_real.val[1]);
//tmp32f.val[1] = vaddq_f32(tmp32_imag.val[0], tmp32_imag.val[1]);

//[> downcast results to int32 <]
//[> in __aarch64__ we can do that with vcvtaq_s32_f32(ret1); vcvtaq_s32_f32(ret2); <]
//sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[0]), 31)));
//PlusHalf = vaddq_f32(tmp32f.val[0], half);
//Round = vsubq_f32(PlusHalf, sign);
//tmp32i.val[0] = vcvtq_s32_f32(Round);

//sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[1]), 31)));
//PlusHalf = vaddq_f32(tmp32f.val[1], half);
//Round = vsubq_f32(PlusHalf, sign);
//tmp32i.val[1] = vcvtq_s32_f32(Round);

//[> downcast results to int16 <]
//tmp16.val[0] = vqmovn_s32(tmp32i.val[0]);
//tmp16.val[1] = vqmovn_s32(tmp32i.val[1]);

//[> compute next four phases <]
//tmp32_real.val[0] = vmulq_f32(_phase_real, _phase4_real);
//tmp32_real.val[1] = vmulq_f32(_phase_imag, _phase4_imag);
//tmp32_imag.val[0] = vmulq_f32(_phase_real, _phase4_imag);
//tmp32_imag.val[1] = vmulq_f32(_phase_imag, _phase4_real);

//_phase_real = vsubq_f32(tmp32_real.val[0], tmp32_real.val[1]);
//_phase_imag = vaddq_f32(tmp32_imag.val[0], tmp32_imag.val[1]);

//// Regenerate phase
//if ((number % 256) == 0)
//{
////printf("computed phase: %f\n", cos(cargf(lv_cmake(_phase_real[0],_phase_imag[0]))));
//phase_est = arg_phase0 + (number + 1) * 4 * arg_phase_inc;
////printf("Estimated phase: %f\n\n", cos(phase_est));

//*phase = lv_cmake(cos(phase_est), sin(phase_est));
//phase2 = (lv_32fc_t)(*phase) * phase_inc;
//phase3 = phase2 * phase_inc;
//phase4 = phase3 * phase_inc;

//__VOLK_ATTR_ALIGNED(16) float32_t ____phase_real[4] = { lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4) };
//__VOLK_ATTR_ALIGNED(16) float32_t ____phase_imag[4] = { lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4) };

//_phase_real = vld1q_f32(____phase_real);
//_phase_imag = vld1q_f32(____phase_imag);

//// Round = vmulq_f32(_phase_real, _phase_real);
//// Round = vmlaq_f32(Round, _phase_imag, _phase_imag);
////               Round = vsqrtq_f32(Round);//printf("sqrt: %f \n", Round[0]);
////Round = vrsqrteq_f32(Round);printf("1/sqtr: %f  \n",Round[0]);
////Round = vrecpeq_f32((Round);
////              _phase_real = vdivq_f32(_phase_real, Round);
////              _phase_imag = vdivq_f32(_phase_imag, Round);
////_phase_real = vmulq_f32(_phase_real, Round);
////_phase_imag = vmulq_f32(_phase_imag, Round);
////printf("After  %i: %f,%f, %f\n\n", number, _phase_real[0], _phase_imag[0], sqrt(_phase_real[0]*_phase_real[0]+_phase_imag[0]*_phase_imag[0]));

//}

//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//a_val = vld2_s16((int16_t*)&(_in_a[n_vec][number*4]));

//b_val.val[0] = vmul_s16(a_val.val[0], tmp16.val[0]);
//b_val.val[1] = vmul_s16(a_val.val[1], tmp16.val[0]);

//// use multiply accumulate/subtract to get result
//b_val.val[0] = vmls_s16(b_val.val[0], a_val.val[1], tmp16.val[1]);
//b_val.val[1] = vmla_s16(b_val.val[1], a_val.val[0], tmp16.val[1]);

//accumulator[n_vec].val[0] = vqadd_s16(accumulator[n_vec].val[0], b_val.val[0]);
//accumulator[n_vec].val[1] = vqadd_s16(accumulator[n_vec].val[1], b_val.val[1]);
//}
//}

//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//vst2_s16((int16_t*)dotProductVector, accumulator[n_vec]); // Store the results back into the dot product vector
//dotProduct = lv_cmake(0,0);
//for (i = 0; i < 4; ++i)
//{
//dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])),
//sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
//}
//_out[n_vec] = dotProduct;
//}
//volk_gnsssdr_free(accumulator);

//vst1q_f32((float32_t*)__phase_real, _phase_real);
//vst1q_f32((float32_t*)__phase_imag, _phase_imag);

//(*phase) = lv_cmake((float32_t)__phase_real[0], (float32_t)__phase_imag[0]);
//}

//for (n = neon_iters * 4; n < num_points; n++)
//{
//tmp16_ = in_common[n];  //printf("neon phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
//tmp32_ = lv_cmake((float32_t)lv_creal(tmp16_), (float32_t)lv_cimag(tmp16_)) * (*phase);
//tmp16_ = lv_cmake((int16_t)rintf(lv_creal(tmp32_)), (int16_t)rintf(lv_cimag(tmp32_)));
//(*phase) *= phase_inc;
//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//tmp = tmp16_ * in_a[n_vec][n];
//_out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
//}
//}
//}

//#endif [> LV_HAVE_NEON <]


//#ifdef LV_HAVE_NEON
//#include <arm_neon.h>
//#include <volk_gnsssdr/volk_gnsssdr_neon_intrinsics.h>

//static inline void volk_gnsssdr_16ic_16i_rotator_dot_prod_16ic_xn_neon_optvma(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const int16_t** in_a,  int num_a_vectors, unsigned int num_points)
//{
//const unsigned int neon_iters = num_points / 4;

//const int16_t** _in_a = in_a;
//const lv_16sc_t* _in_common = in_common;
//lv_16sc_t* _out = result;
//int n_vec;
//int i;
//unsigned int number;
//unsigned int n;
//lv_16sc_t tmp16_, tmp;
//lv_32fc_t tmp32_;

//if (neon_iters > 0)
//{
//lv_16sc_t dotProduct = lv_cmake(0,0);
//float arg_phase0 = cargf(*phase);
//float arg_phase_inc = cargf(phase_inc);
//float phase_est;

//lv_32fc_t ___phase4 = phase_inc * phase_inc * phase_inc * phase_inc;
//__VOLK_ATTR_ALIGNED(16) float32_t __phase4_real[4] = { lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4) };
//__VOLK_ATTR_ALIGNED(16) float32_t __phase4_imag[4] = { lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4) };

//float32x4_t _phase4_real = vld1q_f32(__phase4_real);
//float32x4_t _phase4_imag = vld1q_f32(__phase4_imag);

//lv_32fc_t phase2 = (lv_32fc_t)(*phase) * phase_inc;
//lv_32fc_t phase3 = phase2 * phase_inc;
//lv_32fc_t phase4 = phase3 * phase_inc;

//__VOLK_ATTR_ALIGNED(16) float32_t __phase_real[4] = { lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4) };
//__VOLK_ATTR_ALIGNED(16) float32_t __phase_imag[4] = { lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4) };

//float32x4_t _phase_real = vld1q_f32(__phase_real);
//float32x4_t _phase_imag = vld1q_f32(__phase_imag);

//int16x4x2_t a_val, b_val;
//__VOLK_ATTR_ALIGNED(16) lv_16sc_t dotProductVector[4];
//float32x4_t half = vdupq_n_f32(0.5f);
//int32x4x2_t tmp32i;

//float32x4x2_t tmp32f, tmp32_real, tmp32_imag;
//float32x4_t sign, PlusHalf, Round;

//int16x4x2_t* accumulator1 = (int16x4x2_t*)volk_gnsssdr_malloc(num_a_vectors * sizeof(int16x4x2_t), volk_gnsssdr_get_alignment());
//int16x4x2_t* accumulator2 = (int16x4x2_t*)volk_gnsssdr_malloc(num_a_vectors * sizeof(int16x4x2_t), volk_gnsssdr_get_alignment());

//for(n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//accumulator1[n_vec].val[0] = vdup_n_s16(0);
//accumulator1[n_vec].val[1] = vdup_n_s16(0);
//accumulator2[n_vec].val[0] = vdup_n_s16(0);
//accumulator2[n_vec].val[1] = vdup_n_s16(0);
//}

//for(number = 0; number < neon_iters; number++)
//{
//[> load 4 complex numbers (int 16 bits each component) <]
//b_val = vld2_s16((int16_t*)_in_common);
//__VOLK_GNSSSDR_PREFETCH(_in_common + 8);
//_in_common += 4;

//[> promote them to int 32 bits <]
//tmp32i.val[0] = vmovl_s16(b_val.val[0]);
//tmp32i.val[1] = vmovl_s16(b_val.val[1]);

//[> promote them to float 32 bits <]
//tmp32f.val[0] = vcvtq_f32_s32(tmp32i.val[0]);
//tmp32f.val[1] = vcvtq_f32_s32(tmp32i.val[1]);

//[> complex multiplication of four complex samples (float 32 bits each component) <]
//tmp32_real.val[0] = vmulq_f32(tmp32f.val[0], _phase_real);
//tmp32_real.val[1] = vmulq_f32(tmp32f.val[1], _phase_imag);
//tmp32_imag.val[0] = vmulq_f32(tmp32f.val[0], _phase_imag);
//tmp32_imag.val[1] = vmulq_f32(tmp32f.val[1], _phase_real);

//tmp32f.val[0] = vsubq_f32(tmp32_real.val[0], tmp32_real.val[1]);
//tmp32f.val[1] = vaddq_f32(tmp32_imag.val[0], tmp32_imag.val[1]);

//[> downcast results to int32 <]
//[> in __aarch64__ we can do that with vcvtaq_s32_f32(ret1); vcvtaq_s32_f32(ret2); <]
//sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[0]), 31)));
//PlusHalf = vaddq_f32(tmp32f.val[0], half);
//Round = vsubq_f32(PlusHalf, sign);
//tmp32i.val[0] = vcvtq_s32_f32(Round);

//sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[1]), 31)));
//PlusHalf = vaddq_f32(tmp32f.val[1], half);
//Round = vsubq_f32(PlusHalf, sign);
//tmp32i.val[1] = vcvtq_s32_f32(Round);

//[> downcast results to int16 <]
//b_val.val[0] = vqmovn_s32(tmp32i.val[0]);
//b_val.val[1] = vqmovn_s32(tmp32i.val[1]);

//[> compute next four phases <]
//tmp32_real.val[0] = vmulq_f32(_phase_real, _phase4_real);
//tmp32_real.val[1] = vmulq_f32(_phase_imag, _phase4_imag);
//tmp32_imag.val[0] = vmulq_f32(_phase_real, _phase4_imag);
//tmp32_imag.val[1] = vmulq_f32(_phase_imag, _phase4_real);

//_phase_real = vsubq_f32(tmp32_real.val[0], tmp32_real.val[1]);
//_phase_imag = vaddq_f32(tmp32_imag.val[0], tmp32_imag.val[1]);

//// Regenerate phase
//if ((number % 256) == 0)
//{
////printf("computed phase: %f\n", cos(cargf(lv_cmake(_phase_real[0],_phase_imag[0]))));
//phase_est = arg_phase0 + (number + 1) * 4 * arg_phase_inc;
////printf("Estimated phase: %f\n\n", cos(phase_est));

//*phase = lv_cmake(cos(phase_est), sin(phase_est));
//phase2 = (lv_32fc_t)(*phase) * phase_inc;
//phase3 = phase2 * phase_inc;
//phase4 = phase3 * phase_inc;

//__VOLK_ATTR_ALIGNED(16) float32_t ____phase_real[4] = { lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4) };
//__VOLK_ATTR_ALIGNED(16) float32_t ____phase_imag[4] = { lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4) };

//_phase_real = vld1q_f32(____phase_real);
//_phase_imag = vld1q_f32(____phase_imag);
//}

//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//a_val = vld2_s16((int16_t*)&(_in_a[n_vec][number*4]));

//// use 2 accumulators to remove inter-instruction data dependencies
//accumulator1[n_vec].val[0] = vmla_s16(accumulator1[n_vec].val[0], a_val.val[0], b_val.val[0]);
//accumulator1[n_vec].val[1] = vmla_s16(accumulator1[n_vec].val[1], a_val.val[0], b_val.val[1]);
//accumulator2[n_vec].val[0] = vmls_s16(accumulator2[n_vec].val[0], a_val.val[1], b_val.val[1]);
//accumulator2[n_vec].val[1] = vmla_s16(accumulator2[n_vec].val[1], a_val.val[1], b_val.val[0]);
//}
//}
//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//accumulator1[n_vec].val[0] = vqadd_s16(accumulator1[n_vec].val[0], accumulator2[n_vec].val[0]);
//accumulator1[n_vec].val[1] = vqadd_s16(accumulator1[n_vec].val[1], accumulator2[n_vec].val[1]);
//}
//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//vst2_s16((int16_t*)dotProductVector, accumulator1[n_vec]); // Store the results back into the dot product vector
//dotProduct = lv_cmake(0,0);
//for (i = 0; i < 4; ++i)
//{
//dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])),
//sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
//}
//_out[n_vec] = dotProduct;
//}
//volk_gnsssdr_free(accumulator1);
//volk_gnsssdr_free(accumulator2);

//vst1q_f32((float32_t*)__phase_real, _phase_real);
//vst1q_f32((float32_t*)__phase_imag, _phase_imag);

//(*phase) = lv_cmake((float32_t)__phase_real[0], (float32_t)__phase_imag[0]);
//}

//for (n = neon_iters * 4; n < num_points; n++)
//{
//tmp16_ = in_common[n];  //printf("neon phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
//tmp32_ = lv_cmake((float32_t)lv_creal(tmp16_), (float32_t)lv_cimag(tmp16_)) * (*phase);
//tmp16_ = lv_cmake((int16_t)rintf(lv_creal(tmp32_)), (int16_t)rintf(lv_cimag(tmp32_)));
//(*phase) *= phase_inc;
//for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
//{
//tmp = tmp16_ * in_a[n_vec][n];
//_out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
//}
//}
//}

//#endif [> LV_HAVE_NEON <]

#endif /*INCLUDED_volk_gnsssdr_16ic_16i_dot_prod_16ic_xn_H*/
