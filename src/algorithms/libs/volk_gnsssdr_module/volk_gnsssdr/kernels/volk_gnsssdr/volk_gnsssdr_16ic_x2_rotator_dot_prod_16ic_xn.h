/*!
 * \file volk_gnsssdr_16ic_x2_dot_prod_16ic_xn.h
 * \brief Volk protokernel: multiplies N 16 bits vectors by a common vector
 * phase rotated and accumulates the results in N 16 bits short complex outputs.
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * Volk protokernel that multiplies N 16 bits vectors by a common vector, which is
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

#ifndef INCLUDED_volk_gnsssdr_16ic_xn_rotator_dot_prod_16ic_xn_H
#define INCLUDED_volk_gnsssdr_16ic_xn_rotator_dot_prod_16ic_xn_H


#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/saturation_arithmetic.h>
#include <math.h>
#include <stdio.h>

#ifdef LV_HAVE_GENERIC
/*!
 \brief Rotates and multiplies the reference complex vector with multiple versions of another complex vector, accumulates the results and stores them in the output vector
 \param[out]    result        Array of num_a_vectors components with the multiple versions of in_a multiplied and accumulated The vector where the accumulated result will be stored
 \param[in]     in_common     Pointer to one of the vectors to be rotated, multiplied and accumulated (reference vector)
 \param[in]     phase_inc     Phase increment = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad))
 \param[in,out] phase         Initial / final phase
 \param[in]     in_a          Pointer to an array of pointers to multiple versions of the other vector to be multiplied and accumulated
 \param[in]     num_a_vectors Number of vectors to be multiplied by the reference vector and accumulated
 \param[in]     num_points    The Number of complex values to be multiplied together, accumulated and stored into result
 */
static inline void volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_generic(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_16sc_t** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;
    for (int n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            result[n_vec] = lv_cmake(0,0);
        }
    for (unsigned int n = 0; n < num_points; n++)
        {
            tmp16 = *in_common++; if(n<10 || n >= 8108) printf("generic phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;
            for (int n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
                    //lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                    result[n_vec] = lv_cmake(sat_adds16i(lv_creal(result[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(result[n_vec]), lv_cimag(tmp)));
                }
        }
}

#endif /*LV_HAVE_GENERIC*/


#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

/*!
 \brief Rotates and multiplies the reference complex vector with multiple versions of another complex vector, accumulates the results and stores them in the output vector
 \param[out]    result        Array of num_a_vectors components with the multiple versions of in_a multiplied and accumulated The vector where the accumulated result will be stored
 \param[in]     in_common     Pointer to one of the vectors to be rotated, multiplied and accumulated (reference vector)
 \param[in]     phase_inc     Phase increment = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad))
 \param[in,out] phase         Initial / final phase
 \param[in]     in_a          Pointer to an array of pointers to multiple versions of the other vector to be multiplied and accumulated
 \param[in]     num_a_vectors Number of vectors to be multiplied by the reference vector and accumulated
 \param[in]     num_points    The Number of complex values to be multiplied together, accumulated and stored into result
 */
static inline void volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_a_sse3(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_16sc_t** in_a,  int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake(0,0);

    const unsigned int sse_iters = num_points / 4;

    const lv_16sc_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;

    __VOLK_ATTR_ALIGNED(16) lv_16sc_t dotProductVector[4];

    //todo dyn mem reg

    __m128i* realcacc;
    __m128i* imagcacc;

    realcacc = (__m128i*)calloc(num_a_vectors, sizeof(__m128i)); //calloc also sets memory to 0
    imagcacc = (__m128i*)calloc(num_a_vectors, sizeof(__m128i)); //calloc also sets memory to 0

    __m128i a, b, c, c_sr, mask_imag, mask_real, real, imag, imag1, imag2, b_sl, a_sl, results;

    mask_imag = _mm_set_epi8(255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0);
    mask_real = _mm_set_epi8(0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255);

    // phase rotation registers
    __m128 pa, pb, two_phase_acc_reg, two_phase_inc_reg;
    __m128i pc1, pc2;
    __attribute__((aligned(16))) lv_32fc_t two_phase_inc[2];
    two_phase_inc[0] = phase_inc * phase_inc;
    two_phase_inc[1] = phase_inc * phase_inc;
    two_phase_inc_reg = _mm_load_ps((float*) two_phase_inc);
    __attribute__((aligned(16))) lv_32fc_t two_phase_acc[2];
    two_phase_acc[0] = (*phase);
    two_phase_acc[1] = (*phase) * phase_inc;
    printf("a_sse phase %i: %f,%f\n", 0,lv_creal(two_phase_acc[0]),lv_cimag(two_phase_acc[0]));
    printf("a_sse phase %i: %f,%f\n", 1,lv_creal(two_phase_acc[1]),lv_cimag(two_phase_acc[1]));
    two_phase_acc_reg = _mm_load_ps((float*)two_phase_acc);
    __m128 yl, yh, tmp1, tmp2, tmp3;
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;

    for(unsigned int number = 0; number < sse_iters; number++)
        {
            // Phase rotation on operand in_common starts here:
            //printf("generic phase %i: %f,%f\n", n*4,lv_creal(*phase),lv_cimag(*phase));
            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0]))); // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            //complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1); // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc1 = _mm_cvtps_epi32(pb); // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1); // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            //next two samples
            _in_common += 2;
            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0]))); // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            //complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1); // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc2 = _mm_cvtps_epi32(pb); // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1); // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // store four rotated in_common samples in the register b
            b = _mm_packs_epi32(pc1, pc2);// convert from 32ic to 16ic

            //next two samples
            _in_common += 2;

            for (int n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    a = _mm_load_si128((__m128i*)&(_in_a[n_vec][number*4])); //load (2 byte imag, 2 byte real) x 4 into 128 bits reg

                    c = _mm_mullo_epi16(a, b); // a3.i*b3.i, a3.r*b3.r, ....

                    c_sr = _mm_srli_si128(c, 2); // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
                    real = _mm_subs_epi16(c, c_sr);

                    b_sl = _mm_slli_si128(b, 2); // b3.r, b2.i ....
                    a_sl = _mm_slli_si128(a, 2); // a3.r, a2.i ....

                    imag1 = _mm_mullo_epi16(a, b_sl); // a3.i*b3.r, ....
                    imag2 = _mm_mullo_epi16(b, a_sl); // b3.i*a3.r, ....

                    imag = _mm_adds_epi16(imag1, imag2);

                    realcacc[n_vec] = _mm_adds_epi16 (realcacc[n_vec], real);
                    imagcacc[n_vec] = _mm_adds_epi16 (imagcacc[n_vec], imag);
                }
        }

    for (int n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            realcacc[n_vec] = _mm_and_si128(realcacc[n_vec], mask_real);
            imagcacc[n_vec] = _mm_and_si128(imagcacc[n_vec], mask_imag);

            results = _mm_or_si128(realcacc[n_vec], imagcacc[n_vec]);

            _mm_store_si128((__m128i*)dotProductVector, results); // Store the results back into the dot product vector
            dotProduct = lv_cmake(0,0);
            for (int i = 0; i < 4; ++i)
                {
                    dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])),
                            sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
                }
            _out[n_vec] = dotProduct;
        }
    free(realcacc);
    free(imagcacc);

    _mm_store_ps((float*)two_phase_acc, two_phase_acc_reg);
    //(*phase) = lv_cmake((float*)two_phase_acc[0], (float*)two_phase_acc[1]);
    (*phase) = two_phase_acc[0];

    for(unsigned int n  = sse_iters * 4; n < num_points; n++)
        {
            tmp16 = *in_common++;  printf("a_sse phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;

            for (int n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
                    //lv_16sc_t tmp = lv_cmake(sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_creal(in_a[n_vec][n])), - sat_muls16i(lv_cimag(tmp16), lv_cimag(in_a[n_vec][n]))) , sat_adds16i(sat_muls16i(lv_creal(tmp16), lv_cimag(in_a[n_vec][n])), sat_muls16i(lv_cimag(tmp16), lv_creal(in_a[n_vec][n]))));
                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                            sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }

}
#endif /* LV_HAVE_SSE3 */

#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>

/*!
 \brief Rotates and multiplies the reference complex vector with multiple versions of another complex vector, accumulates the results and stores them in the output vector
 \param[out]    result        Array of num_a_vectors components with the multiple versions of in_a multiplied and accumulated The vector where the accumulated result will be stored
 \param[in]     in_common     Pointer to one of the vectors to be rotated, multiplied and accumulated (reference vector)
 \param[in]     phase_inc     Phase increment = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad))
 \param[in,out] phase         Initial / final phase
 \param[in]     in_a          Pointer to an array of pointers to multiple versions of the other vector to be multiplied and accumulated
 \param[in]     num_a_vectors Number of vectors to be multiplied by the reference vector and accumulated
 \param[in]     num_points    The Number of complex values to be multiplied together, accumulated and stored into result
 */
static inline void volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_u_sse3(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_16sc_t** in_a,  int num_a_vectors, unsigned int num_points)
{
    lv_16sc_t dotProduct = lv_cmake(0,0);

    const unsigned int sse_iters = num_points / 4;

    const lv_16sc_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;

    lv_16sc_t* _out = result;
    __VOLK_ATTR_ALIGNED(16) lv_16sc_t dotProductVector[4];

    //todo dyn mem reg

    __m128i* realcacc;
    __m128i* imagcacc;

    realcacc = (__m128i*)calloc(num_a_vectors, sizeof(__m128i)); //calloc also sets memory to 0
    imagcacc = (__m128i*)calloc(num_a_vectors, sizeof(__m128i)); //calloc also sets memory to 0

    __m128i a, b, c, c_sr, mask_imag, mask_real, real, imag, imag1, imag2, b_sl, a_sl, results;


    mask_imag = _mm_set_epi8(255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0);
    mask_real = _mm_set_epi8(0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255);

    // phase rotation registers
    __m128 pa, pb, two_phase_acc_reg, two_phase_inc_reg;
    __m128i pc1, pc2;
    __attribute__((aligned(16))) lv_32fc_t two_phase_inc[2];
    two_phase_inc[0] = phase_inc * phase_inc;
    two_phase_inc[1] = phase_inc * phase_inc;
    two_phase_inc_reg = _mm_load_ps((float*) two_phase_inc);
    __attribute__((aligned(16))) lv_32fc_t two_phase_acc[2];
    two_phase_acc[0] = (*phase);
    two_phase_acc[1] = (*phase) * phase_inc;
    two_phase_acc_reg = _mm_load_ps((float*)two_phase_acc);
    __m128 yl, yh, tmp1, tmp2, tmp3;
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;

    for(unsigned int number = 0; number < sse_iters; number++)
        {
            // Phase rotation on operand in_common starts here:

            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0]))); // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            //complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1); // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc1 = _mm_cvtps_epi32(pb); // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1); // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            //next two samples
            _in_common += 2;
            pa = _mm_set_ps((float)(lv_cimag(_in_common[1])), (float)(lv_creal(_in_common[1])), (float)(lv_cimag(_in_common[0])), (float)(lv_creal(_in_common[0]))); // //load (2 byte imag, 2 byte real) x 2 into 128 bits reg
            //complex 32fc multiplication b=a*two_phase_acc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(pa, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            pa = _mm_shuffle_ps(pa, pa, 0xB1); // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(pa, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            pb = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            pc2 = _mm_cvtps_epi32(pb); // convert from 32fc to 32ic

            //complex 32fc multiplication two_phase_acc_reg=two_phase_acc_reg*two_phase_inc_reg
            yl = _mm_moveldup_ps(two_phase_acc_reg); // Load yl with cr,cr,dr,dr
            yh = _mm_movehdup_ps(two_phase_acc_reg); // Load yh with ci,ci,di,di
            tmp1 = _mm_mul_ps(two_phase_inc_reg, yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            tmp3 = _mm_shuffle_ps(two_phase_inc_reg, two_phase_inc_reg, 0xB1); // Re-arrange x to be ai,ar,bi,br
            tmp2 = _mm_mul_ps(tmp3, yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            two_phase_acc_reg = _mm_addsub_ps(tmp1, tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di

            // store four rotated in_common samples in the register b
            b = _mm_packs_epi32(pc1, pc2);// convert from 32ic to 16ic

            //next two samples
            _in_common += 2;

            for (int n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    a = _mm_loadu_si128((__m128i*)&(_in_a[n_vec][number*4])); //load (2 byte imag, 2 byte real) x 4 into 128 bits reg

                    c = _mm_mullo_epi16(a, b); // a3.i*b3.i, a3.r*b3.r, ....

                    c_sr = _mm_srli_si128(c, 2); // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
                    real = _mm_subs_epi16(c, c_sr);

                    b_sl = _mm_slli_si128(b, 2); // b3.r, b2.i ....
                    a_sl = _mm_slli_si128(a, 2); // a3.r, a2.i ....

                    imag1 = _mm_mullo_epi16(a, b_sl); // a3.i*b3.r, ....
                    imag2 = _mm_mullo_epi16(b, a_sl); // b3.i*a3.r, ....

                    imag = _mm_adds_epi16(imag1, imag2);

                    realcacc[n_vec] = _mm_adds_epi16(realcacc[n_vec], real);
                    imagcacc[n_vec] = _mm_adds_epi16(imagcacc[n_vec], imag);
                }
        }

    for (int n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            realcacc[n_vec] = _mm_and_si128 (realcacc[n_vec], mask_real);
            imagcacc[n_vec] = _mm_and_si128 (imagcacc[n_vec], mask_imag);

            results = _mm_or_si128(realcacc[n_vec], imagcacc[n_vec]);

            _mm_storeu_si128((__m128i*)dotProductVector, results); // Store the results back into the dot product vector
            dotProduct = lv_cmake(0,0);
            for (int i = 0; i < 4; ++i)
                {
                    dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])),
                            sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
                }
            _out[n_vec] = dotProduct;
        }
    free(realcacc);
    free(imagcacc);

    _mm_store_ps((float*)two_phase_acc, two_phase_acc_reg);
    (*phase) = two_phase_acc[0];//lv_cmake(two_phase_acc[0], two_phase_acc[1]);


    for(unsigned int n  = sse_iters * 4; n < num_points; n++)
        {
            tmp16 = *in_common++;
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            tmp16 = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;
            for (int n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    lv_16sc_t tmp = tmp16 * in_a[n_vec][n];
                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)),
                            sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}
#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_NEON
#include <arm_neon.h>

/*!
 \brief Rotates and multiplies the reference complex vector with multiple versions of another complex vector, accumulates the results and stores them in the output vector
 \param[out]    result        Array of num_a_vectors components with the multiple versions of in_a multiplied and accumulated The vector where the accumulated result will be stored
 \param[in]     in_common     Pointer to one of the vectors to be rotated, multiplied and accumulated (reference vector)
 \param[in]     phase_inc     Phase increment = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad))
 \param[in,out] phase         Initial / final phase
 \param[in]     in_a          Pointer to an array of pointers to multiple versions of the other vector to be multiplied and accumulated
 \param[in]     num_a_vectors Number of vectors to be multiplied by the reference vector and accumulated
 \param[in]     num_points    The Number of complex values to be multiplied together, accumulated and stored into result
 */
static inline void volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_neon(lv_16sc_t* result, const lv_16sc_t* in_common, const lv_32fc_t phase_inc, lv_32fc_t* phase, const lv_16sc_t** in_a,  int num_a_vectors, unsigned int num_points)
{
    const unsigned int neon_iters = num_points / 4;

    const lv_16sc_t** _in_a = in_a;
    const lv_16sc_t* _in_common = in_common;
    lv_16sc_t* _out = result;

    lv_16sc_t tmp16_, tmp;
    lv_32fc_t tmp32_;

    if (neon_iters > 0)
        {
            lv_16sc_t dotProduct = lv_cmake(0,0);

            lv_32fc_t ___phase4 = phase_inc * phase_inc * phase_inc * phase_inc;
            __VOLK_ATTR_ALIGNED(16) float32_t __phase4_real[4] = { lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4) };
            __VOLK_ATTR_ALIGNED(16) float32_t __phase4_imag[4] = { lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4) };

            float32x4_t _phase4_real = vld1q_f32(__phase4_real);
            float32x4_t _phase4_imag = vld1q_f32(__phase4_imag);

            lv_32fc_t phase2 = (lv_32fc_t)(*phase) * phase_inc;
            lv_32fc_t phase3 = phase2 * phase_inc;
            lv_32fc_t phase4 = phase3 * phase_inc;

            __VOLK_ATTR_ALIGNED(16) float32_t __phase_real[4] = { lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4) };
            __VOLK_ATTR_ALIGNED(16) float32_t __phase_imag[4] = { lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4) };

            float32x4_t _phase_real = vld1q_f32(__phase_real);
            float32x4_t _phase_imag = vld1q_f32(__phase_imag);

            int16x4x2_t a_val, b_val, c_val;
            __VOLK_ATTR_ALIGNED(16) lv_16sc_t dotProductVector[4];
            float32x4_t half = vdupq_n_f32(0.5f);
            int16x4x2_t tmp16;
            int32x4x2_t tmp32i;

            float32x4x2_t tmp32f, tmp32_real, tmp32_imag;
            float32x4_t sign, PlusHalf, Round;

            int16x4x2_t* accumulator;
            accumulator = (int16x4x2_t*)calloc(num_a_vectors, sizeof(int16x4x2_t));

            for(int n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    accumulator[n_vec].val[0] = vdup_n_s16(0);
                    accumulator[n_vec].val[1] = vdup_n_s16(0);
                }

            for(unsigned int number = 0; number < neon_iters; number++)
                {
                    /* load 4 complex numbers (int 16 bits each component) */
                    tmp16 = vld2_s16((int16_t*)_in_common);
                    __builtin_prefetch(_in_common + 8);
                    _in_common += 4;

                    /* promote them to int 32 bits */
                    tmp32i.val[0] = vmovl_s16(tmp16.val[0]);
                    tmp32i.val[1] = vmovl_s16(tmp16.val[1]);

                    /* promote them to float 32 bits */
                    tmp32f.val[0] = vcvtq_f32_s32(tmp32i.val[0]);
                    tmp32f.val[1] = vcvtq_f32_s32(tmp32i.val[1]);

                    /* complex multiplication of four complex samples (float 32 bits each component) */
                    tmp32_real.val[0] = vmulq_f32(tmp32f.val[0], _phase_real);
                    tmp32_real.val[1] = vmulq_f32(tmp32f.val[1], _phase_imag);
                    tmp32_imag.val[0] = vmulq_f32(tmp32f.val[0], _phase_imag);
                    tmp32_imag.val[1] = vmulq_f32(tmp32f.val[1], _phase_real);

                    tmp32f.val[0] = vsubq_f32(tmp32_real.val[0], tmp32_real.val[1]);
                    tmp32f.val[1] = vaddq_f32(tmp32_imag.val[0], tmp32_imag.val[1]);

                    /* downcast results to int32 */
                    /* in __aarch64__ we can do that with vcvtaq_s32_f32(ret1); vcvtaq_s32_f32(ret2); */
                    sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[0]), 31)));
                    PlusHalf = vaddq_f32(tmp32f.val[0], half);
                    Round = vsubq_f32(PlusHalf, sign);
                    tmp32i.val[0] = vcvtq_s32_f32(Round);

                    sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[1]), 31)));
                    PlusHalf = vaddq_f32(tmp32f.val[1], half);
                    Round = vsubq_f32(PlusHalf, sign);
                    tmp32i.val[1] = vcvtq_s32_f32(Round);

                    /* downcast results to int16 */
                    tmp16.val[0] = vqmovn_s32(tmp32i.val[0]);
                    tmp16.val[1] = vqmovn_s32(tmp32i.val[1]);

                    /* compute next four phases */
                    tmp32_real.val[0] = vmulq_f32(_phase_real, _phase4_real);
                    tmp32_real.val[1] = vmulq_f32(_phase_imag, _phase4_imag);
                    tmp32_imag.val[0] = vmulq_f32(_phase_real, _phase4_imag);
                    tmp32_imag.val[1] = vmulq_f32(_phase_imag, _phase4_real);

                    _phase_real = vsubq_f32(tmp32_real.val[0], tmp32_real.val[1]);
                    _phase_imag = vaddq_f32(tmp32_imag.val[0], tmp32_imag.val[1]);

                    for (int n_vec = 0; n_vec < num_a_vectors; n_vec++)
                        {
                            a_val = vld2_s16((int16_t*)&(_in_a[n_vec][number*4])); //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
                            __builtin_prefetch(&_in_a[n_vec][number*4] + 8);

                            // multiply the real*real and imag*imag to get real result
                            // a0r*b0r|a1r*b1r|a2r*b2r|a3r*b3r
                            b_val.val[0] = vmul_s16(a_val.val[0], tmp16.val[0]);
                            // a0i*b0i|a1i*b1i|a2i*b2i|a3i*b3i
                            b_val.val[1] = vmul_s16(a_val.val[1], tmp16.val[1]);
                            c_val.val[0] = vqsub_s16(b_val.val[0], b_val.val[1]);

                            // Multiply cross terms to get the imaginary result
                            // a0r*b0i|a1r*b1i|a2r*b2i|a3r*b3i
                            b_val.val[0] = vmul_s16(a_val.val[0], tmp16.val[1]);
                            // a0i*b0r|a1i*b1r|a2i*b2r|a3i*b3r
                            b_val.val[1] = vmul_s16(a_val.val[1], tmp16.val[0]);
                            c_val.val[1] = vqadd_s16(b_val.val[0], b_val.val[1]);

                            accumulator[n_vec].val[0] = vqadd_s16(accumulator[n_vec].val[0], c_val.val[0]);
                            accumulator[n_vec].val[1] = vqadd_s16(accumulator[n_vec].val[1], c_val.val[1]);
                        }
                }

            for (int n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    vst2_s16((int16_t*)dotProductVector, accumulator[n_vec]); // Store the results back into the dot product vector
                    dotProduct = lv_cmake(0,0);
                    for (int i = 0; i < 4; ++i)
                        {
                            dotProduct = lv_cmake(sat_adds16i(lv_creal(dotProduct), lv_creal(dotProductVector[i])),
                                    sat_adds16i(lv_cimag(dotProduct), lv_cimag(dotProductVector[i])));
                        }
                    _out[n_vec] = dotProduct;
                }
            free(accumulator);
            vst1q_f32((float32_t*)__phase_real, _phase_real);
            vst1q_f32((float32_t*)__phase_imag, _phase_imag);

            (*phase) = lv_cmake((float32_t)__phase_real[0], (float32_t)__phase_imag[0]);
        }

    for (unsigned int n = neon_iters * 4; n < num_points; n++)
        {
            tmp16_ = in_common[n];  printf("neon phase %i: %f,%f\n", n,lv_creal(*phase),lv_cimag(*phase));
            tmp32_ = lv_cmake((float32_t)lv_creal(tmp16_), (float32_t)lv_cimag(tmp16_)) * (*phase);
            tmp16_ = lv_cmake((int16_t)rintf(lv_creal(tmp32_)), (int16_t)rintf(lv_cimag(tmp32_)));
            (*phase) *= phase_inc;
            for (int n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    tmp = tmp16_ * in_a[n_vec][n];
                    _out[n_vec] = lv_cmake(sat_adds16i(lv_creal(_out[n_vec]), lv_creal(tmp)), sat_adds16i(lv_cimag(_out[n_vec]), lv_cimag(tmp)));
                }
        }
}

#endif /* LV_HAVE_NEON */

#endif /*INCLUDED_volk_gnsssdr_16ic_xn_dot_prod_16ic_xn_H*/
