/*!
 * \file volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5.h
 * \brief Volk protokernel: performs the carrier wipe-off mixing and the Very early, Early, Prompt, Late and very late correlation with 16 bits vectors using different methods: inside u_sse4_1_first there is one method, inside u_sse4_1_second there is another... This protokernel has been created to test the performance of different methods.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that performs the carrier wipe-off mixing and the
 * Very early, Early, Prompt, Late and very late correlation with 16 bits vectors (8 bits the
 * real part and 8 bits the imaginary part), and accumulates the result 
 * in 32 bits single point values, returning float32 values:
 * - The carrier wipe-off is done by multiplying the input signal by the
 * carrier (multiplication of 16 bits vectors) It returns the input
 * signal in base band (BB)
 * - Very Early values are calculated by multiplying the input signal in BB by the
 * very early code (multiplication of 16 bits vectors), accumulating the results into float32 values
 * - Early values are calculated by multiplying the input signal in BB by the
 * early code (multiplication of 16 bits vectors), accumulating the results into float32 values
 * - Prompt values are calculated by multiplying the input signal in BB by the
 * prompt code (multiplication of 16 bits vectors), accumulating the results into float32 values
 * - Late values are calculated by multiplying the input signal in BB by the
 * late code (multiplication of 16 bits vectors), accumulating the results into float32 values
 * - Very Late values are calculated by multiplying the input signal in BB by the
 * very late code (multiplication of 16 bits vectors), accumulating the results into float32 values
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

#ifndef INCLUDED_gnsssdr_volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5_u_H
#define INCLUDED_gnsssdr_volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5_u_H

#include <inttypes.h>
#include <stdio.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <float.h>
#include <string.h>

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
#include "CommonMacros/CommonMacros_8ic_cw_epl_corr_32fc.h"
#include "CommonMacros/CommonMacros.h"
/*!
 \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param VE_code Very Early PRN code replica input
 \param E_code Early PRN code replica input
 \param P_code Prompt PRN code replica input
 \param L_code Late PRN code replica input
 \param VL_code Very Late PRN code replica input
 \param VE_out Very Early correlation output
 \param E_out Early correlation output
 \param P_out Prompt correlation output
 \param L_out Late correlation output
 \param VL_out Very Late correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5_u_sse4_1_first(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* VE_code, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, const lv_8sc_t* VL_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    __m128i x, y, real_bb_signal_sample, imag_bb_signal_sample;
    __m128i mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, output, real_output, imag_output;

    __m128 VE_code_acc, E_code_acc, P_code_acc, L_code_acc, VL_code_acc;
    __m128i input_i_1, input_i_2, output_i32;
    __m128 output_ps_1, output_ps_2;

    const lv_8sc_t* input_ptr = input;
    const lv_8sc_t* carrier_ptr = carrier;

    const lv_8sc_t* VE_code_ptr = VE_code;
    lv_32fc_t* VE_out_ptr = VE_out;
    const lv_8sc_t* E_code_ptr = E_code;
    lv_32fc_t* E_out_ptr = E_out;
    const lv_8sc_t* P_code_ptr = P_code;
    lv_32fc_t* P_out_ptr = P_out;
    const lv_8sc_t* L_code_ptr = L_code;
    lv_32fc_t* L_out_ptr = L_out;
    const lv_8sc_t* VL_code_ptr = VL_code;
    lv_32fc_t* VL_out_ptr = VL_out;

    *VE_out_ptr = 0;
    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;
    *VL_out_ptr = 0;

    mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);

    VE_code_acc = _mm_setzero_ps();
    E_code_acc = _mm_setzero_ps();
    P_code_acc = _mm_setzero_ps();
    L_code_acc = _mm_setzero_ps();
    VL_code_acc = _mm_setzero_ps();

    if (sse_iters>0)
        {
            for(int number = 0;number < sse_iters; number++)
                {
                    //Perform the carrier wipe-off
                    x = _mm_lddqu_si128((__m128i*)input_ptr);
                    y = _mm_lddqu_si128((__m128i*)carrier_ptr);

                    imagx = _mm_srli_si128 (x, 1);
                    imagx = _mm_and_si128 (imagx, mult1);
                    realx = _mm_and_si128 (x, mult1);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (realx, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imagx, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (realx, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imagx, realy);

                    real_bb_signal_sample = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_bb_signal_sample = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    //Get very early values
                    y = _mm_lddqu_si128((__m128i*)VE_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    output = _mm_blendv_epi8 (imag_output, real_output, mult1);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_1 = _mm_cvtepi32_ps(output_i32);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_2 = _mm_cvtepi32_ps(output_i32);

                    VE_code_acc = _mm_add_ps (VE_code_acc, output_ps_1);
                    VE_code_acc = _mm_add_ps (VE_code_acc, output_ps_2);

                    //Get early values
                    y = _mm_lddqu_si128((__m128i*)E_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    output = _mm_blendv_epi8 (imag_output, real_output, mult1);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_1 = _mm_cvtepi32_ps(output_i32);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_2 = _mm_cvtepi32_ps(output_i32);

                    E_code_acc = _mm_add_ps (E_code_acc, output_ps_1);
                    E_code_acc = _mm_add_ps (E_code_acc, output_ps_2);

                    //Get prompt values
                    y = _mm_lddqu_si128((__m128i*)P_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    output = _mm_blendv_epi8 (imag_output, real_output, mult1);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_1 = _mm_cvtepi32_ps(output_i32);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_2 = _mm_cvtepi32_ps(output_i32);

                    P_code_acc = _mm_add_ps (P_code_acc, output_ps_1);
                    P_code_acc = _mm_add_ps (P_code_acc, output_ps_2);

                    //Get late values
                    y = _mm_lddqu_si128((__m128i*)L_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    output = _mm_blendv_epi8 (imag_output, real_output, mult1);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_1 = _mm_cvtepi32_ps(output_i32);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_2 = _mm_cvtepi32_ps(output_i32);

                    L_code_acc = _mm_add_ps (L_code_acc, output_ps_1);
                    L_code_acc = _mm_add_ps (L_code_acc, output_ps_2);

                    //Get very late values
                    y = _mm_lddqu_si128((__m128i*)VL_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    output = _mm_blendv_epi8 (imag_output, real_output, mult1);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_1 = _mm_cvtepi32_ps(output_i32);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_2 = _mm_cvtepi32_ps(output_i32);

                    VL_code_acc = _mm_add_ps (VL_code_acc, output_ps_1);
                    VL_code_acc = _mm_add_ps (VL_code_acc, output_ps_2);

                    input_ptr += 8;
                    carrier_ptr += 8;
                    VE_code_ptr += 8;
                    E_code_ptr += 8;
                    P_code_ptr += 8;
                    L_code_ptr += 8;
                    VL_code_ptr += 8;
                }

            __VOLK_ATTR_ALIGNED(16) lv_32fc_t VE_dotProductVector[2];
            __VOLK_ATTR_ALIGNED(16) lv_32fc_t E_dotProductVector[2];
            __VOLK_ATTR_ALIGNED(16) lv_32fc_t P_dotProductVector[2];
            __VOLK_ATTR_ALIGNED(16) lv_32fc_t L_dotProductVector[2];
            __VOLK_ATTR_ALIGNED(16) lv_32fc_t VL_dotProductVector[2];

            _mm_storeu_ps((float*)VE_dotProductVector,VE_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)E_dotProductVector,E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)P_dotProductVector,P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)L_dotProductVector,L_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)VL_dotProductVector,VL_code_acc); // Store the results back into the dot product vector

            for (int i = 0; i<2; ++i)
                {
                    *VE_out_ptr += VE_dotProductVector[i];
                    *E_out_ptr += E_dotProductVector[i];
                    *P_out_ptr += P_dotProductVector[i];
                    *L_out_ptr += L_dotProductVector[i];
                    *VL_out_ptr += VL_dotProductVector[i];
                }
        }

    lv_8sc_t bb_signal_sample;
    for(int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get very early, early, prompt, late and very late values for each
            *VE_out_ptr += (lv_32fc_t) (bb_signal_sample * (*VE_code_ptr++));
            *E_out_ptr += (lv_32fc_t) (bb_signal_sample * (*E_code_ptr++));
            *P_out_ptr += (lv_32fc_t) (bb_signal_sample * (*P_code_ptr++));
            *L_out_ptr += (lv_32fc_t) (bb_signal_sample * (*L_code_ptr++));
            *VL_out_ptr += (lv_32fc_t) (bb_signal_sample * (*VL_code_ptr++));
        }
}
#endif /* LV_HAVE_SSE4_1 */

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
#include "CommonMacros/CommonMacros_8ic_cw_epl_corr_32fc.h"
#include "CommonMacros/CommonMacros.h"
/*!
 \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param VE_code Very Early PRN code replica input
 \param E_code Early PRN code replica input
 \param P_code Prompt PRN code replica input
 \param L_code Late PRN code replica input
 \param VL_code Very Late PRN code replica input
 \param VE_out Very Early correlation output
 \param E_out Early correlation output
 \param P_out Prompt correlation output
 \param L_out Late correlation output
 \param VL_out Very Late correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5_u_sse4_1_second(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* VE_code, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, const lv_8sc_t* VL_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    __m128i x, x_abs, y, y_aux, bb_signal_sample_aux, bb_signal_sample_aux_abs;;
    __m128i mult1, output, real_output, imag_output;

    __m128 VE_code_acc, E_code_acc, P_code_acc, L_code_acc, VL_code_acc;
    __m128i input_i_1, input_i_2, output_i32;
    __m128 output_ps_1, output_ps_2;

    __m128i check_sign_sequence = _mm_set_epi8 (255, 1, 255, 1, 255, 1, 255, 1, 255, 1, 255, 1, 255, 1, 255, 1);

    const lv_8sc_t* input_ptr = input;
    const lv_8sc_t* carrier_ptr = carrier;

    const lv_8sc_t* VE_code_ptr = VE_code;
    lv_32fc_t* VE_out_ptr = VE_out;
    const lv_8sc_t* E_code_ptr = E_code;
    lv_32fc_t* E_out_ptr = E_out;
    const lv_8sc_t* P_code_ptr = P_code;
    lv_32fc_t* P_out_ptr = P_out;
    const lv_8sc_t* L_code_ptr = L_code;
    lv_32fc_t* L_out_ptr = L_out;
    const lv_8sc_t* VL_code_ptr = VL_code;
    lv_32fc_t* VL_out_ptr = VL_out;

    *VE_out_ptr = 0;
    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;
    *VL_out_ptr = 0;

    mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);

    VE_code_acc = _mm_setzero_ps();
    E_code_acc = _mm_setzero_ps();
    P_code_acc = _mm_setzero_ps();
    L_code_acc = _mm_setzero_ps();
    VL_code_acc = _mm_setzero_ps();

    if (sse_iters>0)
        {
            for(int number = 0;number < sse_iters; number++)
                {
                    //Perform the carrier wipe-off
                    x = _mm_lddqu_si128((__m128i*)input_ptr);
                    y = _mm_lddqu_si128((__m128i*)carrier_ptr);

                    x_abs = _mm_abs_epi8 (x);

                    y_aux = _mm_sign_epi8 (y, x);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (x_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, _mm_set_epi8 (14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1));
                    y_aux = _mm_sign_epi8 (y_aux, x);
                    imag_output = _mm_maddubs_epi16 (x_abs, y_aux);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    bb_signal_sample_aux = _mm_blendv_epi8 (imag_output, real_output, mult1);

                    bb_signal_sample_aux_abs = _mm_abs_epi8 (bb_signal_sample_aux);

                    //Get very early values
                    y = _mm_lddqu_si128((__m128i*)VE_code_ptr);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, _mm_set_epi8 (14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1));
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    output = _mm_blendv_epi8 (imag_output, real_output, mult1);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_1 = _mm_cvtepi32_ps(output_i32);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_2 = _mm_cvtepi32_ps(output_i32);

                    VE_code_acc = _mm_add_ps (VE_code_acc, output_ps_1);
                    VE_code_acc = _mm_add_ps (VE_code_acc, output_ps_2);

                    //Get early values
                    y = _mm_lddqu_si128((__m128i*)E_code_ptr);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, _mm_set_epi8 (14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1));
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    output = _mm_blendv_epi8 (imag_output, real_output, mult1);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_1 = _mm_cvtepi32_ps(output_i32);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_2 = _mm_cvtepi32_ps(output_i32);

                    E_code_acc = _mm_add_ps (E_code_acc, output_ps_1);
                    E_code_acc = _mm_add_ps (E_code_acc, output_ps_2);

                    //Get prompt values
                    y = _mm_lddqu_si128((__m128i*)P_code_ptr);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, _mm_set_epi8 (14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1));
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    output = _mm_blendv_epi8 (imag_output, real_output, mult1);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_1 = _mm_cvtepi32_ps(output_i32);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_2 = _mm_cvtepi32_ps(output_i32);

                    P_code_acc = _mm_add_ps (P_code_acc, output_ps_1);
                    P_code_acc = _mm_add_ps (P_code_acc, output_ps_2);

                    //Get late values
                    y = _mm_lddqu_si128((__m128i*)L_code_ptr);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, _mm_set_epi8 (14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1));
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    output = _mm_blendv_epi8 (imag_output, real_output, mult1);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_1 = _mm_cvtepi32_ps(output_i32);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_2 = _mm_cvtepi32_ps(output_i32);

                    L_code_acc = _mm_add_ps (L_code_acc, output_ps_1);
                    L_code_acc = _mm_add_ps (L_code_acc, output_ps_2);

                    //Get very late values
                    y = _mm_lddqu_si128((__m128i*)VL_code_ptr);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, _mm_set_epi8 (14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1));
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    output = _mm_blendv_epi8 (imag_output, real_output, mult1);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_1 = _mm_cvtepi32_ps(output_i32);

                    input_i_1 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    input_i_2 = _mm_cvtepi8_epi32(output);
                    output = _mm_srli_si128 (output, 4);
                    output_i32 = _mm_add_epi32 (input_i_1, input_i_2);
                    output_ps_2 = _mm_cvtepi32_ps(output_i32);

                    VL_code_acc = _mm_add_ps (VL_code_acc, output_ps_1);
                    VL_code_acc = _mm_add_ps (VL_code_acc, output_ps_2);

                    input_ptr += 8;
                    carrier_ptr += 8;
                    VE_code_ptr += 8;
                    E_code_ptr += 8;
                    P_code_ptr += 8;
                    L_code_ptr += 8;
                    VL_code_ptr += 8;
                }

            __VOLK_ATTR_ALIGNED(16) lv_32fc_t VE_dotProductVector[2];
            __VOLK_ATTR_ALIGNED(16) lv_32fc_t E_dotProductVector[2];
            __VOLK_ATTR_ALIGNED(16) lv_32fc_t P_dotProductVector[2];
            __VOLK_ATTR_ALIGNED(16) lv_32fc_t L_dotProductVector[2];
            __VOLK_ATTR_ALIGNED(16) lv_32fc_t VL_dotProductVector[2];

            _mm_storeu_ps((float*)VE_dotProductVector,VE_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)E_dotProductVector,E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)P_dotProductVector,P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)L_dotProductVector,L_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)VL_dotProductVector,VL_code_acc); // Store the results back into the dot product vector

            for (int i = 0; i<2; ++i)
                {
                    *VE_out_ptr += VE_dotProductVector[i];
                    *E_out_ptr += E_dotProductVector[i];
                    *P_out_ptr += P_dotProductVector[i];
                    *L_out_ptr += L_dotProductVector[i];
                    *VL_out_ptr += VL_dotProductVector[i];
                }
        }

    lv_8sc_t bb_signal_sample;
    for(int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get very early, early, prompt, late and very late values for each
            *VE_out_ptr += (lv_32fc_t) (bb_signal_sample * (*VE_code_ptr++));
            *E_out_ptr += (lv_32fc_t) (bb_signal_sample * (*E_code_ptr++));
            *P_out_ptr += (lv_32fc_t) (bb_signal_sample * (*P_code_ptr++));
            *L_out_ptr += (lv_32fc_t) (bb_signal_sample * (*L_code_ptr++));
            *VL_out_ptr += (lv_32fc_t) (bb_signal_sample * (*VL_code_ptr++));
        }
}
#endif /* LV_HAVE_SSE4_1 */

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
#include "CommonMacros/CommonMacros_8ic_cw_epl_corr_32fc.h"
#include "CommonMacros/CommonMacros.h"
/*!
 \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param VE_code Very Early PRN code replica input
 \param E_code Early PRN code replica input
 \param P_code Prompt PRN code replica input
 \param L_code Late PRN code replica input
 \param VL_code Very Late PRN code replica input
 \param VE_out Very Early correlation output
 \param E_out Early correlation output
 \param P_out Prompt correlation output
 \param L_out Late correlation output
 \param VL_out Very Late correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5_u_sse4_1_third(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* VE_code, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, const lv_8sc_t* VL_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    __m128i x, x_abs, y, y_aux, bb_signal_sample_aux, bb_signal_sample_aux_abs;;
    __m128i mult1, real_output, imag_output;

    __m128 real_VE_code_acc, imag_VE_code_acc, real_E_code_acc, imag_E_code_acc, real_P_code_acc, imag_P_code_acc, real_L_code_acc, imag_L_code_acc, real_VL_code_acc, imag_VL_code_acc;
    __m128i real_output_i_1, real_output_i_2, imag_output_i_1, imag_output_i_2, real_output_i32, imag_output_i32;
    __m128 real_output_ps, imag_output_ps;

    __m128i check_sign_sequence = _mm_set_epi8 (255, 1, 255, 1, 255, 1, 255, 1, 255, 1, 255, 1, 255, 1, 255, 1);
    __m128i rearrange_sequence = _mm_set_epi8 (14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1);

    const lv_8sc_t* input_ptr = input;
    const lv_8sc_t* carrier_ptr = carrier;

    const lv_8sc_t* VE_code_ptr = VE_code;
    lv_32fc_t* VE_out_ptr = VE_out;
    const lv_8sc_t* E_code_ptr = E_code;
    lv_32fc_t* E_out_ptr = E_out;
    const lv_8sc_t* P_code_ptr = P_code;
    lv_32fc_t* P_out_ptr = P_out;
    const lv_8sc_t* L_code_ptr = L_code;
    lv_32fc_t* L_out_ptr = L_out;
    const lv_8sc_t* VL_code_ptr = VL_code;
    lv_32fc_t* VL_out_ptr = VL_out;

    float VE_out_real = 0;
    float VE_out_imag = 0;
    float E_out_real = 0;
    float E_out_imag = 0;
    float P_out_real = 0;
    float P_out_imag = 0;
    float L_out_real = 0;
    float L_out_imag = 0;
    float VL_out_real = 0;
    float VL_out_imag = 0;

    mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);

    real_VE_code_acc = _mm_setzero_ps();
    imag_VE_code_acc = _mm_setzero_ps();
    real_E_code_acc = _mm_setzero_ps();
    imag_E_code_acc = _mm_setzero_ps();
    real_P_code_acc = _mm_setzero_ps();
    imag_P_code_acc = _mm_setzero_ps();
    real_L_code_acc = _mm_setzero_ps();
    imag_L_code_acc = _mm_setzero_ps();
    real_VL_code_acc = _mm_setzero_ps();
    imag_VL_code_acc = _mm_setzero_ps();

    if (sse_iters>0)
        {
            for(int number = 0;number < sse_iters; number++)
                {
                    //Perform the carrier wipe-off
                    x = _mm_lddqu_si128((__m128i*)input_ptr);
                    y = _mm_lddqu_si128((__m128i*)carrier_ptr);

                    x_abs = _mm_abs_epi8 (x);

                    y_aux = _mm_sign_epi8 (y, x);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (x_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, rearrange_sequence);
                    y_aux = _mm_sign_epi8 (y_aux, x);
                    imag_output = _mm_maddubs_epi16 (x_abs, y_aux);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    bb_signal_sample_aux = _mm_blendv_epi8 (imag_output, real_output, mult1);
                    bb_signal_sample_aux_abs = _mm_abs_epi8 (bb_signal_sample_aux);

                    //Get very early values
                    y = _mm_lddqu_si128((__m128i*)VE_code_ptr);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, rearrange_sequence);
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_i32 = _mm_add_epi32 (real_output_i_1, real_output_i_2);
                    real_output_ps = _mm_cvtepi32_ps(real_output_i32);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_i32 = _mm_add_epi32 (imag_output_i_1, imag_output_i_2);
                    imag_output_ps = _mm_cvtepi32_ps(imag_output_i32);

                    real_VE_code_acc = _mm_add_ps (real_VE_code_acc, real_output_ps);
                    imag_VE_code_acc = _mm_add_ps (imag_VE_code_acc, imag_output_ps);

                    //Get early values
                    y = _mm_lddqu_si128((__m128i*)E_code_ptr);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, rearrange_sequence);
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_i32 = _mm_add_epi32 (real_output_i_1, real_output_i_2);
                    real_output_ps = _mm_cvtepi32_ps(real_output_i32);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_i32 = _mm_add_epi32 (imag_output_i_1, imag_output_i_2);
                    imag_output_ps = _mm_cvtepi32_ps(imag_output_i32);

                    real_E_code_acc = _mm_add_ps (real_E_code_acc, real_output_ps);
                    imag_E_code_acc = _mm_add_ps (imag_E_code_acc, imag_output_ps);

                    //Get prompt values
                    y = _mm_lddqu_si128((__m128i*)P_code_ptr);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, rearrange_sequence);
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_i32 = _mm_add_epi32 (real_output_i_1, real_output_i_2);
                    real_output_ps = _mm_cvtepi32_ps(real_output_i32);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_i32 = _mm_add_epi32 (imag_output_i_1, imag_output_i_2);
                    imag_output_ps = _mm_cvtepi32_ps(imag_output_i32);

                    real_P_code_acc = _mm_add_ps (real_P_code_acc, real_output_ps);
                    imag_P_code_acc = _mm_add_ps (imag_P_code_acc, imag_output_ps);

                    //Get late values
                    y = _mm_lddqu_si128((__m128i*)L_code_ptr);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, rearrange_sequence);
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_i32 = _mm_add_epi32 (real_output_i_1, real_output_i_2);
                    real_output_ps = _mm_cvtepi32_ps(real_output_i32);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_i32 = _mm_add_epi32 (imag_output_i_1, imag_output_i_2);
                    imag_output_ps = _mm_cvtepi32_ps(imag_output_i32);

                    real_L_code_acc = _mm_add_ps (real_L_code_acc, real_output_ps);
                    imag_L_code_acc = _mm_add_ps (imag_L_code_acc, imag_output_ps);

                    //Get very late values
                    y = _mm_lddqu_si128((__m128i*)VL_code_ptr);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, _mm_set_epi8 (14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1));
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_i32 = _mm_add_epi32 (real_output_i_1, real_output_i_2);
                    real_output_ps = _mm_cvtepi32_ps(real_output_i32);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_i32 = _mm_add_epi32 (imag_output_i_1, imag_output_i_2);
                    imag_output_ps = _mm_cvtepi32_ps(imag_output_i32);

                    real_VL_code_acc = _mm_add_ps (real_VL_code_acc, real_output_ps);
                    imag_VL_code_acc = _mm_add_ps (imag_VL_code_acc, imag_output_ps);

                    input_ptr += 8;
                    carrier_ptr += 8;
                    VE_code_ptr += 8;
                    E_code_ptr += 8;
                    P_code_ptr += 8;
                    L_code_ptr += 8;
                    VL_code_ptr += 8;
                }

            __VOLK_ATTR_ALIGNED(16) float real_VE_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_VE_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_L_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_L_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_VL_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_VL_dotProductVector[4];

            _mm_storeu_ps((float*)real_VE_dotProductVector,real_VE_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_VE_dotProductVector,imag_VE_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_E_dotProductVector,real_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_E_dotProductVector,imag_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_P_dotProductVector,real_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_P_dotProductVector,imag_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_L_dotProductVector,real_L_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_L_dotProductVector,imag_L_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_VL_dotProductVector,real_VL_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_VL_dotProductVector,imag_VL_code_acc); // Store the results back into the dot product vector

            for (int i = 0; i<4; ++i)
                {
                    VE_out_real += real_VE_dotProductVector[i];
                    VE_out_imag += imag_VE_dotProductVector[i];
                    E_out_real += real_E_dotProductVector[i];
                    E_out_imag += imag_E_dotProductVector[i];
                    P_out_real += real_P_dotProductVector[i];
                    P_out_imag += imag_P_dotProductVector[i];
                    L_out_real += real_L_dotProductVector[i];
                    L_out_imag += imag_L_dotProductVector[i];
                    VL_out_real += real_VL_dotProductVector[i];
                    VL_out_imag += imag_VL_dotProductVector[i];
                }
            *VE_out_ptr = lv_cmake(VE_out_real, VE_out_imag);
            *E_out_ptr = lv_cmake(E_out_real, E_out_imag);
            *P_out_ptr = lv_cmake(P_out_real, P_out_imag);
            *L_out_ptr = lv_cmake(L_out_real, L_out_imag);
            *VL_out_ptr = lv_cmake(VL_out_real, VL_out_imag);
        }

    lv_16sc_t bb_signal_sample;
    for(int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get very early, early, prompt, late and very late values for each
            *VE_out_ptr += (lv_32fc_t) (bb_signal_sample * ((lv_16sc_t)*VE_code_ptr++));
            *E_out_ptr += (lv_32fc_t) (bb_signal_sample * ((lv_16sc_t)*E_code_ptr++));
            *P_out_ptr += (lv_32fc_t) (bb_signal_sample * ((lv_16sc_t)*P_code_ptr++));
            *L_out_ptr += (lv_32fc_t) (bb_signal_sample * ((lv_16sc_t)*L_code_ptr++));
            *VL_out_ptr += (lv_32fc_t) (bb_signal_sample * ((lv_16sc_t)*VL_code_ptr++));
        }
}
#endif /* LV_HAVE_SSE4_1 */

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
#include "CommonMacros/CommonMacros_8ic_cw_epl_corr_32fc.h"
#include "CommonMacros/CommonMacros.h"
/*!
 \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param VE_code Very Early PRN code replica input
 \param E_code Early PRN code replica input
 \param P_code Prompt PRN code replica input
 \param L_code Late PRN code replica input
 \param VL_code Very Late PRN code replica input
 \param VE_out Very Early correlation output
 \param E_out Early correlation output
 \param P_out Prompt correlation output
 \param L_out Late correlation output
 \param VL_out Very Late correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5_u_sse4_1_fourth(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* VE_code, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, const lv_8sc_t* VL_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    __m128i x, x_abs, y, y_aux, bb_signal_sample_aux, bb_signal_sample_aux_abs;;
    __m128i real_output, imag_output;
    __m128 real_VE_code_acc, imag_VE_code_acc, real_E_code_acc, imag_E_code_acc, real_P_code_acc, imag_P_code_acc, real_L_code_acc, imag_L_code_acc, real_VL_code_acc, imag_VL_code_acc;
    __m128i real_output_i_1, real_output_i_2, imag_output_i_1, imag_output_i_2, real_output_i32, imag_output_i32;
    __m128 real_output_ps, imag_output_ps;
    __m128i minus128control;

    __m128i minus128 = _mm_set1_epi8 (-128);
    __m128i check_sign_sequence = _mm_set_epi8 (255, 1, 255, 1, 255, 1, 255, 1, 255, 1, 255, 1, 255, 1, 255, 1);
    __m128i rearrange_sequence = _mm_set_epi8(14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1);
    __m128i mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);

    const lv_8sc_t* input_ptr = input;
    const lv_8sc_t* carrier_ptr = carrier;

    const lv_8sc_t* VE_code_ptr = VE_code;
    lv_32fc_t* VE_out_ptr = VE_out;
    const lv_8sc_t* E_code_ptr = E_code;
    lv_32fc_t* E_out_ptr = E_out;
    const lv_8sc_t* P_code_ptr = P_code;
    lv_32fc_t* P_out_ptr = P_out;
    const lv_8sc_t* L_code_ptr = L_code;
    lv_32fc_t* L_out_ptr = L_out;
    const lv_8sc_t* VL_code_ptr = VL_code;
    lv_32fc_t* VL_out_ptr = VL_out;

    float VE_out_real = 0;
    float VE_out_imag = 0;
    float E_out_real = 0;
    float E_out_imag = 0;
    float P_out_real = 0;
    float P_out_imag = 0;
    float L_out_real = 0;
    float L_out_imag = 0;
    float VL_out_real = 0;
    float VL_out_imag = 0;

    real_VE_code_acc = _mm_setzero_ps();
    imag_VE_code_acc = _mm_setzero_ps();
    real_E_code_acc = _mm_setzero_ps();
    imag_E_code_acc = _mm_setzero_ps();
    real_P_code_acc = _mm_setzero_ps();
    imag_P_code_acc = _mm_setzero_ps();
    real_L_code_acc = _mm_setzero_ps();
    imag_L_code_acc = _mm_setzero_ps();
    real_VL_code_acc = _mm_setzero_ps();
    imag_VL_code_acc = _mm_setzero_ps();

    if (sse_iters>0)
        {
            for(int number = 0;number < sse_iters; number++)
                {
                    //Perform the carrier wipe-off
                    x = _mm_lddqu_si128((__m128i*)input_ptr);
                    y = _mm_lddqu_si128((__m128i*)carrier_ptr);

                    x_abs = _mm_abs_epi8 (x);

                    y_aux = _mm_sign_epi8 (y, x);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (x_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, rearrange_sequence);
                    y_aux = _mm_sign_epi8 (y_aux, x);
                    imag_output = _mm_maddubs_epi16 (x_abs, y_aux);

                    imag_output = _mm_slli_si128 (imag_output, 1);
                    bb_signal_sample_aux = _mm_blendv_epi8 (imag_output, real_output, mult1);
                    bb_signal_sample_aux_abs = _mm_abs_epi8 (bb_signal_sample_aux);

                    //Get very early values
                    y = _mm_lddqu_si128((__m128i*)VE_code_ptr);
                    minus128control = _mm_cmpeq_epi8 (y, minus128);
                    y = _mm_sub_epi8 (y, minus128control);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, rearrange_sequence);
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_i32 = _mm_add_epi32 (real_output_i_1, real_output_i_2);
                    real_output_ps = _mm_cvtepi32_ps(real_output_i32);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_i32 = _mm_add_epi32 (imag_output_i_1, imag_output_i_2);
                    imag_output_ps = _mm_cvtepi32_ps(imag_output_i32);

                    real_VE_code_acc = _mm_add_ps (real_VE_code_acc, real_output_ps);
                    imag_VE_code_acc = _mm_add_ps (imag_VE_code_acc, imag_output_ps);

                    //Get early values
                    y = _mm_lddqu_si128((__m128i*)E_code_ptr);
                    minus128control = _mm_cmpeq_epi8 (y, minus128);
                    y = _mm_sub_epi8 (y, minus128control);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, rearrange_sequence);
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_i32 = _mm_add_epi32 (real_output_i_1, real_output_i_2);
                    real_output_ps = _mm_cvtepi32_ps(real_output_i32);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_i32 = _mm_add_epi32 (imag_output_i_1, imag_output_i_2);
                    imag_output_ps = _mm_cvtepi32_ps(imag_output_i32);

                    real_E_code_acc = _mm_add_ps (real_E_code_acc, real_output_ps);
                    imag_E_code_acc = _mm_add_ps (imag_E_code_acc, imag_output_ps);

                    //Get prompt values
                    y = _mm_lddqu_si128((__m128i*)P_code_ptr);
                    minus128control = _mm_cmpeq_epi8 (y, minus128);
                    y = _mm_sub_epi8 (y, minus128control);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, rearrange_sequence);
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_i32 = _mm_add_epi32 (real_output_i_1, real_output_i_2);
                    real_output_ps = _mm_cvtepi32_ps(real_output_i32);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_i32 = _mm_add_epi32 (imag_output_i_1, imag_output_i_2);
                    imag_output_ps = _mm_cvtepi32_ps(imag_output_i32);

                    real_P_code_acc = _mm_add_ps (real_P_code_acc, real_output_ps);
                    imag_P_code_acc = _mm_add_ps (imag_P_code_acc, imag_output_ps);

                    //Get late values
                    y = _mm_lddqu_si128((__m128i*)L_code_ptr);
                    minus128control = _mm_cmpeq_epi8 (y, minus128);
                    y = _mm_sub_epi8 (y, minus128control);

                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, rearrange_sequence);
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_i32 = _mm_add_epi32 (real_output_i_1, real_output_i_2);
                    real_output_ps = _mm_cvtepi32_ps(real_output_i32);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_i32 = _mm_add_epi32 (imag_output_i_1, imag_output_i_2);
                    imag_output_ps = _mm_cvtepi32_ps(imag_output_i32);

                    real_L_code_acc = _mm_add_ps (real_L_code_acc, real_output_ps);
                    imag_L_code_acc = _mm_add_ps (imag_L_code_acc, imag_output_ps);

                    //Get very late values
                    y = _mm_lddqu_si128((__m128i*)VL_code_ptr);
                    minus128control = _mm_cmpeq_epi8 (y, minus128);
                    y = _mm_sub_epi8 (y, minus128control);


                    y_aux = _mm_sign_epi8 (y, bb_signal_sample_aux);
                    y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);
                    real_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    y_aux = _mm_shuffle_epi8 (y, _mm_set_epi8 (14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1));
                    y_aux = _mm_sign_epi8 (y_aux, bb_signal_sample_aux);
                    imag_output = _mm_maddubs_epi16 (bb_signal_sample_aux_abs, y_aux);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_i32 = _mm_add_epi32 (real_output_i_1, real_output_i_2);
                    real_output_ps = _mm_cvtepi32_ps(real_output_i32);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_i32 = _mm_add_epi32 (imag_output_i_1, imag_output_i_2);
                    imag_output_ps = _mm_cvtepi32_ps(imag_output_i32);

                    real_VL_code_acc = _mm_add_ps (real_VL_code_acc, real_output_ps);
                    imag_VL_code_acc = _mm_add_ps (imag_VL_code_acc, imag_output_ps);

                    input_ptr += 8;
                    carrier_ptr += 8;
                    VE_code_ptr += 8;
                    E_code_ptr += 8;
                    P_code_ptr += 8;
                    L_code_ptr += 8;
                    VL_code_ptr += 8;
                }

            __VOLK_ATTR_ALIGNED(16) float real_VE_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_VE_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_L_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_L_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_VL_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_VL_dotProductVector[4];

            _mm_storeu_ps((float*)real_VE_dotProductVector,real_VE_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_VE_dotProductVector,imag_VE_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_E_dotProductVector,real_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_E_dotProductVector,imag_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_P_dotProductVector,real_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_P_dotProductVector,imag_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_L_dotProductVector,real_L_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_L_dotProductVector,imag_L_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_VL_dotProductVector,real_VL_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_VL_dotProductVector,imag_VL_code_acc); // Store the results back into the dot product vector

            for (int i = 0; i<4; ++i)
                {
                    VE_out_real += real_VE_dotProductVector[i];
                    VE_out_imag += imag_VE_dotProductVector[i];
                    E_out_real += real_E_dotProductVector[i];
                    E_out_imag += imag_E_dotProductVector[i];
                    P_out_real += real_P_dotProductVector[i];
                    P_out_imag += imag_P_dotProductVector[i];
                    L_out_real += real_L_dotProductVector[i];
                    L_out_imag += imag_L_dotProductVector[i];
                    VL_out_real += real_VL_dotProductVector[i];
                    VL_out_imag += imag_VL_dotProductVector[i];
                }
            *VE_out_ptr = lv_cmake(VE_out_real, VE_out_imag);
            *E_out_ptr = lv_cmake(E_out_real, E_out_imag);
            *P_out_ptr = lv_cmake(P_out_real, P_out_imag);
            *L_out_ptr = lv_cmake(L_out_real, L_out_imag);
            *VL_out_ptr = lv_cmake(VL_out_real, VL_out_imag);
        }

    lv_16sc_t bb_signal_sample;
    for(int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get very early, early, prompt, late and very late values for each
            *VE_out_ptr += (lv_32fc_t) (bb_signal_sample * ((lv_16sc_t)*VE_code_ptr++));
            *E_out_ptr += (lv_32fc_t) (bb_signal_sample * ((lv_16sc_t)*E_code_ptr++));
            *P_out_ptr += (lv_32fc_t) (bb_signal_sample * ((lv_16sc_t)*P_code_ptr++));
            *L_out_ptr += (lv_32fc_t) (bb_signal_sample * ((lv_16sc_t)*L_code_ptr++));
            *VL_out_ptr += (lv_32fc_t) (bb_signal_sample * ((lv_16sc_t)*VL_code_ptr++));
        }
}
#endif /* LV_HAVE_SSE4_1 */


#ifdef LV_HAVE_GENERIC

/*!
 \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param VE_code Very Early PRN code replica input
 \param E_code Early PRN code replica input
 \param P_code Prompt PRN code replica input
 \param L_code Late PRN code replica input
 \param VL_code Very Late PRN code replica input
 \param VE_out Very Early correlation output
 \param E_out Early correlation output
 \param P_out Prompt correlation output
 \param L_out Late correlation output
 \param VL_out Very Late correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5_generic(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* VE_code, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, const lv_8sc_t* VL_code, unsigned int num_points)
{
    *VE_out = 0;
    *E_out = 0;
    *P_out = 0;
    *L_out = 0;
    *VL_out = 0;


    lv_16sc_t VE_code_value;
    lv_16sc_t E_code_value;
    lv_16sc_t P_code_value;
    lv_16sc_t L_code_value;
    lv_16sc_t VL_code_value;
    lv_16sc_t bb_signal_sample;

    for(unsigned int i=0; i < num_points; ++i)
        {
            VE_code_value = VE_code[i];
            E_code_value = E_code[i];
            P_code_value = P_code[i];
            L_code_value = L_code[i];
            VL_code_value = VL_code[i];

            if(lv_creal(VE_code_value) == -128)
                {
                    VE_code_value = lv_cmake(-127, lv_cimag(VE_code_value));
                }
            if(lv_cimag(VE_code_value) == -128)
                {
                    VE_code_value = lv_cmake(lv_creal(VE_code_value), -127);
                }

            if(lv_creal(E_code_value) == -128)
                {
                    E_code_value = lv_cmake(-127, lv_cimag(E_code_value));
                }
            if(lv_cimag(E_code_value) == -128)
                {
                    E_code_value = lv_cmake(lv_creal(E_code_value), -127);
                }

            if(lv_creal(P_code_value) == -128)
                {
                    P_code_value = lv_cmake(-127, lv_cimag(P_code_value));
                }
            if(lv_cimag(P_code_value) == -128)
                {
                    P_code_value = lv_cmake(lv_creal(P_code_value), -127);
                }

            if(lv_creal(L_code_value) == -128)
                {
                    L_code_value = lv_cmake(-127, lv_cimag(L_code_value));
                }
            if(lv_cimag(L_code_value) == -128)
                {
                    L_code_value = lv_cmake(lv_creal(L_code_value), -127);
                }

            if(lv_creal(VL_code_value) == -128)
                {
                    VL_code_value = lv_cmake(-127, lv_cimag(VL_code_value));
                }
            if(lv_cimag(VL_code_value) == -128)
                {
                    VL_code_value = lv_cmake(lv_creal(VL_code_value), -127);
                }

            //Perform the carrier wipe-off
            bb_signal_sample = input[i] * carrier[i];
            // Now get very early, early, prompt, late and very late values for each
            *VE_out += (lv_32fc_t) (bb_signal_sample * VE_code_value);
            *E_out += (lv_32fc_t) (bb_signal_sample * E_code_value);
            *P_out += (lv_32fc_t) (bb_signal_sample * P_code_value);
            *L_out += (lv_32fc_t) (bb_signal_sample * L_code_value);
            *VL_out += (lv_32fc_t) (bb_signal_sample * VL_code_value);
        }
}

#endif /* LV_HAVE_GENERIC */

//#ifdef LV_HAVE_GENERIC
//#include <stdio.h>
//#include <stdlib.h>
//#include <tmmintrin.h>
//
//#ifndef MAX
//#define MAX(a,b) ((a) > (b) ? a : b)
//#endif
//
//#ifndef MIN
//#define MIN(a,b) ((a) < (b) ? a : b)
//#endif
//
///*!
// \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
// \param input The input signal input
// \param carrier The carrier signal input
// \param VE_code Very Early PRN code replica input
// \param E_code Early PRN code replica input
// \param P_code Prompt PRN code replica input
// \param L_code Late PRN code replica input
// \param VL_code Very Late PRN code replica input
// \param VE_out Very Early correlation output
// \param E_out Early correlation output
// \param P_out Prompt correlation output
// \param L_out Late correlation output
// \param VL_out Very Late correlation output
// \param num_points The number of complex values in vectors
// */
//static inline void volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5_generic(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* VE_code, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, const lv_8sc_t* VL_code, unsigned int num_points)
//{
//    *VE_out = 0;
//    *E_out = 0;
//    *P_out = 0;
//    *L_out = 0;
//    *VL_out = 0;
//    
//    lv_16sc_t VE_out16;
//    lv_16sc_t E_out16;
//    lv_16sc_t P_out16;
//    lv_16sc_t L_out16;
//    lv_16sc_t VL_out16;
//    
//    int32_t max = 32767;
//    int32_t min = -32768;
//    
//    int16_t real_real;
//    int16_t imag_imag;
//    int16_t real_imag;
//    int16_t imag_real;
//    int32_t out_real_32;
//    int32_t out_imag_32;
//    int16_t out_real_16;
//    int16_t out_imag_16;
//    int16_t aux1;
//    int16_t aux2;
//    
//    
//    lv_8sc_t bb_signal_sample = lv_cmake(0, 0);
//
//    // perform very early, Early, Prompt, Late and very late correlation
//    for(int i=0; i < num_points; ++i)
//    {
//        //Perform the carrier wipe-off
//        bb_signal_sample = input[i] * carrier[i];
//        
//        aux1 = (int16_t)lv_creal(bb_signal_sample);
//        aux2 = (int16_t)lv_creal(VE_code[i]);
//        real_real = aux1*aux2;
//        aux1 = (int16_t)lv_cimag(bb_signal_sample);
//        aux2 = (int16_t)lv_cimag(VE_code[i]);
//        imag_imag = aux1*aux2;
//        aux1 = (int16_t)lv_creal(bb_signal_sample);
//        aux2 = (int16_t)lv_cimag(VE_code[i]);
//        real_imag = aux1*aux2;
//        aux1 = (int16_t)lv_cimag(bb_signal_sample);
//        aux2 = (int16_t)lv_creal(VE_code[i]);
//        imag_real = aux1*aux2;
//        out_real_32 = (int32_t)real_real - (int32_t)imag_imag;
//        out_imag_32 = (int32_t)real_imag + (int32_t)imag_real;
//        out_real_16 = MIN(MAX(out_real_32, min), max);
//        out_imag_16 = MIN(MAX(out_imag_32, min), max);
//        VE_out16 = lv_cmake(out_real_16, out_imag_16);
//        
//        
//        
//        if(lv_creal(L_code[i]) == -128)
//        {
//            int8_t* L_pointer = (int8_t*)&L_code[i];
//            *L_pointer = -127;
//        }
//        if(lv_cimag(L_code[i]) == -128)
//        {
//            int8_t* L_pointer = (int8_t*)&L_code[i];
//            L_pointer++;
//            *L_pointer = -127;
//        }
//        aux1 = (int16_t)lv_creal(bb_signal_sample);
//        aux2 = (int16_t)lv_creal(L_code[i]);
//        real_real = aux1*aux2;
//        aux1 = (int16_t)lv_cimag(bb_signal_sample);
//        aux2 = (int16_t)lv_cimag(L_code[i]);
//        imag_imag = aux1*aux2;
//        aux1 = (int16_t)lv_creal(bb_signal_sample);
//        aux2 = (int16_t)lv_cimag(L_code[i]);
//        real_imag = aux1*aux2;
//        aux1 = (int16_t)lv_cimag(bb_signal_sample);
//        aux2 = (int16_t)lv_creal(L_code[i]);
//        imag_real = aux1*aux2;
//        out_real_32 = (int32_t)real_real - (int32_t)imag_imag;
//        out_imag_32 = (int32_t)real_imag + (int32_t)imag_real;
//        out_real_16 = MIN(MAX(out_real_32, min), max);
//        out_imag_16 = MIN(MAX(out_imag_32, min), max);
//        L_out16 = lv_cmake(out_real_16, out_imag_16);
//        
//        E_out16 = (lv_16sc_t)bb_signal_sample * (lv_16sc_t)E_code[i];
//        P_out16 = (lv_16sc_t)bb_signal_sample * (lv_16sc_t)P_code[i];
//        VL_out16 = (lv_16sc_t)bb_signal_sample * (lv_16sc_t)VL_code[i];
//
//        
//        *VE_out += (lv_32fc_t) VE_out16;
//        *E_out += (lv_32fc_t) E_out16;
//        *P_out += (lv_32fc_t) P_out16;
//        *L_out += (lv_32fc_t) L_out16;
//        *VL_out += (lv_32fc_t) VL_out16;
//        
//        //error en la parte real de L con 32 muestras
//        //*L_out = lv_cmake(12, 12);
//    }
//}
//
//#endif /* LV_HAVE_GENERIC */

//#ifdef LV_HAVE_GENERIC
///*!
// \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
// \param input The input signal input
// \param carrier The carrier signal input
// \param VE_code Very Early PRN code replica input
// \param E_code Early PRN code replica input
// \param P_code Prompt PRN code replica input
// \param L_code Late PRN code replica input
// \param VL_code Very Late PRN code replica input
// \param VE_out Very Early correlation output
// \param E_out Early correlation output
// \param P_out Prompt correlation output
// \param L_out Late correlation output
// \param VL_out Very Late correlation output
// \param num_points The number of complex values in vectors
// */
//static inline void volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5_generic(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* VE_code, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, const lv_8sc_t* VL_code, unsigned int num_points)
//{
//    lv_8sc_t bb_signal_sample;
//    
//    bb_signal_sample = lv_cmake(0, 0);
//    
//    *VE_out = 0;
//    *E_out = 0;
//    *P_out = 0;
//    *L_out = 0;
//    *VL_out = 0;
//    // perform very early, Early, Prompt, Late and very late correlation
//    for(int i=0; i < num_points; ++i)
//    {
//        //Perform the carrier wipe-off
//        bb_signal_sample = input[i] * carrier[i];
//        
//        *VE_out += (lv_32fc_t) (bb_signal_sample * VE_code[i]);
//        *E_out += (lv_32fc_t) (bb_signal_sample * E_code[i]);
//        *P_out += (lv_32fc_t) (bb_signal_sample * P_code[i]);
//        *L_out += (lv_32fc_t) (bb_signal_sample * L_code[i]);
//        *VL_out += (lv_32fc_t) (bb_signal_sample * VL_code[i]);
//    }
//}
//
//#endif /* LV_HAVE_GENERIC */

#endif /* INCLUDED_gnsssdr_volk_gnsssdr_8ic_x7_cw_vepl_corr_TEST_32fc_x5_u_H */
