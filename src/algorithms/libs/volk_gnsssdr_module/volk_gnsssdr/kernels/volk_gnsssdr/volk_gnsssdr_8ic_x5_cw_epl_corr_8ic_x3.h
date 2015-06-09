/*!
 * \file volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3.h
 * \brief Volk protokernel: performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation with 16 bits vectors
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that performs the carrier wipe-off mixing and the 
 * Early, Prompt, and Late correlation with 16 bits vectors (8 bits the 
 * real part and 8 bits the imaginary part):
 * - The carrier wipe-off is done by multiplying the input signal by the 
 * carrier (multiplication of 16 bits vectors) It returns the input 
 * signal in base band (BB)
 * - Early values are calculated by multiplying the input signal in BB by the
 * early code (multiplication of 16 bits vectors), accumulating the results
 * - Prompt values are calculated by multiplying the input signal in BB by the
 * prompt code (multiplication of 16 bits vectors), accumulating the results
 * - Late values are calculated by multiplying the input signal in BB by the
 * late code (multiplication of 16 bits vectors), accumulating the results
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

#ifndef INCLUDED_gnsssdr_volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_u_H
#define INCLUDED_gnsssdr_volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_u_H

#include <inttypes.h>
#include <stdio.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <float.h>
#include <string.h>

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>

/*! \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
    \param input The input signal input
    \param carrier The carrier signal input
    \param E_code Early PRN code replica input
    \param P_code Early PRN code replica input
    \param L_code Early PRN code replica input
    \param E_out Early correlation output
    \param P_out Early correlation output
    \param L_out Early correlation output
    \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_u_sse4_1(lv_8sc_t* E_out, lv_8sc_t* P_out, lv_8sc_t* L_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    __m128i x, y, real_bb_signal_sample, imag_bb_signal_sample, real_E_code_acc, imag_E_code_acc, real_L_code_acc, imag_L_code_acc, real_P_code_acc, imag_P_code_acc;
    __m128i mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, output, real_output, imag_output;

    const lv_8sc_t* input_ptr = input;
    const lv_8sc_t* carrier_ptr = carrier;

    const lv_8sc_t* E_code_ptr = E_code;
    lv_8sc_t* E_out_ptr = E_out;
    const lv_8sc_t* L_code_ptr = L_code;
    lv_8sc_t* L_out_ptr = L_out;
    const lv_8sc_t* P_code_ptr = P_code;
    lv_8sc_t* P_out_ptr = P_out;

    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;

    mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);

    real_E_code_acc = _mm_setzero_si128();
    imag_E_code_acc = _mm_setzero_si128();
    real_L_code_acc = _mm_setzero_si128();
    imag_L_code_acc = _mm_setzero_si128();
    real_P_code_acc = _mm_setzero_si128();
    imag_P_code_acc = _mm_setzero_si128();

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

                    real_E_code_acc = _mm_add_epi16 (real_E_code_acc, real_output);
                    imag_E_code_acc = _mm_add_epi16 (imag_E_code_acc, imag_output);

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

                    real_L_code_acc = _mm_add_epi16 (real_L_code_acc, real_output);
                    imag_L_code_acc = _mm_add_epi16 (imag_L_code_acc, imag_output);

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

                    real_P_code_acc = _mm_add_epi16 (real_P_code_acc, real_output);
                    imag_P_code_acc = _mm_add_epi16 (imag_P_code_acc, imag_output);

                    input_ptr += 8;
                    carrier_ptr += 8;
                    E_code_ptr += 8;
                    L_code_ptr += 8;
                    P_code_ptr += 8;
                }

            __VOLK_ATTR_ALIGNED(16) lv_8sc_t E_dotProductVector[8];
            __VOLK_ATTR_ALIGNED(16) lv_8sc_t L_dotProductVector[8];
            __VOLK_ATTR_ALIGNED(16) lv_8sc_t P_dotProductVector[8];

            imag_E_code_acc = _mm_slli_si128 (imag_E_code_acc, 1);
            output = _mm_blendv_epi8 (imag_E_code_acc, real_E_code_acc, mult1);
            _mm_storeu_si128((__m128i*)E_dotProductVector, output);

            imag_L_code_acc = _mm_slli_si128 (imag_L_code_acc, 1);
            output = _mm_blendv_epi8 (imag_L_code_acc, real_L_code_acc, mult1);
            _mm_storeu_si128((__m128i*)L_dotProductVector, output);

            imag_P_code_acc = _mm_slli_si128 (imag_P_code_acc, 1);
            output = _mm_blendv_epi8 (imag_P_code_acc, real_P_code_acc, mult1);
            _mm_storeu_si128((__m128i*)P_dotProductVector, output);

            for (int i = 0; i<8; ++i)
                {
                    *E_out_ptr += E_dotProductVector[i];
                    *L_out_ptr += L_dotProductVector[i];
                    *P_out_ptr += P_dotProductVector[i];
                }
        }

    lv_8sc_t bb_signal_sample;
    for(int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get early, late, and prompt values for each
            *E_out_ptr += bb_signal_sample * (*E_code_ptr++);
            *P_out_ptr += bb_signal_sample * (*P_code_ptr++);
            *L_out_ptr += bb_signal_sample * (*L_code_ptr++);
        }
}

#endif /* LV_HAVE_SSE4_1 */

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/*!
 \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param E_code Early PRN code replica input
 \param P_code Early PRN code replica input
 \param L_code Early PRN code replica input
 \param E_out Early correlation output
 \param P_out Early correlation output
 \param L_out Early correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_u_sse2(lv_8sc_t* E_out, lv_8sc_t* P_out, lv_8sc_t* L_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    __m128i x, y, real_bb_signal_sample, imag_bb_signal_sample, real_E_code_acc, imag_E_code_acc, real_L_code_acc, imag_L_code_acc, real_P_code_acc, imag_P_code_acc;
    __m128i mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, output, real_output, imag_output;

    const lv_8sc_t* input_ptr = input;
    const lv_8sc_t* carrier_ptr = carrier;

    const lv_8sc_t* E_code_ptr = E_code;
    lv_8sc_t* E_out_ptr = E_out;
    const lv_8sc_t* L_code_ptr = L_code;
    lv_8sc_t* L_out_ptr = L_out;
    const lv_8sc_t* P_code_ptr = P_code;
    lv_8sc_t* P_out_ptr = P_out;

    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;

    mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);

    real_E_code_acc = _mm_setzero_si128();
    imag_E_code_acc = _mm_setzero_si128();
    real_L_code_acc = _mm_setzero_si128();
    imag_L_code_acc = _mm_setzero_si128();
    real_P_code_acc = _mm_setzero_si128();
    imag_P_code_acc = _mm_setzero_si128();

    if (sse_iters>0)
        {
            for(unsigned int number = 0;number < sse_iters; number++)
                {
                    //Perform the carrier wipe-off
                    x = _mm_loadu_si128((__m128i*)input_ptr);
                    y = _mm_loadu_si128((__m128i*)carrier_ptr);

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

                    //Get early values
                    y = _mm_loadu_si128((__m128i*)E_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_E_code_acc = _mm_add_epi16 (real_E_code_acc, real_output);
                    imag_E_code_acc = _mm_add_epi16 (imag_E_code_acc, imag_output);

                    //Get late values
                    y = _mm_loadu_si128((__m128i*)L_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_L_code_acc = _mm_add_epi16 (real_L_code_acc, real_output);
                    imag_L_code_acc = _mm_add_epi16 (imag_L_code_acc, imag_output);

                    //Get prompt values
                    y = _mm_loadu_si128((__m128i*)P_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_P_code_acc = _mm_add_epi16 (real_P_code_acc, real_output);
                    imag_P_code_acc = _mm_add_epi16 (imag_P_code_acc, imag_output);

                    input_ptr += 8;
                    carrier_ptr += 8;
                    E_code_ptr += 8;
                    L_code_ptr += 8;
                    P_code_ptr += 8;
                }

            __VOLK_ATTR_ALIGNED(16) lv_8sc_t E_dotProductVector[8];
            __VOLK_ATTR_ALIGNED(16) lv_8sc_t L_dotProductVector[8];
            __VOLK_ATTR_ALIGNED(16) lv_8sc_t P_dotProductVector[8];

            real_E_code_acc = _mm_and_si128 (real_E_code_acc, mult1);
            imag_E_code_acc = _mm_and_si128 (imag_E_code_acc, mult1);
            imag_E_code_acc = _mm_slli_si128 (imag_E_code_acc, 1);
            output = _mm_or_si128 (real_E_code_acc, imag_E_code_acc);
            _mm_storeu_si128((__m128i*)E_dotProductVector, output);

            real_L_code_acc = _mm_and_si128 (real_L_code_acc, mult1);
            imag_L_code_acc = _mm_and_si128 (imag_L_code_acc, mult1);
            imag_L_code_acc = _mm_slli_si128 (imag_L_code_acc, 1);
            output = _mm_or_si128 (real_L_code_acc, imag_L_code_acc);
            _mm_storeu_si128((__m128i*)L_dotProductVector, output);

            real_P_code_acc = _mm_and_si128 (real_P_code_acc, mult1);
            imag_P_code_acc = _mm_and_si128 (imag_P_code_acc, mult1);
            imag_P_code_acc = _mm_slli_si128 (imag_P_code_acc, 1);
            output = _mm_or_si128 (real_P_code_acc, imag_P_code_acc);
            _mm_storeu_si128((__m128i*)P_dotProductVector, output);

            for (int i = 0; i<8; ++i)
                {
                    *E_out_ptr += E_dotProductVector[i];
                    *L_out_ptr += L_dotProductVector[i];
                    *P_out_ptr += P_dotProductVector[i];
                }
        }

    lv_8sc_t bb_signal_sample;
    for(unsigned int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get early, late, and prompt values for each
            *E_out_ptr += bb_signal_sample * (*E_code_ptr++);
            *P_out_ptr += bb_signal_sample * (*P_code_ptr++);
            *L_out_ptr += bb_signal_sample * (*L_code_ptr++);
        }
}

#endif /* LV_HAVE_SSE2 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param E_code Early PRN code replica input
 \param P_code Early PRN code replica input
 \param L_code Early PRN code replica input
 \param E_out Early correlation output
 \param P_out Early correlation output
 \param L_out Early correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_generic(lv_8sc_t* E_out, lv_8sc_t* P_out, lv_8sc_t* L_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, unsigned int num_points)
{
    lv_8sc_t bb_signal_sample;

    bb_signal_sample = lv_cmake(0, 0);

    *E_out = 0;
    *P_out = 0;
    *L_out = 0;
    // perform Early, Prompt and Late correlation
    for(unsigned int i=0; i < num_points; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = input[i] * carrier[i];
            // Now get early, late, and prompt values for each
            *E_out += bb_signal_sample * E_code[i];
            *P_out += bb_signal_sample * P_code[i];
            *L_out += bb_signal_sample * L_code[i];
        }
}

#endif /* LV_HAVE_GENERIC */

#endif /* INCLUDED_gnsssdr_volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_u_H */


#ifndef INCLUDED_gnsssdr_volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_a_H
#define INCLUDED_gnsssdr_volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_a_H

#include <inttypes.h>
#include <stdio.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <float.h>
#include <string.h>

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
/*!
 \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param E_code Early PRN code replica input
 \param P_code Early PRN code replica input
 \param L_code Early PRN code replica input
 \param E_out Early correlation output
 \param P_out Early correlation output
 \param L_out Early correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_a_sse4_1(lv_8sc_t* E_out, lv_8sc_t* P_out, lv_8sc_t* L_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    __m128i x, y, real_bb_signal_sample, imag_bb_signal_sample, real_E_code_acc, imag_E_code_acc, real_L_code_acc, imag_L_code_acc, real_P_code_acc, imag_P_code_acc;
    __m128i mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, output, real_output, imag_output;

    const lv_8sc_t* input_ptr = input;
    const lv_8sc_t* carrier_ptr = carrier;

    const lv_8sc_t* E_code_ptr = E_code;
    lv_8sc_t* E_out_ptr = E_out;
    const lv_8sc_t* L_code_ptr = L_code;
    lv_8sc_t* L_out_ptr = L_out;
    const lv_8sc_t* P_code_ptr = P_code;
    lv_8sc_t* P_out_ptr = P_out;

    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;

    mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);

    real_E_code_acc = _mm_setzero_si128();
    imag_E_code_acc = _mm_setzero_si128();
    real_L_code_acc = _mm_setzero_si128();
    imag_L_code_acc = _mm_setzero_si128();
    real_P_code_acc = _mm_setzero_si128();
    imag_P_code_acc = _mm_setzero_si128();

    if (sse_iters>0)
        {
            for(int number = 0;number < sse_iters; number++)
                {
                    //Perform the carrier wipe-off
                    x = _mm_load_si128((__m128i*)input_ptr);
                    y = _mm_load_si128((__m128i*)carrier_ptr);

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

                    //Get early values
                    y = _mm_load_si128((__m128i*)E_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_E_code_acc = _mm_add_epi16 (real_E_code_acc, real_output);
                    imag_E_code_acc = _mm_add_epi16 (imag_E_code_acc, imag_output);

                    //Get late values
                    y = _mm_load_si128((__m128i*)L_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_L_code_acc = _mm_add_epi16 (real_L_code_acc, real_output);
                    imag_L_code_acc = _mm_add_epi16 (imag_L_code_acc, imag_output);

                    //Get prompt values
                    y = _mm_load_si128((__m128i*)P_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_P_code_acc = _mm_add_epi16 (real_P_code_acc, real_output);
                    imag_P_code_acc = _mm_add_epi16 (imag_P_code_acc, imag_output);

                    input_ptr += 8;
                    carrier_ptr += 8;
                    E_code_ptr += 8;
                    L_code_ptr += 8;
                    P_code_ptr += 8;
                }

            __VOLK_ATTR_ALIGNED(16) lv_8sc_t E_dotProductVector[8];
            __VOLK_ATTR_ALIGNED(16) lv_8sc_t L_dotProductVector[8];
            __VOLK_ATTR_ALIGNED(16) lv_8sc_t P_dotProductVector[8];

            imag_E_code_acc = _mm_slli_si128 (imag_E_code_acc, 1);
            output = _mm_blendv_epi8 (imag_E_code_acc, real_E_code_acc, mult1);
            _mm_store_si128((__m128i*)E_dotProductVector, output);

            imag_L_code_acc = _mm_slli_si128 (imag_L_code_acc, 1);
            output = _mm_blendv_epi8 (imag_L_code_acc, real_L_code_acc, mult1);
            _mm_store_si128((__m128i*)L_dotProductVector, output);

            imag_P_code_acc = _mm_slli_si128 (imag_P_code_acc, 1);
            output = _mm_blendv_epi8 (imag_P_code_acc, real_P_code_acc, mult1);
            _mm_store_si128((__m128i*)P_dotProductVector, output);

            for (int i = 0; i<8; ++i)
                {
                    *E_out_ptr += E_dotProductVector[i];
                    *L_out_ptr += L_dotProductVector[i];
                    *P_out_ptr += P_dotProductVector[i];
                }
        }

    lv_8sc_t bb_signal_sample;
    for(int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get early, late, and prompt values for each
            *E_out_ptr += bb_signal_sample * (*E_code_ptr++);
            *P_out_ptr += bb_signal_sample * (*P_code_ptr++);
            *L_out_ptr += bb_signal_sample * (*L_code_ptr++);
        }
}

#endif /* LV_HAVE_SSE4_1 */

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/*!
 \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param E_code Early PRN code replica input
 \param P_code Early PRN code replica input
 \param L_code Early PRN code replica input
 \param E_out Early correlation output
 \param P_out Early correlation output
 \param L_out Early correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_a_sse2(lv_8sc_t* E_out, lv_8sc_t* P_out, lv_8sc_t* L_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    __m128i x, y, real_bb_signal_sample, imag_bb_signal_sample, real_E_code_acc, imag_E_code_acc, real_L_code_acc, imag_L_code_acc, real_P_code_acc, imag_P_code_acc;
    __m128i mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, output, real_output, imag_output;

    const lv_8sc_t* input_ptr = input;
    const lv_8sc_t* carrier_ptr = carrier;

    const lv_8sc_t* E_code_ptr = E_code;
    lv_8sc_t* E_out_ptr = E_out;
    const lv_8sc_t* L_code_ptr = L_code;
    lv_8sc_t* L_out_ptr = L_out;
    const lv_8sc_t* P_code_ptr = P_code;
    lv_8sc_t* P_out_ptr = P_out;

    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;

    mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);

    real_E_code_acc = _mm_setzero_si128();
    imag_E_code_acc = _mm_setzero_si128();
    real_L_code_acc = _mm_setzero_si128();
    imag_L_code_acc = _mm_setzero_si128();
    real_P_code_acc = _mm_setzero_si128();
    imag_P_code_acc = _mm_setzero_si128();

    if (sse_iters>0)
        {
            for(unsigned int number = 0;number < sse_iters; number++)
                {
                    //Perform the carrier wipe-off
                    x = _mm_load_si128((__m128i*)input_ptr);
                    y = _mm_load_si128((__m128i*)carrier_ptr);

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

                    //Get early values
                    y = _mm_load_si128((__m128i*)E_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_E_code_acc = _mm_add_epi16 (real_E_code_acc, real_output);
                    imag_E_code_acc = _mm_add_epi16 (imag_E_code_acc, imag_output);

                    //Get late values
                    y = _mm_load_si128((__m128i*)L_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_L_code_acc = _mm_add_epi16 (real_L_code_acc, real_output);
                    imag_L_code_acc = _mm_add_epi16 (imag_L_code_acc, imag_output);

                    //Get prompt values
                    y = _mm_load_si128((__m128i*)P_code_ptr);

                    imagy = _mm_srli_si128 (y, 1);
                    imagy = _mm_and_si128 (imagy, mult1);
                    realy = _mm_and_si128 (y, mult1);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_P_code_acc = _mm_add_epi16 (real_P_code_acc, real_output);
                    imag_P_code_acc = _mm_add_epi16 (imag_P_code_acc, imag_output);

                    input_ptr += 8;
                    carrier_ptr += 8;
                    E_code_ptr += 8;
                    L_code_ptr += 8;
                    P_code_ptr += 8;
                }

            __VOLK_ATTR_ALIGNED(16) lv_8sc_t E_dotProductVector[8];
            __VOLK_ATTR_ALIGNED(16) lv_8sc_t L_dotProductVector[8];
            __VOLK_ATTR_ALIGNED(16) lv_8sc_t P_dotProductVector[8];

            real_E_code_acc = _mm_and_si128 (real_E_code_acc, mult1);
            imag_E_code_acc = _mm_and_si128 (imag_E_code_acc, mult1);
            imag_E_code_acc = _mm_slli_si128 (imag_E_code_acc, 1);
            output = _mm_or_si128 (real_E_code_acc, imag_E_code_acc);
            _mm_store_si128((__m128i*)E_dotProductVector, output);

            real_L_code_acc = _mm_and_si128 (real_L_code_acc, mult1);
            imag_L_code_acc = _mm_and_si128 (imag_L_code_acc, mult1);
            imag_L_code_acc = _mm_slli_si128 (imag_L_code_acc, 1);
            output = _mm_or_si128 (real_L_code_acc, imag_L_code_acc);
            _mm_store_si128((__m128i*)L_dotProductVector, output);

            real_P_code_acc = _mm_and_si128 (real_P_code_acc, mult1);
            imag_P_code_acc = _mm_and_si128 (imag_P_code_acc, mult1);
            imag_P_code_acc = _mm_slli_si128 (imag_P_code_acc, 1);
            output = _mm_or_si128 (real_P_code_acc, imag_P_code_acc);
            _mm_store_si128((__m128i*)P_dotProductVector, output);

            for (unsigned int i = 0; i<8; ++i)
                {
                    *E_out_ptr += E_dotProductVector[i];
                    *L_out_ptr += L_dotProductVector[i];
                    *P_out_ptr += P_dotProductVector[i];
                }
        }

    lv_8sc_t bb_signal_sample;
    for(unsigned int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get early, late, and prompt values for each
            *E_out_ptr += bb_signal_sample * (*E_code_ptr++);
            *P_out_ptr += bb_signal_sample * (*P_code_ptr++);
            *L_out_ptr += bb_signal_sample * (*L_code_ptr++);
        }
}

#endif /* LV_HAVE_SSE2 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param E_code Early PRN code replica input
 \param P_code Early PRN code replica input
 \param L_code Early PRN code replica input
 \param E_out Early correlation output
 \param P_out Early correlation output
 \param L_out Early correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_a_generic(lv_8sc_t* E_out, lv_8sc_t* P_out, lv_8sc_t* L_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, unsigned int num_points)
{
    lv_8sc_t bb_signal_sample;

    bb_signal_sample = lv_cmake(0, 0);

    *E_out = 0;
    *P_out = 0;
    *L_out = 0;
    // perform Early, Prompt and Late correlation
    for(unsigned int i=0; i < num_points; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = input[i] * carrier[i];
            // Now get early, late, and prompt values for each
            *E_out += bb_signal_sample * E_code[i];
            *P_out += bb_signal_sample * P_code[i];
            *L_out += bb_signal_sample * L_code[i];
        }
}

#endif /* LV_HAVE_GENERIC */

#ifdef LV_HAVE_ORC
/*!
 \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param E_code Early PRN code replica input
 \param P_code Early PRN code replica input
 \param L_code Early PRN code replica input
 \param E_out Early correlation output
 \param P_out Early correlation output
 \param L_out Early correlation output
 \param num_points The number of complex values in vectors
 */

extern void volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_first_a_orc_impl(short* E_out_real, short* E_out_imag, short* P_out_real, short* P_out_imag, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* E_code, const lv_8sc_t* P_code, unsigned int num_points);
extern void volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_second_a_orc_impl(short* L_out_real, short* L_out_imag, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* L_code, unsigned int num_points);
static inline void volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_u_orc(lv_8sc_t* E_out, lv_8sc_t* P_out, lv_8sc_t* L_out, const lv_8sc_t* input, const lv_8sc_t* carrier, const lv_8sc_t* E_code, const lv_8sc_t* P_code, const lv_8sc_t* L_code, unsigned int num_points){

    short E_out_real = 0;
    short E_out_imag = 0;
    char* E_out_real_c = (char*)&E_out_real;
    E_out_real_c++;
    char* E_out_imag_c = (char*)&E_out_imag;
    E_out_imag_c++;

    short P_out_real = 0;
    short P_out_imag = 0;
    char* P_out_real_c = (char*)&P_out_real;
    P_out_real_c++;
    char* P_out_imag_c = (char*)&P_out_imag;
    P_out_imag_c++;

    short L_out_real = 0;
    short L_out_imag = 0;
    char* L_out_real_c = (char*)&L_out_real;
    L_out_real_c++;
    char* L_out_imag_c = (char*)&L_out_imag;
    L_out_imag_c++;

    volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_first_a_orc_impl( &E_out_real, &E_out_imag, &P_out_real, &P_out_imag, input, carrier, E_code, P_code, num_points);
    volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_second_a_orc_impl( &L_out_real, &L_out_imag, input, carrier, L_code, num_points);

    //ORC implementation of 8ic_x5_cw_epl_corr_8ic_x3 is done in two different functions because it seems that
    //in one function the length of the code gives memory problems (bad access, segmentation fault).
    //Also, the maximum number of accumulators that can be used is 4 (and we need 6).
    //The "carrier wipe-off" step is done two times: one in the first function and another one in the second.
    //Joining all the ORC code in one function would be quicker because the "carrier wipe-off" step would be done just
    //one time.

    *E_out = lv_cmake(*E_out_real_c, *E_out_imag_c);
    *P_out = lv_cmake(*P_out_real_c, *P_out_imag_c);
    *L_out = lv_cmake(*L_out_real_c, *L_out_imag_c);
}
#endif /* LV_HAVE_ORC */

#endif /* INCLUDED_gnsssdr_volk_gnsssdr_8ic_x5_cw_epl_corr_8ic_x3_a_H */
