/*!
 * \file CommonMacros.h
 * \brief Common macros used inside the volk protokernels.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
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
#ifndef INCLUDED_gnsssdr_CommonMacros_u_H
#define INCLUDED_gnsssdr_CommonMacros_u_H

    #ifdef LV_HAVE_SSE4_1
      /*!
        \brief Macros for U_SSE4_1
      */

        #ifndef CM_16IC_X2_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE4_1
        #define CM_16IC_X2_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE4_1(input1, input2, real, imag)\
        imag = _mm_srli_si128 (input1, 2);\
        imag = _mm_blend_epi16 (input2, imag, 85);\
        real = _mm_slli_si128 (input2, 2);\
        real = _mm_blend_epi16 (real, input1, 85);
        #endif /* CM_16IC_X2_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE4_1 */

        #ifndef CM_16IC_CONVERT_AND_ACC_32FC_U_SSE4_1
        #define CM_16IC_CONVERT_AND_ACC_32FC_U_SSE4_1(input, input_i_1, input_i_2, output_i32, output_ps)\
        input_i_1 = _mm_cvtepi16_epi32(input);\
        input = _mm_srli_si128 (input, 8);\
        input_i_2 = _mm_cvtepi16_epi32(input);\
        output_i32 = _mm_add_epi32 (input_i_1, input_i_2);\
        output_ps = _mm_cvtepi32_ps(output_i32);
        #endif /* CM_16IC_CONVERT_AND_ACC_32FC_U_SSE4_1 */

        #ifndef CM_8IC_CONVERT_AND_ACC_32FC_U_SSE4_1
        #define CM_8IC_CONVERT_AND_ACC_32FC_U_SSE4_1(input, input_i_1, input_i_2, output_i32, output_i32_1, output_i32_2, output_ps)\
        input_i_1 = _mm_cvtepi8_epi32(input);\
        input = _mm_srli_si128 (input, 4);\
        input_i_2 = _mm_cvtepi8_epi32(input);\
        input = _mm_srli_si128 (input, 4);\
        output_i32_1 = _mm_add_epi32 (input_i_1, input_i_2);\
        input_i_1 = _mm_cvtepi8_epi32(input);\
        input = _mm_srli_si128 (input, 4);\
        input_i_2 = _mm_cvtepi8_epi32(input);\
        input = _mm_srli_si128 (input, 4);\
        output_i32_2 = _mm_add_epi32 (input_i_1, input_i_2);\
        output_i32 = _mm_add_epi32 (output_i32_1, output_i32_2);\
        output_ps = _mm_cvtepi32_ps(output_i32);
        #endif /* CM_8IC_CONVERT_AND_ACC_32FC_U_SSE4_1 */

    #endif /* LV_HAVE_SSE4_1 */

    #ifdef LV_HAVE_SSE2
    /*!
     \brief Macros for U_SSE2
     */

    #ifdef LV_HAVE_SSSE3
    /*!
     \brief Macros for U_SSSE3
     */

        #ifndef CM_8IC_X2_SCALAR_PRODUCT_16IC_X2_U_SSSE3
        #define CM_8IC_X2_SCALAR_PRODUCT_16IC_X2_U_SSSE3(y, x, check_sign_sequence, rearrange_sequence, y_aux, x_abs, real_output, imag_output)\
        y_aux = _mm_sign_epi8 (y, x);\
        y_aux = _mm_sign_epi8 (y_aux, check_sign_sequence);\
        real_output = _mm_maddubs_epi16 (x_abs, y_aux);\
        \
        y_aux = _mm_shuffle_epi8 (y, rearrange_sequence);\
        y_aux = _mm_sign_epi8 (y_aux, x);\
        imag_output = _mm_maddubs_epi16 (x_abs, y_aux);
        #endif /* CM_8IC_X2_SCALAR_PRODUCT_16IC_X2_U_SSSE3 */

    #endif /* LV_HAVE_SSSE3 */

        #ifndef CM_16IC_X4_SCALAR_PRODUCT_16IC_X2_U_SSE2
        #define CM_16IC_X4_SCALAR_PRODUCT_16IC_X2_U_SSE2(realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output)\
        realx_mult_realy = _mm_mullo_epi16 (realx, realy);\
        imagx_mult_imagy = _mm_mullo_epi16 (imagx, imagy);\
        realx_mult_imagy = _mm_mullo_epi16 (realx, imagy);\
        imagx_mult_realy = _mm_mullo_epi16 (imagx, realy);\
        real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);\
        imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);
        #endif /* CM_16IC_X4_SCALAR_PRODUCT_16IC_X2_U_SSE2 */

        #ifndef CM_8IC_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE2
        #define CM_8IC_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE2(input, mult1, real, imag)\
        imag = _mm_srli_si128 (input, 1);\
        imag = _mm_and_si128 (imag, mult1);\
        real = _mm_and_si128 (input, mult1);
        #endif /* CM_8IC_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE2 */

        #ifndef CM_8IC_CONVERT_AND_ACC_32FC_U_SSE2
        #define CM_8IC_CONVERT_AND_ACC_32FC_U_SSE2(input, input_i_1, input_i_2, output_i32, output_ps_1, output_ps_2)\
        input_i_1 = _mm_unpacklo_epi8(_mm_setzero_si128(), input);\
        input_i_2 = _mm_unpacklo_epi16(_mm_setzero_si128(), input_i_1);\
        input_i_1 = _mm_unpackhi_epi16(_mm_setzero_si128(), input_i_1);\
        input_i_1 = _mm_srai_epi32(input_i_1, 24);\
        input_i_2 = _mm_srai_epi32(input_i_2, 24);\
        output_i32 = _mm_add_epi32(input_i_1, input_i_2);\
        output_ps_1 = _mm_cvtepi32_ps(output_i32);\
        \
        input_i_1 = _mm_unpackhi_epi8(_mm_setzero_si128(), input);\
        input_i_2 = _mm_unpacklo_epi16(_mm_setzero_si128(), input_i_1);\
        input_i_1 = _mm_unpackhi_epi16(_mm_setzero_si128(), input_i_1);\
        input_i_1 = _mm_srai_epi32(input_i_1, 24);\
        input_i_2 = _mm_srai_epi32(input_i_2, 24);\
        output_i32 = _mm_add_epi32(input_i_1, input_i_2);\
        output_ps_2 = _mm_cvtepi32_ps(output_i32);
        #endif /* CM_8IC_CONVERT_AND_ACC_32FC_U_SSE2 */

        #ifndef CM_8IC_CONTROLMINUS128_8IC_U_SSE2
        #define CM_8IC_CONTROLMINUS128_8IC_U_SSE2(y, minus128, minus128control)\
        minus128control = _mm_cmpeq_epi8 (y, minus128);\
        y = _mm_sub_epi8 (y, minus128control);
        #endif /* CM_8IC_CONTROLMINUS128_8IC_U_SSE2 */

    #endif /* LV_HAVE_SSE2 */

    #ifdef LV_HAVE_GENERIC
    /*!
     \brief Macros for U_GENERIC
     */

    #endif /* LV_HAVE_GENERIC */
#endif /* INCLUDED_gnsssdr_CommonMacros_u_H */


#ifndef INCLUDED_gnsssdr_CommonMacros_a_H
#define INCLUDED_gnsssdr_CommonMacros_a_H

    #ifdef LV_HAVE_SSE4_1
    /*!
     \brief Macros for A_SSE4_1
     */

    #endif /* LV_HAVE_SSE4_1 */

    #ifdef LV_HAVE_SSE2
    /*!
     \brief Macros for U_SSE2
     */

    #endif /* LV_HAVE_SSE2 */

    #ifdef LV_HAVE_GENERIC
    /*!
     \brief Macros for A_GENERIC
     */

    #endif /* LV_HAVE_GENERIC */
#endif /* INCLUDED_gnsssdr_CommonMacros_a_H */
