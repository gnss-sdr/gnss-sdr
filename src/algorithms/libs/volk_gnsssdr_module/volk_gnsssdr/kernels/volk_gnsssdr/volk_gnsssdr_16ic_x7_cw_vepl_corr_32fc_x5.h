/*!
 * \file volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5.h
 * \brief Volk protokernel: performs the carrier wipe-off mixing and the Very early, Early, Prompt, Late and very late correlation with 32 bits vectors and returns float32 values.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that performs the carrier wipe-off mixing and the 
 * Very Early, Early, Prompt, Late and Very Late correlation with 32 bits vectors (16 bits the
 * real part and 16 bits the imaginary part) and accumulates into float32 values, returning them:
 * - The carrier wipe-off is done by multiplying the input signal by the 
 * carrier (multiplication of 32 bits vectors) It returns the input
 * signal in base band (BB)
 * - Very Early values are calculated by multiplying the input signal in BB by the
 * very early code (multiplication of 32 bits vectors), converting that to float32 and accumulating the results
 * - Early values are calculated by multiplying the input signal in BB by the
 * early code (multiplication of 32 bits vectors), converting that to float32 and accumulating the results
 * - Prompt values are calculated by multiplying the input signal in BB by the
 * prompt code (multiplication of 32 bits vectors), converting that to float32 and accumulating the results
 * - Late values are calculated by multiplying the input signal in BB by the
 * late code (multiplication of 32 bits vectors), converting that to float32 and accumulating the results
 * - Very Late values are calculated by multiplying the input signal in BB by the
 * very late code (multiplication of 32 bits vectors), converting that to float32 and accumulating the results
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

#ifndef INCLUDED_gnsssdr_volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5_u_H
#define INCLUDED_gnsssdr_volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5_u_H

#include <inttypes.h>
#include <stdio.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <float.h>
#include <string.h>

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
#include "CommonMacros/CommonMacros_16ic_cw_epl_corr_32fc.h"
#include "CommonMacros/CommonMacros.h"
  /*!
    \brief Performs the carrier wipe-off mixing and the Very Early, Early, Prompt, Late and Very Vate correlation
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
static inline void volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5_u_sse4_1(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* VE_code, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, const lv_16sc_t* VL_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    
    __m128i x1, x2, y1, y2, real_bb_signal_sample, imag_bb_signal_sample;
    __m128i realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output;
    
    __m128 real_VE_code_acc, imag_VE_code_acc, real_E_code_acc, imag_E_code_acc, real_P_code_acc, imag_P_code_acc, real_L_code_acc, imag_L_code_acc, real_VL_code_acc, imag_VL_code_acc;
    __m128i input_i_1, input_i_2, output_i32;
    __m128 real_output_ps, imag_output_ps;
    
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
    
    const lv_16sc_t* input_ptr = input;
    const lv_16sc_t* carrier_ptr = carrier;
    
    const lv_16sc_t* VE_code_ptr = VE_code;
    lv_32fc_t* VE_out_ptr = VE_out;
    const lv_16sc_t* E_code_ptr = E_code;
    lv_32fc_t* E_out_ptr = E_out;
    const lv_16sc_t* L_code_ptr = L_code;
    lv_32fc_t* L_out_ptr = L_out;
    const lv_16sc_t* P_code_ptr = P_code;
    lv_32fc_t* P_out_ptr = P_out;
    const lv_16sc_t* VL_code_ptr = VL_code;
    lv_32fc_t* VL_out_ptr = VL_out;
    
    *VE_out_ptr = 0;
    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;
    *VL_out_ptr = 0;
    
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
        for(unsigned int number = 0;number < sse_iters; number++){
            
            //Perform the carrier wipe-off
            x1 = _mm_lddqu_si128((__m128i*)input_ptr);
            input_ptr += 4;
            x2 = _mm_lddqu_si128((__m128i*)input_ptr);
            
            y1 = _mm_lddqu_si128((__m128i*)carrier_ptr);
            carrier_ptr += 4;
            y2 = _mm_lddqu_si128((__m128i*)carrier_ptr);
            
            CM_16IC_X2_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE4_1(x1, x2, realx, imagx)
            CM_16IC_X2_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE4_1(y1, y2, realy, imagy)
            CM_16IC_X4_SCALAR_PRODUCT_16IC_X2_U_SSE2(realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_bb_signal_sample, imag_bb_signal_sample)
            
            //Get very early values
            y1 = _mm_lddqu_si128((__m128i*)VE_code_ptr);
            VE_code_ptr += 4;
            y2 = _mm_lddqu_si128((__m128i*)VE_code_ptr);
            
            CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1(y1, y2, realy, imagy, real_bb_signal_sample, imag_bb_signal_sample,realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, input_i_1, input_i_2, output_i32, real_output_ps, imag_output_ps)
            
            real_VE_code_acc = _mm_add_ps (real_VE_code_acc, real_output_ps);
            imag_VE_code_acc = _mm_add_ps (imag_VE_code_acc, imag_output_ps);
            
            //Get early values
            y1 = _mm_lddqu_si128((__m128i*)E_code_ptr);
            E_code_ptr += 4;
            y2 = _mm_lddqu_si128((__m128i*)E_code_ptr);
            
            CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1(y1, y2, realy, imagy, real_bb_signal_sample, imag_bb_signal_sample,realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, input_i_1, input_i_2, output_i32, real_output_ps, imag_output_ps)
            
            real_E_code_acc = _mm_add_ps (real_E_code_acc, real_output_ps);
            imag_E_code_acc = _mm_add_ps (imag_E_code_acc, imag_output_ps);
            
            //Get prompt values
            y1 = _mm_lddqu_si128((__m128i*)P_code_ptr);
            P_code_ptr += 4;
            y2 = _mm_lddqu_si128((__m128i*)P_code_ptr);
            
            CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1(y1, y2, realy, imagy, real_bb_signal_sample, imag_bb_signal_sample,realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, input_i_1, input_i_2, output_i32, real_output_ps, imag_output_ps)
            
            real_P_code_acc = _mm_add_ps (real_P_code_acc, real_output_ps);
            imag_P_code_acc = _mm_add_ps (imag_P_code_acc, imag_output_ps);
            
            //Get late values
            y1 = _mm_lddqu_si128((__m128i*)L_code_ptr);
            L_code_ptr += 4;
            y2 = _mm_lddqu_si128((__m128i*)L_code_ptr);
            
            CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1(y1, y2, realy, imagy, real_bb_signal_sample, imag_bb_signal_sample,realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, input_i_1, input_i_2, output_i32, real_output_ps, imag_output_ps)
            
            real_L_code_acc = _mm_add_ps (real_L_code_acc, real_output_ps);
            imag_L_code_acc = _mm_add_ps (imag_L_code_acc, imag_output_ps);
            
            //Get very late values
            y1 = _mm_lddqu_si128((__m128i*)VL_code_ptr);
            VL_code_ptr += 4;
            y2 = _mm_lddqu_si128((__m128i*)VL_code_ptr);
            
            CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1(y1, y2, realy, imagy, real_bb_signal_sample, imag_bb_signal_sample,realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, input_i_1, input_i_2, output_i32, real_output_ps, imag_output_ps)
            
            real_VL_code_acc = _mm_add_ps (real_VL_code_acc, real_output_ps);
            imag_VL_code_acc = _mm_add_ps (imag_VL_code_acc, imag_output_ps);
            
            input_ptr += 4;
            carrier_ptr += 4;
            VE_code_ptr += 4;
            E_code_ptr += 4;
            P_code_ptr += 4;
            L_code_ptr += 4;
            VL_code_ptr += 4;
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
    for(unsigned int i=0; i < num_points%8; ++i)
    {
        //Perform the carrier wipe-off
        bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
        // Now get early, late, and prompt values for each
        *VE_out_ptr += (lv_32fc_t) (bb_signal_sample * (*VE_code_ptr++));
        *E_out_ptr += (lv_32fc_t) (bb_signal_sample * (*E_code_ptr++));
        *P_out_ptr += (lv_32fc_t) (bb_signal_sample * (*P_code_ptr++));
        *L_out_ptr += (lv_32fc_t) (bb_signal_sample * (*L_code_ptr++));
        *VL_out_ptr += (lv_32fc_t) (bb_signal_sample * (*VL_code_ptr++));
    }
    
}
#endif /* LV_HAVE_SSE4_1 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Performs the carrier wipe-off mixing and the Very Early, Early, Prompt, Late and Very Vate correlation
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
static inline void volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5_generic(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* VE_code, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, const lv_16sc_t* VL_code, unsigned int num_points)
{
    lv_16sc_t bb_signal_sample;
    lv_16sc_t tmp1;
    lv_16sc_t tmp2;
    lv_16sc_t tmp3;
    lv_16sc_t tmp4;
    lv_16sc_t tmp5;
    
    bb_signal_sample = lv_cmake(0, 0);
    
    *VE_out = 0;
    *E_out = 0;
    *P_out = 0;
    *L_out = 0;
    *VL_out = 0;
    // perform Early, Prompt and Late correlation
    
    for(unsigned int i=0; i < num_points; ++i)
    {
        //Perform the carrier wipe-off
        bb_signal_sample = input[i] * carrier[i];
        
        tmp1 = bb_signal_sample * VE_code[i];
        tmp2 = bb_signal_sample * E_code[i];
        tmp3 = bb_signal_sample * P_code[i];
        tmp4 = bb_signal_sample * L_code[i];
        tmp5 = bb_signal_sample * VL_code[i];
        
        // Now get early, late, and prompt values for each
        *VE_out += (lv_32fc_t)tmp1;
        *E_out += (lv_32fc_t)tmp2;
        *P_out += (lv_32fc_t)tmp3;
        *L_out += (lv_32fc_t)tmp4;
        *VL_out += (lv_32fc_t)tmp5;
    }
}
#endif /* LV_HAVE_GENERIC */
#endif /* INCLUDED_gnsssdr_volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5_u_H */


#ifndef INCLUDED_gnsssdr_volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5_a_H
#define INCLUDED_gnsssdr_volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5_a_H

#include <inttypes.h>
#include <stdio.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <float.h>
#include <string.h>

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
#include "CommonMacros/CommonMacros_16ic_cw_epl_corr_32fc.h"
#include "CommonMacros/CommonMacros.h"
/*!
 \brief Performs the carrier wipe-off mixing and the Very Early, Early, Prompt, Late and Very Vate correlation
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
static inline void volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5_a_sse4_1(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* VE_code, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, const lv_16sc_t* VL_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    
    __m128i x1, x2, y1, y2, real_bb_signal_sample, imag_bb_signal_sample;
    __m128i realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output;
    
    __m128 real_VE_code_acc, imag_VE_code_acc, real_E_code_acc, imag_E_code_acc, real_P_code_acc, imag_P_code_acc, real_L_code_acc, imag_L_code_acc, real_VL_code_acc, imag_VL_code_acc;
    __m128i input_i_1, input_i_2, output_i32;
    __m128 real_output_ps, imag_output_ps;
    
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
    
    const lv_16sc_t* input_ptr = input;
    const lv_16sc_t* carrier_ptr = carrier;
    
    const lv_16sc_t* VE_code_ptr = VE_code;
    lv_32fc_t* VE_out_ptr = VE_out;
    const lv_16sc_t* E_code_ptr = E_code;
    lv_32fc_t* E_out_ptr = E_out;
    const lv_16sc_t* L_code_ptr = L_code;
    lv_32fc_t* L_out_ptr = L_out;
    const lv_16sc_t* P_code_ptr = P_code;
    lv_32fc_t* P_out_ptr = P_out;
    const lv_16sc_t* VL_code_ptr = VL_code;
    lv_32fc_t* VL_out_ptr = VL_out;
    
    *VE_out_ptr = 0;
    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;
    *VL_out_ptr = 0;
    
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
        for(unsigned int number = 0;number < sse_iters; number++){
            
            //Perform the carrier wipe-off
            x1 = _mm_load_si128((__m128i*)input_ptr);
            input_ptr += 4;
            x2 = _mm_load_si128((__m128i*)input_ptr);
            
            y1 = _mm_load_si128((__m128i*)carrier_ptr);
            carrier_ptr += 4;
            y2 = _mm_load_si128((__m128i*)carrier_ptr);
            
            CM_16IC_X2_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE4_1(x1, x2, realx, imagx)
            CM_16IC_X2_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE4_1(y1, y2, realy, imagy)
            CM_16IC_X4_SCALAR_PRODUCT_16IC_X2_U_SSE2(realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_bb_signal_sample, imag_bb_signal_sample)
            
            //Get very early values
            y1 = _mm_load_si128((__m128i*)VE_code_ptr);
            VE_code_ptr += 4;
            y2 = _mm_load_si128((__m128i*)VE_code_ptr);
            
            CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1(y1, y2, realy, imagy, real_bb_signal_sample, imag_bb_signal_sample,realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, input_i_1, input_i_2, output_i32, real_output_ps, imag_output_ps)
            
            real_VE_code_acc = _mm_add_ps (real_VE_code_acc, real_output_ps);
            imag_VE_code_acc = _mm_add_ps (imag_VE_code_acc, imag_output_ps);
            
            //Get early values
            y1 = _mm_load_si128((__m128i*)E_code_ptr);
            E_code_ptr += 4;
            y2 = _mm_load_si128((__m128i*)E_code_ptr);
            
            CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1(y1, y2, realy, imagy, real_bb_signal_sample, imag_bb_signal_sample,realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, input_i_1, input_i_2, output_i32, real_output_ps, imag_output_ps)
            
            real_E_code_acc = _mm_add_ps (real_E_code_acc, real_output_ps);
            imag_E_code_acc = _mm_add_ps (imag_E_code_acc, imag_output_ps);
            
            //Get prompt values
            y1 = _mm_load_si128((__m128i*)P_code_ptr);
            P_code_ptr += 4;
            y2 = _mm_load_si128((__m128i*)P_code_ptr);
            
            CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1(y1, y2, realy, imagy, real_bb_signal_sample, imag_bb_signal_sample,realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, input_i_1, input_i_2, output_i32, real_output_ps, imag_output_ps)
            
            real_P_code_acc = _mm_add_ps (real_P_code_acc, real_output_ps);
            imag_P_code_acc = _mm_add_ps (imag_P_code_acc, imag_output_ps);
            
            //Get late values
            y1 = _mm_load_si128((__m128i*)L_code_ptr);
            L_code_ptr += 4;
            y2 = _mm_load_si128((__m128i*)L_code_ptr);
            
            CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1(y1, y2, realy, imagy, real_bb_signal_sample, imag_bb_signal_sample,realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, input_i_1, input_i_2, output_i32, real_output_ps, imag_output_ps)
            
            real_L_code_acc = _mm_add_ps (real_L_code_acc, real_output_ps);
            imag_L_code_acc = _mm_add_ps (imag_L_code_acc, imag_output_ps);
            
            //Get very late values
            y1 = _mm_load_si128((__m128i*)VL_code_ptr);
            VL_code_ptr += 4;
            y2 = _mm_load_si128((__m128i*)VL_code_ptr);
            
            CM_16IC_X2_CW_CORR_32FC_X2_U_SSE4_1(y1, y2, realy, imagy, real_bb_signal_sample, imag_bb_signal_sample,realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, input_i_1, input_i_2, output_i32, real_output_ps, imag_output_ps)
            
            real_VL_code_acc = _mm_add_ps (real_VL_code_acc, real_output_ps);
            imag_VL_code_acc = _mm_add_ps (imag_VL_code_acc, imag_output_ps);
            
            input_ptr += 4;
            carrier_ptr += 4;
            VE_code_ptr += 4;
            E_code_ptr += 4;
            P_code_ptr += 4;
            L_code_ptr += 4;
            VL_code_ptr += 4;
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
        
        _mm_store_ps((float*)real_VE_dotProductVector,real_VE_code_acc); // Store the results back into the dot product vector
        _mm_store_ps((float*)imag_VE_dotProductVector,imag_VE_code_acc); // Store the results back into the dot product vector
        _mm_store_ps((float*)real_E_dotProductVector,real_E_code_acc); // Store the results back into the dot product vector
        _mm_store_ps((float*)imag_E_dotProductVector,imag_E_code_acc); // Store the results back into the dot product vector
        _mm_store_ps((float*)real_P_dotProductVector,real_P_code_acc); // Store the results back into the dot product vector
        _mm_store_ps((float*)imag_P_dotProductVector,imag_P_code_acc); // Store the results back into the dot product vector
        _mm_store_ps((float*)real_L_dotProductVector,real_L_code_acc); // Store the results back into the dot product vector
        _mm_store_ps((float*)imag_L_dotProductVector,imag_L_code_acc); // Store the results back into the dot product vector
        _mm_store_ps((float*)real_VL_dotProductVector,real_VL_code_acc); // Store the results back into the dot product vector
        _mm_store_ps((float*)imag_VL_dotProductVector,imag_VL_code_acc); // Store the results back into the dot product vector
        
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
    for(unsigned int i=0; i < num_points%8; ++i)
    {
        //Perform the carrier wipe-off
        bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
        // Now get early, late, and prompt values for each
        *VE_out_ptr += (lv_32fc_t) (bb_signal_sample * (*VE_code_ptr++));
        *E_out_ptr += (lv_32fc_t) (bb_signal_sample * (*E_code_ptr++));
        *P_out_ptr += (lv_32fc_t) (bb_signal_sample * (*P_code_ptr++));
        *L_out_ptr += (lv_32fc_t) (bb_signal_sample * (*L_code_ptr++));
        *VL_out_ptr += (lv_32fc_t) (bb_signal_sample * (*VL_code_ptr++));
    }
    
}
#endif /* LV_HAVE_SSE4_1 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Performs the carrier wipe-off mixing and the Very Early, Early, Prompt, Late and Very Vate correlation
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
static inline void volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5_a_generic(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* VE_code, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, const lv_16sc_t* VL_code, unsigned int num_points)
{
    lv_16sc_t bb_signal_sample;
    lv_16sc_t tmp1;
    lv_16sc_t tmp2;
    lv_16sc_t tmp3;
    lv_16sc_t tmp4;
    lv_16sc_t tmp5;
    
    bb_signal_sample = lv_cmake(0, 0);
    
    *VE_out = 0;
    *E_out = 0;
    *P_out = 0;
    *L_out = 0;
    *VL_out = 0;
    // perform Early, Prompt and Late correlation
    
    for(unsigned int i=0; i < num_points; ++i)
    {
        //Perform the carrier wipe-off
        bb_signal_sample = input[i] * carrier[i];
        
        tmp1 = bb_signal_sample * VE_code[i];
        tmp2 = bb_signal_sample * E_code[i];
        tmp3 = bb_signal_sample * P_code[i];
        tmp4 = bb_signal_sample * L_code[i];
        tmp5 = bb_signal_sample * VL_code[i];
        
        // Now get early, late, and prompt values for each
        *VE_out += (lv_32fc_t)tmp1;
        *E_out += (lv_32fc_t)tmp2;
        *P_out += (lv_32fc_t)tmp3;
        *L_out += (lv_32fc_t)tmp4;
        *VL_out += (lv_32fc_t)tmp5;
    }
}
#endif /* LV_HAVE_GENERIC */
#endif /* INCLUDED_gnsssdr_volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5_a_H */
