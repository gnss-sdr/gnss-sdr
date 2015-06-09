/*!
 * \file volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3.h
 * \brief Volk protokernel: performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation with 32 bits vectors using different methods: inside u_sse4_1_first there is one method, inside u_sse4_1_second there is another... This protokernel has been created to test the performance of different methods.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that performs the carrier wipe-off mixing and the 
 * Early, Prompt, and Late correlation with 32 bits vectors (16 bits the
 * real part and 16 bits the imaginary part):
 * - The carrier wipe-off is done by multiplying the input signal by the 
 * carrier (multiplication of 32 bits vectors) It returns the input
 * signal in base band (BB)
 * - Early values are calculated by multiplying the input signal in BB by the
 * early code (multiplication of 32 bits vectors), accumulating the results
 * - Prompt values are calculated by multiplying the input signal in BB by the
 * prompt code (multiplication of 32 bits vectors), accumulating the results
 * - Late values are calculated by multiplying the input signal in BB by the
 * late code (multiplication of 32 bits vectors), accumulating the results
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

#ifndef INCLUDED_gnsssdr_volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_u_H
#define INCLUDED_gnsssdr_volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_u_H

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
static inline void volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_u_sse4_1_first(lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    
    __m128i x, y, yaux, yl, yh, tmp1, tmp2, z, bb_signal_sample, bb_signal_sample_suffled;
    
    __m128 z_ps_1, z_ps_2, z_E, z_P, z_L;
    __m128i z_i_1, z_i_2;
    
    lv_32fc_t dotProduct_E;
    lv_32fc_t dotProduct_P;
    lv_32fc_t dotProduct_L;
    
    z_E = _mm_setzero_ps();
    z_P = _mm_setzero_ps();
    z_L = _mm_setzero_ps();
    
    const lv_16sc_t* _input = input;
    const lv_16sc_t* _carrier = carrier;
    const lv_16sc_t* _E_code = E_code;
    const lv_16sc_t* _P_code = P_code;
    const lv_16sc_t* _L_code = L_code;
    
    dotProduct_E = 0;
    dotProduct_P = 0;
    dotProduct_L = 0;
    
    if (sse_iters>0)
    {
        for(unsigned int number = 0;number < sse_iters; number++)
        {
            //Perform the carrier wipe-off
            x = _mm_lddqu_si128((__m128i*)_input); // Load the ar + ai, br + bi as ar,ai,br,bi
            y = _mm_lddqu_si128((__m128i*)_carrier); // Load the cr + ci, dr + di as cr,ci,dr,di
            
            // Load yl with cr,cr,dr,dr
            // Load yh with ci,ci,di,di
            yaux = _mm_shuffle_epi8 (y, _mm_set_epi8 (15, 14, 11, 10, 7, 6, 3, 2, 13, 12, 9, 8, 5, 4, 1, 0));
            yl = _mm_unpacklo_epi16(yaux, yaux);
            yh = _mm_unpackhi_epi16(yaux, yaux);
            
            tmp1 = _mm_mullo_epi16(x,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            
            x = _mm_shuffle_epi8 (x, _mm_set_epi8 (13, 12, 15, 14, 9, 8, 11, 10, 5, 4, 7, 6, 1, 0, 3, 2)); // Re-arrange x to be ai,ar,bi,br
            
            tmp2 = _mm_mullo_epi16(x,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            
            tmp2 = _mm_mullo_epi16(tmp2,_mm_set_epi16 (1, -1, 1, -1, 1, -1, 1, -1));
            bb_signal_sample = _mm_add_epi16(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            bb_signal_sample_suffled = _mm_shuffle_epi8 (bb_signal_sample, _mm_set_epi8 (13, 12, 15, 14, 9, 8, 11, 10, 5, 4, 7, 6, 1, 0, 3, 2)); // Re-arrange bb_signal_sample to be ai,ar,bi,br
            
            // correlation E,P,L (3x vector scalar product)
            // Early
            y = _mm_lddqu_si128((__m128i*)_E_code); // Load the cr + ci, dr + di as cr,ci,dr,di
            
            yaux = _mm_shuffle_epi8 (y, _mm_set_epi8 (15, 14, 11, 10, 7, 6, 3, 2, 13, 12, 9, 8, 5, 4, 1, 0));
            yl = _mm_unpacklo_epi16(yaux, yaux);
            yh = _mm_unpackhi_epi16(yaux, yaux);
            
            tmp1 = _mm_mullo_epi16(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            
            tmp2 = _mm_mullo_epi16(bb_signal_sample_suffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            
            tmp2 = _mm_mullo_epi16(tmp2,_mm_set_epi16 (1, -1, 1, -1, 1, -1, 1, -1));
            z = _mm_add_epi16(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            
            z_i_1 = _mm_cvtepi16_epi32(z);
            z_ps_1 = _mm_cvtepi32_ps(z_i_1);
            z = _mm_srli_si128 (z, 8);
            z_i_2 = _mm_cvtepi16_epi32(z);
            z_ps_2 = _mm_cvtepi32_ps(z_i_2);
            
            z_E = _mm_add_ps(z_E, z_ps_1); // Add the complex multiplication results together
            z_E = _mm_add_ps(z_E, z_ps_2); // Add the complex multiplication results together
            
            // Prompt
            y = _mm_lddqu_si128((__m128i*)_P_code); // Load the cr + ci, dr + di as cr,ci,dr,di
            
            yaux = _mm_shuffle_epi8 (y, _mm_set_epi8 (15, 14, 11, 10, 7, 6, 3, 2, 13, 12, 9, 8, 5, 4, 1, 0));
            yl = _mm_unpacklo_epi16(yaux, yaux);
            yh = _mm_unpackhi_epi16(yaux, yaux);
            
            tmp1 = _mm_mullo_epi16(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            
            tmp2 = _mm_mullo_epi16(bb_signal_sample_suffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            
            tmp2 = _mm_mullo_epi16(tmp2,_mm_set_epi16 (1, -1, 1, -1, 1, -1, 1, -1));
            z = _mm_add_epi16(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            
            z_i_1 = _mm_cvtepi16_epi32(z);
            z_ps_1 = _mm_cvtepi32_ps(z_i_1);
            z = _mm_srli_si128 (z, 8);
            z_i_2 = _mm_cvtepi16_epi32(z);
            z_ps_2 = _mm_cvtepi32_ps(z_i_2);
            
            z_P = _mm_add_ps(z_P, z_ps_1); // Add the complex multiplication results together
            z_P = _mm_add_ps(z_P, z_ps_2); // Add the complex multiplication results together
            
            // Late
            y = _mm_lddqu_si128((__m128i*)_L_code); // Load the cr + ci, dr + di as cr,ci,dr,di
            
            yaux = _mm_shuffle_epi8 (y, _mm_set_epi8 (15, 14, 11, 10, 7, 6, 3, 2, 13, 12, 9, 8, 5, 4, 1, 0));
            yl = _mm_unpacklo_epi16(yaux, yaux);
            yh = _mm_unpackhi_epi16(yaux, yaux);
            
            tmp1 = _mm_mullo_epi16(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
            
            tmp2 = _mm_mullo_epi16(bb_signal_sample_suffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
            
            tmp2 = _mm_mullo_epi16(tmp2,_mm_set_epi16 (1, -1, 1, -1, 1, -1, 1, -1));
            z = _mm_add_epi16(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
            
            z_i_1 = _mm_cvtepi16_epi32(z);
            z_ps_1 = _mm_cvtepi32_ps(z_i_1);
            z = _mm_srli_si128 (z, 8);
            z_i_2 = _mm_cvtepi16_epi32(z);
            z_ps_2 = _mm_cvtepi32_ps(z_i_2);
            
            z_L = _mm_add_ps(z_L, z_ps_1); // Add the complex multiplication results together
            z_L = _mm_add_ps(z_L, z_ps_2); // Add the complex multiplication results together
            
            _input += 4;
            _carrier += 4;
            _E_code += 4;
            _L_code += 4;
            _P_code += 4;
        }
        
        __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_E[2];
        __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_P[2];
        __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_L[2];
        
        _mm_storeu_ps((float*)dotProductVector_E,z_E); // Store the results back into the dot product vector
        _mm_storeu_ps((float*)dotProductVector_P,z_P); // Store the results back into the dot product vector
        _mm_storeu_ps((float*)dotProductVector_L,z_L); // Store the results back into the dot product vector
        
        dotProduct_E = ( dotProductVector_E[0] + dotProductVector_E[1] );
        dotProduct_P = ( dotProductVector_P[0] + dotProductVector_P[1] );
        dotProduct_L = ( dotProductVector_L[0] + dotProductVector_L[1] );
        }
    
    for(unsigned int i=0; i < num_points%4; ++i)
    {
        dotProduct_E += (lv_32fc_t)((*_input) * (*_E_code++)*(*_carrier));
        dotProduct_P += (lv_32fc_t)((*_input) * (*_P_code++)*(*_carrier));
        dotProduct_L += (lv_32fc_t)((*_input++) * (*_L_code++)*(*_carrier++));
    }
    
    *E_out = dotProduct_E;
    *P_out = dotProduct_P;
    *L_out = dotProduct_L;
    
    

}
#endif /* LV_HAVE_SSE4_1 */

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
static inline void volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_u_sse4_1_second(lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    
    __m128i x1, x2, y1, y2, real_bb_signal_sample, imag_bb_signal_sample;
    __m128i mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output;
    
    __m128 real_E_code_acc, imag_E_code_acc, real_P_code_acc, imag_P_code_acc, real_L_code_acc, imag_L_code_acc;
    __m128i real_output_i_1, real_output_i_2, imag_output_i_1, imag_output_i_2;
    __m128 real_output_ps_1, real_output_ps_2, imag_output_ps_1, imag_output_ps_2;
    
    float E_out_real = 0;
    float E_out_imag = 0;
    float P_out_real = 0;
    float P_out_imag = 0;
    float L_out_real = 0;
    float L_out_imag = 0;
    
    const lv_16sc_t* input_ptr = input;
    const lv_16sc_t* carrier_ptr = carrier;
    
    const lv_16sc_t* E_code_ptr = E_code;
    lv_32fc_t* E_out_ptr = E_out;
    const lv_16sc_t* L_code_ptr = L_code;
    lv_32fc_t* L_out_ptr = L_out;
    const lv_16sc_t* P_code_ptr = P_code;
    lv_32fc_t* P_out_ptr = P_out;
    
    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;
    
    mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);
    
    real_E_code_acc = _mm_setzero_ps();
    imag_E_code_acc = _mm_setzero_ps();
    real_P_code_acc = _mm_setzero_ps();
    imag_P_code_acc = _mm_setzero_ps();
    real_L_code_acc = _mm_setzero_ps();
    imag_L_code_acc = _mm_setzero_ps();
    
    if (sse_iters>0)
    {
            for(unsigned int number = 0;number < sse_iters; number++)
                {
                    //Perform the carrier wipe-off
                    x1 = _mm_lddqu_si128((__m128i*)input_ptr);
                    input_ptr += 4;
                    x2 = _mm_lddqu_si128((__m128i*)input_ptr);

                    y1 = _mm_lddqu_si128((__m128i*)carrier_ptr);
                    carrier_ptr += 4;
                    y2 = _mm_lddqu_si128((__m128i*)carrier_ptr);

                    imagx = _mm_srli_si128 (x1, 2);
                    imagx = _mm_blend_epi16 (x2, imagx, 85);
                    realx = _mm_slli_si128 (x2, 2);
                    realx = _mm_blend_epi16 (realx, x1, 85);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (realx, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imagx, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (realx, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imagx, realy);

                    real_bb_signal_sample = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_bb_signal_sample = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    //Get early values
                    y1 = _mm_lddqu_si128((__m128i*)E_code_ptr);
                    E_code_ptr += 4;
                    y2 = _mm_lddqu_si128((__m128i*)E_code_ptr);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output_ps_1 = _mm_cvtepi32_ps(real_output_i_1);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_ps_2 = _mm_cvtepi32_ps(real_output_i_2);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_ps_1 = _mm_cvtepi32_ps(imag_output_i_1);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_ps_2 = _mm_cvtepi32_ps(imag_output_i_2);

                    real_E_code_acc = _mm_add_ps (real_E_code_acc, real_output_ps_1);
                    real_E_code_acc = _mm_add_ps (real_E_code_acc, real_output_ps_2);
                    imag_E_code_acc = _mm_add_ps (imag_E_code_acc, imag_output_ps_1);
                    imag_E_code_acc = _mm_add_ps (imag_E_code_acc, imag_output_ps_2);

                    //Get prompt values
                    y1 = _mm_lddqu_si128((__m128i*)P_code_ptr);
                    P_code_ptr += 4;
                    y2 = _mm_lddqu_si128((__m128i*)P_code_ptr);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output_ps_1 = _mm_cvtepi32_ps(real_output_i_1);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_ps_2 = _mm_cvtepi32_ps(real_output_i_2);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_ps_1 = _mm_cvtepi32_ps(imag_output_i_1);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_ps_2 = _mm_cvtepi32_ps(imag_output_i_2);

                    real_P_code_acc = _mm_add_ps (real_P_code_acc, real_output_ps_1);
                    real_P_code_acc = _mm_add_ps (real_P_code_acc, real_output_ps_2);
                    imag_P_code_acc = _mm_add_ps (imag_P_code_acc, imag_output_ps_1);
                    imag_P_code_acc = _mm_add_ps (imag_P_code_acc, imag_output_ps_2);

                    //Get late values
                    y1 = _mm_lddqu_si128((__m128i*)L_code_ptr);
                    L_code_ptr += 4;
                    y2 = _mm_lddqu_si128((__m128i*)L_code_ptr);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    real_output_i_1 = _mm_cvtepi16_epi32(real_output);
                    real_output_ps_1 = _mm_cvtepi32_ps(real_output_i_1);
                    real_output = _mm_srli_si128 (real_output, 8);
                    real_output_i_2 = _mm_cvtepi16_epi32(real_output);
                    real_output_ps_2 = _mm_cvtepi32_ps(real_output_i_2);

                    imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_ps_1 = _mm_cvtepi32_ps(imag_output_i_1);
                    imag_output = _mm_srli_si128 (imag_output, 8);
                    imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
                    imag_output_ps_2 = _mm_cvtepi32_ps(imag_output_i_2);

                    real_L_code_acc = _mm_add_ps (real_L_code_acc, real_output_ps_1);
                    real_L_code_acc = _mm_add_ps (real_L_code_acc, real_output_ps_2);
                    imag_L_code_acc = _mm_add_ps (imag_L_code_acc, imag_output_ps_1);
                    imag_L_code_acc = _mm_add_ps (imag_L_code_acc, imag_output_ps_2);

                    input_ptr += 4;
                    carrier_ptr += 4;
                    E_code_ptr += 4;
                    L_code_ptr += 4;
                    P_code_ptr += 4;
                }

            __VOLK_ATTR_ALIGNED(16) float real_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_L_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_L_dotProductVector[4];

            _mm_storeu_ps((float*)real_E_dotProductVector,real_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_E_dotProductVector,imag_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_P_dotProductVector,real_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_P_dotProductVector,imag_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_L_dotProductVector,real_L_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_L_dotProductVector,imag_L_code_acc); // Store the results back into the dot product vector

            for (int i = 0; i<4; ++i)
                {
                    E_out_real += real_E_dotProductVector[i];
                    E_out_imag += imag_E_dotProductVector[i];
                    P_out_real += real_P_dotProductVector[i];
                    P_out_imag += imag_P_dotProductVector[i];
                    L_out_real += real_L_dotProductVector[i];
                    L_out_imag += imag_L_dotProductVector[i];
                }
            *E_out_ptr = lv_cmake(E_out_real, E_out_imag);
            *P_out_ptr = lv_cmake(P_out_real, P_out_imag);
            *L_out_ptr = lv_cmake(L_out_real, L_out_imag);
    }

    lv_16sc_t bb_signal_sample;
    for(unsigned int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get early, late, and prompt values for each
            *E_out_ptr += (lv_32fc_t) (bb_signal_sample * (*E_code_ptr++));
            *P_out_ptr += (lv_32fc_t) (bb_signal_sample * (*P_code_ptr++));
            *L_out_ptr += (lv_32fc_t) (bb_signal_sample * (*L_code_ptr++));
        }
}
#endif /* LV_HAVE_SSE4_1 */

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
static inline void volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_u_sse4_1_third(lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    unsigned int index = 0;
    unsigned int indexPlus4 = 0;

    __m128i x1, x2, y1, y2, real_bb_signal_sample, imag_bb_signal_sample;
    __m128i realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, real_output_i32, imag_output_i32;

    __m128 real_E_code_acc, imag_E_code_acc, real_P_code_acc, imag_P_code_acc, real_L_code_acc, imag_L_code_acc;
    __m128i real_output_i_1, real_output_i_2, imag_output_i_1, imag_output_i_2;
    __m128 real_output_ps, imag_output_ps;

    float E_out_real = 0;
    float E_out_imag = 0;
    float P_out_real = 0;
    float P_out_imag = 0;
    float L_out_real = 0;
    float L_out_imag = 0;

    const lv_16sc_t* input_ptr = input;
    const lv_16sc_t* carrier_ptr = carrier;

    const lv_16sc_t* E_code_ptr = E_code;
    lv_32fc_t* E_out_ptr = E_out;
    const lv_16sc_t* L_code_ptr = L_code;
    lv_32fc_t* L_out_ptr = L_out;
    const lv_16sc_t* P_code_ptr = P_code;
    lv_32fc_t* P_out_ptr = P_out;

    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;

    real_E_code_acc = _mm_setzero_ps();
    imag_E_code_acc = _mm_setzero_ps();
    real_P_code_acc = _mm_setzero_ps();
    imag_P_code_acc = _mm_setzero_ps();
    real_L_code_acc = _mm_setzero_ps();
    imag_L_code_acc = _mm_setzero_ps();

    if (sse_iters>0)
        {
            for(index = 0;index < 8*sse_iters; index+=8)
                {
                    indexPlus4 = index + 4;
                    //Perform the carrier wipe-off
                    x1 = _mm_lddqu_si128((__m128i*)&input_ptr[index]);
                    x2 = _mm_lddqu_si128((__m128i*)&input_ptr[indexPlus4]);

                    y1 = _mm_lddqu_si128((__m128i*)&carrier_ptr[index]);
                    y2 = _mm_lddqu_si128((__m128i*)&carrier_ptr[indexPlus4]);

                    imagx = _mm_srli_si128 (x1, 2);
                    imagx = _mm_blend_epi16 (x2, imagx, 85);
                    realx = _mm_slli_si128 (x2, 2);
                    realx = _mm_blend_epi16 (realx, x1, 85);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (realx, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imagx, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (realx, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imagx, realy);

                    real_bb_signal_sample = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_bb_signal_sample = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    //Get early values
                    y1 = _mm_lddqu_si128((__m128i*)&E_code_ptr[index]);
                    y2 = _mm_lddqu_si128((__m128i*)&E_code_ptr[indexPlus4]);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

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
                    y1 = _mm_lddqu_si128((__m128i*)&P_code_ptr[index]);
                    y2 = _mm_lddqu_si128((__m128i*)&P_code_ptr[indexPlus4]);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

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
                    y1 = _mm_lddqu_si128((__m128i*)&L_code_ptr[index]);
                    y2 = _mm_lddqu_si128((__m128i*)&L_code_ptr[indexPlus4]);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

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
                }

            __VOLK_ATTR_ALIGNED(16) float real_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_L_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_L_dotProductVector[4];

            _mm_storeu_ps((float*)real_E_dotProductVector,real_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_E_dotProductVector,imag_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_P_dotProductVector,real_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_P_dotProductVector,imag_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_L_dotProductVector,real_L_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_L_dotProductVector,imag_L_code_acc); // Store the results back into the dot product vector

            for (int i = 0; i<4; ++i)
                {
                    E_out_real += real_E_dotProductVector[i];
                    E_out_imag += imag_E_dotProductVector[i];
                    P_out_real += real_P_dotProductVector[i];
                    P_out_imag += imag_P_dotProductVector[i];
                    L_out_real += real_L_dotProductVector[i];
                    L_out_imag += imag_L_dotProductVector[i];
                }
            *E_out_ptr = lv_cmake(E_out_real, E_out_imag);
            *P_out_ptr = lv_cmake(P_out_real, P_out_imag);
            *L_out_ptr = lv_cmake(L_out_real, L_out_imag);
        }

    lv_16sc_t bb_signal_sample;
    for(; index < num_points; index++)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = input_ptr[index] * carrier_ptr[index];
            // Now get early, late, and prompt values for each
            *E_out_ptr += (lv_32fc_t) (bb_signal_sample * E_code_ptr[index]);
            *P_out_ptr += (lv_32fc_t) (bb_signal_sample * P_code_ptr[index]);
            *L_out_ptr += (lv_32fc_t) (bb_signal_sample * L_code_ptr[index]);
        }
}
#endif /* LV_HAVE_SSE4_1 */

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
static inline void volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_u_sse4_1_fourth(lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    __m128i x1, x2, y1, y2, real_bb_signal_sample, imag_bb_signal_sample;
    __m128i realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output, real_output_i32, imag_output_i32;

    __m128 real_E_code_acc, imag_E_code_acc, real_P_code_acc, imag_P_code_acc, real_L_code_acc, imag_L_code_acc;
    __m128i real_output_i_1, real_output_i_2, imag_output_i_1, imag_output_i_2;
    __m128 real_output_ps, imag_output_ps;

    float E_out_real = 0;
    float E_out_imag = 0;
    float P_out_real = 0;
    float P_out_imag = 0;
    float L_out_real = 0;
    float L_out_imag = 0;

    const lv_16sc_t* input_ptr = input;
    const lv_16sc_t* carrier_ptr = carrier;

    const lv_16sc_t* E_code_ptr = E_code;
    lv_32fc_t* E_out_ptr = E_out;
    const lv_16sc_t* L_code_ptr = L_code;
    lv_32fc_t* L_out_ptr = L_out;
    const lv_16sc_t* P_code_ptr = P_code;
    lv_32fc_t* P_out_ptr = P_out;

    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;

    real_E_code_acc = _mm_setzero_ps();
    imag_E_code_acc = _mm_setzero_ps();
    real_P_code_acc = _mm_setzero_ps();
    imag_P_code_acc = _mm_setzero_ps();
    real_L_code_acc = _mm_setzero_ps();
    imag_L_code_acc = _mm_setzero_ps();

    if (sse_iters>0)
        {
            for(unsigned int number = 0;number < sse_iters; number++)
                {
                    //Perform the carrier wipe-off
                    x1 = _mm_lddqu_si128((__m128i*)input_ptr);
                    input_ptr += 4;
                    x2 = _mm_lddqu_si128((__m128i*)input_ptr);

                    y1 = _mm_lddqu_si128((__m128i*)carrier_ptr);
                    carrier_ptr += 4;
                    y2 = _mm_lddqu_si128((__m128i*)carrier_ptr);

                    imagx = _mm_srli_si128 (x1, 2);
                    imagx = _mm_blend_epi16 (x2, imagx, 85);
                    realx = _mm_slli_si128 (x2, 2);
                    realx = _mm_blend_epi16 (realx, x1, 85);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (realx, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imagx, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (realx, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imagx, realy);

                    real_bb_signal_sample = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_bb_signal_sample = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

                    //Get early values
                    y1 = _mm_lddqu_si128((__m128i*)E_code_ptr);
                    E_code_ptr += 4;
                    y2 = _mm_lddqu_si128((__m128i*)E_code_ptr);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

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
                    y1 = _mm_lddqu_si128((__m128i*)P_code_ptr);
                    P_code_ptr += 4;
                    y2 = _mm_lddqu_si128((__m128i*)P_code_ptr);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

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
                    y1 = _mm_lddqu_si128((__m128i*)L_code_ptr);
                    L_code_ptr += 4;
                    y2 = _mm_lddqu_si128((__m128i*)L_code_ptr);

                    imagy = _mm_srli_si128 (y1, 2);
                    imagy = _mm_blend_epi16 (y2, imagy, 85);
                    realy = _mm_slli_si128 (y2, 2);
                    realy = _mm_blend_epi16 (realy, y1, 85);

                    realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
                    imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
                    realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
                    imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);

                    real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
                    imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);

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

                    input_ptr += 4;
                    carrier_ptr += 4;
                    E_code_ptr += 4;
                    L_code_ptr += 4;
                    P_code_ptr += 4;
                }

            __VOLK_ATTR_ALIGNED(16) float real_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_L_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_L_dotProductVector[4];

            _mm_storeu_ps((float*)real_E_dotProductVector,real_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_E_dotProductVector,imag_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_P_dotProductVector,real_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_P_dotProductVector,imag_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_L_dotProductVector,real_L_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_L_dotProductVector,imag_L_code_acc); // Store the results back into the dot product vector

            for (int i = 0; i<4; ++i)
                {
                    E_out_real += real_E_dotProductVector[i];
                    E_out_imag += imag_E_dotProductVector[i];
                    P_out_real += real_P_dotProductVector[i];
                    P_out_imag += imag_P_dotProductVector[i];
                    L_out_real += real_L_dotProductVector[i];
                    L_out_imag += imag_L_dotProductVector[i];
                }
            *E_out_ptr = lv_cmake(E_out_real, E_out_imag);
            *P_out_ptr = lv_cmake(P_out_real, P_out_imag);
            *L_out_ptr = lv_cmake(L_out_real, L_out_imag);
        }

    lv_16sc_t bb_signal_sample;
    for(unsigned int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get early, late, and prompt values for each
            *E_out_ptr += (lv_32fc_t) (bb_signal_sample * (*E_code_ptr++));
            *P_out_ptr += (lv_32fc_t) (bb_signal_sample * (*P_code_ptr++));
            *L_out_ptr += (lv_32fc_t) (bb_signal_sample * (*L_code_ptr++));
        }
}
#endif /* LV_HAVE_SSE4_1 */

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
#include "CommonMacros/CommonMacros.h"
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

static inline void volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_u_sse4_1_fifth(lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;

    __m128i realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy;
    __m128i input_i_1, input_i_2, output_i32;

    __m128i x1, x2, y1, y2, real_bb_signal_sample, imag_bb_signal_sample;
    __m128i realx, imagx, realy, imagy, real_output, imag_output;

    __m128 real_E_code_acc, imag_E_code_acc, real_P_code_acc, imag_P_code_acc, real_L_code_acc, imag_L_code_acc;
    __m128 real_output_ps, imag_output_ps;

    float E_out_real = 0;
    float E_out_imag = 0;
    float P_out_real = 0;
    float P_out_imag = 0;
    float L_out_real = 0;
    float L_out_imag = 0;

    const lv_16sc_t* input_ptr = input;
    const lv_16sc_t* carrier_ptr = carrier;

    const lv_16sc_t* E_code_ptr = E_code;
    lv_32fc_t* E_out_ptr = E_out;
    const lv_16sc_t* L_code_ptr = L_code;
    lv_32fc_t* L_out_ptr = L_out;
    const lv_16sc_t* P_code_ptr = P_code;
    lv_32fc_t* P_out_ptr = P_out;

    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;

    real_E_code_acc = _mm_setzero_ps();
    imag_E_code_acc = _mm_setzero_ps();
    real_P_code_acc = _mm_setzero_ps();
    imag_P_code_acc = _mm_setzero_ps();
    real_L_code_acc = _mm_setzero_ps();
    imag_L_code_acc = _mm_setzero_ps();

    if (sse_iters>0)
        {
            for(unsigned int number = 0;number < sse_iters; number++)
                {
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

                    //Get early values
                    y1 = _mm_lddqu_si128((__m128i*)E_code_ptr);
                    E_code_ptr += 4;
                    y2 = _mm_lddqu_si128((__m128i*)E_code_ptr);

                    CM_16IC_X2_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE4_1(y1, y2, realy, imagy)
                    CM_16IC_X4_SCALAR_PRODUCT_16IC_X2_U_SSE2(real_bb_signal_sample, imag_bb_signal_sample, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output)

                    CM_16IC_CONVERT_AND_ACC_32FC_U_SSE4_1(real_output, input_i_1, input_i_2, output_i32, real_output_ps)
                    CM_16IC_CONVERT_AND_ACC_32FC_U_SSE4_1(imag_output, input_i_1, input_i_2, output_i32, imag_output_ps)

                    real_E_code_acc = _mm_add_ps (real_E_code_acc, real_output_ps);
                    imag_E_code_acc = _mm_add_ps (imag_E_code_acc, imag_output_ps);

                    //Get prompt values
                    y1 = _mm_lddqu_si128((__m128i*)P_code_ptr);
                    P_code_ptr += 4;
                    y2 = _mm_lddqu_si128((__m128i*)P_code_ptr);

                    CM_16IC_X2_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE4_1(y1, y2, realy, imagy)
                    CM_16IC_X4_SCALAR_PRODUCT_16IC_X2_U_SSE2(real_bb_signal_sample, imag_bb_signal_sample, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output)

                    CM_16IC_CONVERT_AND_ACC_32FC_U_SSE4_1(real_output, input_i_1, input_i_2, output_i32, real_output_ps)
                    CM_16IC_CONVERT_AND_ACC_32FC_U_SSE4_1(imag_output, input_i_1, input_i_2, output_i32, imag_output_ps)

                    real_P_code_acc = _mm_add_ps (real_P_code_acc, real_output_ps);
                    imag_P_code_acc = _mm_add_ps (imag_P_code_acc, imag_output_ps);

                    //Get late values
                    y1 = _mm_lddqu_si128((__m128i*)L_code_ptr);
                    L_code_ptr += 4;
                    y2 = _mm_lddqu_si128((__m128i*)L_code_ptr);

                    CM_16IC_X2_REARRANGE_VECTOR_INTO_REAL_IMAG_16IC_X2_U_SSE4_1(y1, y2, realy, imagy)
                    CM_16IC_X4_SCALAR_PRODUCT_16IC_X2_U_SSE2(real_bb_signal_sample, imag_bb_signal_sample, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output)

                    CM_16IC_CONVERT_AND_ACC_32FC_U_SSE4_1(real_output, input_i_1, input_i_2, output_i32, real_output_ps)
                    CM_16IC_CONVERT_AND_ACC_32FC_U_SSE4_1(imag_output, input_i_1, input_i_2, output_i32, imag_output_ps)

                    real_L_code_acc = _mm_add_ps (real_L_code_acc, real_output_ps);
                    imag_L_code_acc = _mm_add_ps (imag_L_code_acc, imag_output_ps);

                    input_ptr += 4;
                    carrier_ptr += 4;
                    E_code_ptr += 4;
                    L_code_ptr += 4;
                    P_code_ptr += 4;
                }

            __VOLK_ATTR_ALIGNED(16) float real_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_L_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_L_dotProductVector[4];

            _mm_storeu_ps((float*)real_E_dotProductVector,real_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_E_dotProductVector,imag_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_P_dotProductVector,real_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_P_dotProductVector,imag_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_L_dotProductVector,real_L_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_L_dotProductVector,imag_L_code_acc); // Store the results back into the dot product vector

            for (int i = 0; i<4; ++i)
                {
                    E_out_real += real_E_dotProductVector[i];
                    E_out_imag += imag_E_dotProductVector[i];
                    P_out_real += real_P_dotProductVector[i];
                    P_out_imag += imag_P_dotProductVector[i];
                    L_out_real += real_L_dotProductVector[i];
                    L_out_imag += imag_L_dotProductVector[i];
                }
            *E_out_ptr = lv_cmake(E_out_real, E_out_imag);
            *P_out_ptr = lv_cmake(P_out_real, P_out_imag);
            *L_out_ptr = lv_cmake(L_out_real, L_out_imag);
        }

    lv_16sc_t bb_signal_sample;
    for(unsigned int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get early, late, and prompt values for each
            *E_out_ptr += (lv_32fc_t) (bb_signal_sample * (*E_code_ptr++));
            *P_out_ptr += (lv_32fc_t) (bb_signal_sample * (*P_code_ptr++));
            *L_out_ptr += (lv_32fc_t) (bb_signal_sample * (*L_code_ptr++));
        }
}
#endif /* LV_HAVE_SSE4_1 */

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
#include "CommonMacros/CommonMacros_16ic_cw_epl_corr_32fc.h"
#include "CommonMacros/CommonMacros.h"
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

static inline void volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_u_sse4_1_sixth(lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 8;
    
    __m128i realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy;
    __m128i input_i_1, input_i_2, output_i32;
    
    __m128i x1, x2, y1, y2, real_bb_signal_sample, imag_bb_signal_sample;
    __m128i realx, imagx, realy, imagy, real_output, imag_output;
    
    __m128 real_E_code_acc, imag_E_code_acc, real_P_code_acc, imag_P_code_acc, real_L_code_acc, imag_L_code_acc;
    __m128 real_output_ps, imag_output_ps;
    
    float E_out_real = 0;
    float E_out_imag = 0;
    float P_out_real = 0;
    float P_out_imag = 0;
    float L_out_real = 0;
    float L_out_imag = 0;
    
    const lv_16sc_t* input_ptr = input;
    const lv_16sc_t* carrier_ptr = carrier;
    
    const lv_16sc_t* E_code_ptr = E_code;
    lv_32fc_t* E_out_ptr = E_out;
    const lv_16sc_t* L_code_ptr = L_code;
    lv_32fc_t* L_out_ptr = L_out;
    const lv_16sc_t* P_code_ptr = P_code;
    lv_32fc_t* P_out_ptr = P_out;
    
    *E_out_ptr = 0;
    *P_out_ptr = 0;
    *L_out_ptr = 0;
    
    real_E_code_acc = _mm_setzero_ps();
    imag_E_code_acc = _mm_setzero_ps();
    real_P_code_acc = _mm_setzero_ps();
    imag_P_code_acc = _mm_setzero_ps();
    real_L_code_acc = _mm_setzero_ps();
    imag_L_code_acc = _mm_setzero_ps();
    
    if (sse_iters>0)
        {
            for(unsigned int number = 0;number < sse_iters; number++)
                {
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

                    input_ptr += 4;
                    carrier_ptr += 4;
                    E_code_ptr += 4;
                    L_code_ptr += 4;
                    P_code_ptr += 4;
                }

            __VOLK_ATTR_ALIGNED(16) float real_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_E_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_P_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float real_L_dotProductVector[4];
            __VOLK_ATTR_ALIGNED(16) float imag_L_dotProductVector[4];

            _mm_storeu_ps((float*)real_E_dotProductVector,real_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_E_dotProductVector,imag_E_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_P_dotProductVector,real_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_P_dotProductVector,imag_P_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)real_L_dotProductVector,real_L_code_acc); // Store the results back into the dot product vector
            _mm_storeu_ps((float*)imag_L_dotProductVector,imag_L_code_acc); // Store the results back into the dot product vector

            for (int i = 0; i<4; ++i)
                {
                    E_out_real += real_E_dotProductVector[i];
                    E_out_imag += imag_E_dotProductVector[i];
                    P_out_real += real_P_dotProductVector[i];
                    P_out_imag += imag_P_dotProductVector[i];
                    L_out_real += real_L_dotProductVector[i];
                    L_out_imag += imag_L_dotProductVector[i];
                }
            *E_out_ptr = lv_cmake(E_out_real, E_out_imag);
            *P_out_ptr = lv_cmake(P_out_real, P_out_imag);
            *L_out_ptr = lv_cmake(L_out_real, L_out_imag);
        }

    lv_16sc_t bb_signal_sample;
    for(unsigned int i=0; i < num_points%8; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
            // Now get early, late, and prompt values for each
            *E_out_ptr += (lv_32fc_t) (bb_signal_sample * (*E_code_ptr++));
            *P_out_ptr += (lv_32fc_t) (bb_signal_sample * (*P_code_ptr++));
            *L_out_ptr += (lv_32fc_t) (bb_signal_sample * (*L_code_ptr++));
        }
}
#endif /* LV_HAVE_SSE4_1 */

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
static inline void volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_generic(lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, unsigned int num_points)
{
    lv_16sc_t bb_signal_sample;
    lv_16sc_t tmp1;
    lv_16sc_t tmp2;
    lv_16sc_t tmp3;

    bb_signal_sample = lv_cmake(0, 0);

    *E_out = 0;
    *P_out = 0;
    *L_out = 0;
    // perform Early, Prompt and Late correlation

    for(unsigned int i=0; i < num_points; ++i)
        {
            //Perform the carrier wipe-off
            bb_signal_sample = input[i] * carrier[i];

            tmp1 = bb_signal_sample * E_code[i];
            tmp2 = bb_signal_sample * P_code[i];
            tmp3 = bb_signal_sample * L_code[i];

            // Now get early, late, and prompt values for each
            *E_out += (lv_32fc_t)tmp1;
            *P_out += (lv_32fc_t)tmp2;
            *L_out += (lv_32fc_t)tmp3;
        }
}
#endif /* LV_HAVE_GENERIC */
#endif /* INCLUDED_gnsssdr_volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_u_H */


#ifndef INCLUDED_gnsssdr_volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_a_H
#define INCLUDED_gnsssdr_volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_a_H

#include <inttypes.h>
#include <stdio.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <float.h>
#include <string.h>
//
//#ifdef LV_HAVE_SSE4_1
//#include <smmintrin.h>
///*!
// \brief Performs the carrier wipe-off mixing and the Early, Prompt, and Late correlation
// \param input The input signal input
// \param carrier The carrier signal input
// \param E_code Early PRN code replica input
// \param P_code Early PRN code replica input
// \param L_code Early PRN code replica input
// \param E_out Early correlation output
// \param P_out Early correlation output
// \param L_out Early correlation output
// \param num_points The number of complex values in vectors
// */
//static inline void volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_a_sse4_1(lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, unsigned int num_points)
//{
//    const unsigned int sse_iters = num_points / 8;
//    
//    __m128i x1, x2, y1, y2, real_bb_signal_sample, imag_bb_signal_sample;
//    __m128i mult1, realx, imagx, realy, imagy, realx_mult_realy, imagx_mult_imagy, realx_mult_imagy, imagx_mult_realy, real_output, imag_output;
//    
//    __m128 real_E_code_acc, imag_E_code_acc, real_P_code_acc, imag_P_code_acc, real_L_code_acc, imag_L_code_acc;
//    __m128i real_output_i_1, real_output_i_2, imag_output_i_1, imag_output_i_2;
//    __m128 real_output_ps_1, real_output_ps_2, imag_output_ps_1, imag_output_ps_2;
//    
//    float E_out_real = 0;
//    float E_out_imag = 0;
//    float P_out_real = 0;
//    float P_out_imag = 0;
//    float L_out_real = 0;
//    float L_out_imag = 0;
//    
//    const lv_16sc_t* input_ptr = input;
//    const lv_16sc_t* carrier_ptr = carrier;
//    
//    const lv_16sc_t* E_code_ptr = E_code;
//    lv_32fc_t* E_out_ptr = E_out;
//    const lv_16sc_t* L_code_ptr = L_code;
//    lv_32fc_t* L_out_ptr = L_out;
//    const lv_16sc_t* P_code_ptr = P_code;
//    lv_32fc_t* P_out_ptr = P_out;
//    
//    *E_out_ptr = 0;
//    *P_out_ptr = 0;
//    *L_out_ptr = 0;
//    
//    mult1 = _mm_set_epi8(0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255);
//    
//    real_E_code_acc = _mm_setzero_ps();
//    imag_E_code_acc = _mm_setzero_ps();
//    real_P_code_acc = _mm_setzero_ps();
//    imag_P_code_acc = _mm_setzero_ps();
//    real_L_code_acc = _mm_setzero_ps();
//    imag_L_code_acc = _mm_setzero_ps();
//    
//    if (sse_iters>0)
//    {
//        for(int number = 0;number < sse_iters; number++){
//            
//            //Perform the carrier wipe-off
//            x1 = _mm_lddqu_si128((__m128i*)input_ptr);
//            input_ptr += 4;
//            x2 = _mm_lddqu_si128((__m128i*)input_ptr);
//            
//            y1 = _mm_lddqu_si128((__m128i*)carrier_ptr);
//            carrier_ptr += 4;
//            y2 = _mm_lddqu_si128((__m128i*)carrier_ptr);
//            
//            imagx = _mm_srli_si128 (x1, 2);
//            imagx = _mm_blend_epi16 (x2, imagx, 85);
//            realx = _mm_slli_si128 (x2, 2);
//            realx = _mm_blend_epi16 (realx, x1, 85);
//            
//            imagy = _mm_srli_si128 (y1, 2);
//            imagy = _mm_blend_epi16 (y2, imagy, 85);
//            realy = _mm_slli_si128 (y2, 2);
//            realy = _mm_blend_epi16 (realy, y1, 85);
//            
//            realx_mult_realy = _mm_mullo_epi16 (realx, realy);
//            imagx_mult_imagy = _mm_mullo_epi16 (imagx, imagy);
//            realx_mult_imagy = _mm_mullo_epi16 (realx, imagy);
//            imagx_mult_realy = _mm_mullo_epi16 (imagx, realy);
//            
//            real_bb_signal_sample = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
//            imag_bb_signal_sample = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);
//            
//            //Get early values
//            y1 = _mm_lddqu_si128((__m128i*)E_code_ptr);
//            E_code_ptr += 4;
//            y2 = _mm_lddqu_si128((__m128i*)E_code_ptr);
//            
//            imagy = _mm_srli_si128 (y1, 2);
//            imagy = _mm_blend_epi16 (y2, imagy, 85);
//            realy = _mm_slli_si128 (y2, 2);
//            realy = _mm_blend_epi16 (realy, y1, 85);
//            
//            realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
//            imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
//            realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
//            imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);
//            
//            real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
//            imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);
//            
//            real_output_i_1 = _mm_cvtepi16_epi32(real_output);
//            real_output_ps_1 = _mm_cvtepi32_ps(real_output_i_1);
//            real_output = _mm_srli_si128 (real_output, 8);
//            real_output_i_2 = _mm_cvtepi16_epi32(real_output);
//            real_output_ps_2 = _mm_cvtepi32_ps(real_output_i_2);
//            
//            imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
//            imag_output_ps_1 = _mm_cvtepi32_ps(imag_output_i_1);
//            imag_output = _mm_srli_si128 (imag_output, 8);
//            imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
//            imag_output_ps_2 = _mm_cvtepi32_ps(imag_output_i_2);
//            
//            real_E_code_acc = _mm_add_ps (real_E_code_acc, real_output_ps_1);
//            real_E_code_acc = _mm_add_ps (real_E_code_acc, real_output_ps_2);
//            imag_E_code_acc = _mm_add_ps (imag_E_code_acc, imag_output_ps_1);
//            imag_E_code_acc = _mm_add_ps (imag_E_code_acc, imag_output_ps_2);
//            
//            //Get prompt values
//            y1 = _mm_lddqu_si128((__m128i*)P_code_ptr);
//            P_code_ptr += 4;
//            y2 = _mm_lddqu_si128((__m128i*)P_code_ptr);
//            
//            imagy = _mm_srli_si128 (y1, 2);
//            imagy = _mm_blend_epi16 (y2, imagy, 85);
//            realy = _mm_slli_si128 (y2, 2);
//            realy = _mm_blend_epi16 (realy, y1, 85);
//            
//            realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
//            imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
//            realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
//            imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);
//            
//            real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
//            imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);
//            
//            real_output_i_1 = _mm_cvtepi16_epi32(real_output);
//            real_output_ps_1 = _mm_cvtepi32_ps(real_output_i_1);
//            real_output = _mm_srli_si128 (real_output, 8);
//            real_output_i_2 = _mm_cvtepi16_epi32(real_output);
//            real_output_ps_2 = _mm_cvtepi32_ps(real_output_i_2);
//            
//            imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
//            imag_output_ps_1 = _mm_cvtepi32_ps(imag_output_i_1);
//            imag_output = _mm_srli_si128 (imag_output, 8);
//            imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
//            imag_output_ps_2 = _mm_cvtepi32_ps(imag_output_i_2);
//            
//            real_P_code_acc = _mm_add_ps (real_P_code_acc, real_output_ps_1);
//            real_P_code_acc = _mm_add_ps (real_P_code_acc, real_output_ps_2);
//            imag_P_code_acc = _mm_add_ps (imag_P_code_acc, imag_output_ps_1);
//            imag_P_code_acc = _mm_add_ps (imag_P_code_acc, imag_output_ps_2);
//            
//            //Get late values
//            y1 = _mm_lddqu_si128((__m128i*)L_code_ptr);
//            L_code_ptr += 4;
//            y2 = _mm_lddqu_si128((__m128i*)L_code_ptr);
//            
//            imagy = _mm_srli_si128 (y1, 2);
//            imagy = _mm_blend_epi16 (y2, imagy, 85);
//            realy = _mm_slli_si128 (y2, 2);
//            realy = _mm_blend_epi16 (realy, y1, 85);
//            
//            realx_mult_realy = _mm_mullo_epi16 (real_bb_signal_sample, realy);
//            imagx_mult_imagy = _mm_mullo_epi16 (imag_bb_signal_sample, imagy);
//            realx_mult_imagy = _mm_mullo_epi16 (real_bb_signal_sample, imagy);
//            imagx_mult_realy = _mm_mullo_epi16 (imag_bb_signal_sample, realy);
//            
//            real_output = _mm_sub_epi16 (realx_mult_realy, imagx_mult_imagy);
//            imag_output = _mm_add_epi16 (realx_mult_imagy, imagx_mult_realy);
//            
//            real_output_i_1 = _mm_cvtepi16_epi32(real_output);
//            real_output_ps_1 = _mm_cvtepi32_ps(real_output_i_1);
//            real_output = _mm_srli_si128 (real_output, 8);
//            real_output_i_2 = _mm_cvtepi16_epi32(real_output);
//            real_output_ps_2 = _mm_cvtepi32_ps(real_output_i_2);
//            
//            imag_output_i_1 = _mm_cvtepi16_epi32(imag_output);
//            imag_output_ps_1 = _mm_cvtepi32_ps(imag_output_i_1);
//            imag_output = _mm_srli_si128 (imag_output, 8);
//            imag_output_i_2 = _mm_cvtepi16_epi32(imag_output);
//            imag_output_ps_2 = _mm_cvtepi32_ps(imag_output_i_2);
//            
//            real_L_code_acc = _mm_add_ps (real_L_code_acc, real_output_ps_1);
//            real_L_code_acc = _mm_add_ps (real_L_code_acc, real_output_ps_2);
//            imag_L_code_acc = _mm_add_ps (imag_L_code_acc, imag_output_ps_1);
//            imag_L_code_acc = _mm_add_ps (imag_L_code_acc, imag_output_ps_2);
//            
//            input_ptr += 4;
//            carrier_ptr += 4;
//            E_code_ptr += 4;
//            L_code_ptr += 4;
//            P_code_ptr += 4;
//        }
//        
//        __VOLK_ATTR_ALIGNED(16) float real_E_dotProductVector[4];
//        __VOLK_ATTR_ALIGNED(16) float imag_E_dotProductVector[4];
//        __VOLK_ATTR_ALIGNED(16) float real_P_dotProductVector[4];
//        __VOLK_ATTR_ALIGNED(16) float imag_P_dotProductVector[4];
//        __VOLK_ATTR_ALIGNED(16) float real_L_dotProductVector[4];
//        __VOLK_ATTR_ALIGNED(16) float imag_L_dotProductVector[4];
//        
//        _mm_storeu_ps((float*)real_E_dotProductVector,real_E_code_acc); // Store the results back into the dot product vector
//        _mm_storeu_ps((float*)imag_E_dotProductVector,imag_E_code_acc); // Store the results back into the dot product vector
//        _mm_storeu_ps((float*)real_P_dotProductVector,real_P_code_acc); // Store the results back into the dot product vector
//        _mm_storeu_ps((float*)imag_P_dotProductVector,imag_P_code_acc); // Store the results back into the dot product vector
//        _mm_storeu_ps((float*)real_L_dotProductVector,real_L_code_acc); // Store the results back into the dot product vector
//        _mm_storeu_ps((float*)imag_L_dotProductVector,imag_L_code_acc); // Store the results back into the dot product vector
//        
//        for (int i = 0; i<4; ++i)
//        {
//            E_out_real += real_E_dotProductVector[i];
//            E_out_imag += imag_E_dotProductVector[i];
//            P_out_real += real_P_dotProductVector[i];
//            P_out_imag += imag_P_dotProductVector[i];
//            L_out_real += real_L_dotProductVector[i];
//            L_out_imag += imag_L_dotProductVector[i];
//        }
//        *E_out_ptr = lv_cmake(E_out_real, E_out_imag);
//        *P_out_ptr = lv_cmake(P_out_real, P_out_imag);
//        *L_out_ptr = lv_cmake(L_out_real, L_out_imag);
//    }
//    
//    lv_16sc_t bb_signal_sample;
//    for(int i=0; i < num_points%8; ++i)
//    {
//        //Perform the carrier wipe-off
//        bb_signal_sample = (*input_ptr++) * (*carrier_ptr++);
//        // Now get early, late, and prompt values for each
//        *E_out_ptr += (lv_32fc_t) (bb_signal_sample * (*E_code_ptr++));
//        *P_out_ptr += (lv_32fc_t) (bb_signal_sample * (*P_code_ptr++));
//        *L_out_ptr += (lv_32fc_t) (bb_signal_sample * (*L_code_ptr++));
//    }
//}
//#endif /* LV_HAVE_SSE4_1 */
//
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
static inline void volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_a_generic(lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, const lv_16sc_t* input, const lv_16sc_t* carrier, const lv_16sc_t* E_code, const lv_16sc_t* P_code, const lv_16sc_t* L_code, unsigned int num_points)
{
    lv_16sc_t bb_signal_sample;
    lv_16sc_t tmp1;
    lv_16sc_t tmp2;
    lv_16sc_t tmp3;
    
    bb_signal_sample = lv_cmake(0, 0);
    
    *E_out = 0;
    *P_out = 0;
    *L_out = 0;
    // perform Early, Prompt and Late correlation
    
    for(unsigned int i=0; i < num_points; ++i)
    {
        //Perform the carrier wipe-off
        bb_signal_sample = input[i] * carrier[i];
        
        tmp1 = bb_signal_sample * E_code[i];
        tmp2 = bb_signal_sample * P_code[i];
        tmp3 = bb_signal_sample * L_code[i];
        
        // Now get early, late, and prompt values for each
        *E_out += (lv_32fc_t)tmp1;
        *P_out += (lv_32fc_t)tmp2;
        *L_out += (lv_32fc_t)tmp3;
    }
}
#endif /* LV_HAVE_GENERIC */
#endif /* INCLUDED_gnsssdr_volk_gnsssdr_16ic_x5_cw_epl_corr_TEST_32fc_x3_a_H */
