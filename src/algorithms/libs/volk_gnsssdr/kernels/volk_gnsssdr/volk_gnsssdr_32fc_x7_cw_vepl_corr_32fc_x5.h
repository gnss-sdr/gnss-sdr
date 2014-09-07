/*!
 * \file volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5
 * \brief Volk protokernel: performs the carrier wipe-off mixing and the VE, Early, Prompt, Late and VL correlation with 64 bits vectors
 * \authors <ul>
 *          <li>Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Andrés Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that performs the carrier wipe-off mixing and the
 * VE, Early, Prompt, Late and VL correlation with 64 bits vectors (32 bits the
 * real part and 32 bits the imaginary part):
 * - The carrier wipe-off is done by multiplying the input signal by the
 * carrier (multiplication of 64 bits vectors) It returns the input
 * signal in base band (BB)
 * - VE values are calculated by multiplying the input signal in BB by the
 * VE code (multiplication of 64 bits vectors), accumulating the results
 * - Early values are calculated by multiplying the input signal in BB by the
 * early code (multiplication of 64 bits vectors), accumulating the results
 * - Prompt values are calculated by multiplying the input signal in BB by the
 * prompt code (multiplication of 64 bits vectors), accumulating the results
 * - Late values are calculated by multiplying the input signal in BB by the
 * late code (multiplication of 64 bits vectors), accumulating the results
 * - VL values are calculated by multiplying the input signal in BB by the
 * VL code (multiplication of 64 bits vectors), accumulating the results
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef INCLUDED_gnsssdr_volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5_u_H
#define INCLUDED_gnsssdr_volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5_u_H

#include <inttypes.h>
#include <stdio.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <float.h>
#include <string.h>

/*!
 * TODO: Code the SSE4 version and benchmark it
 */
#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>


  /*!
    \brief Performs the carrier wipe-off mixing and the VE, Early, Prompt, Late and VL correlation
    \param input The input signal input
    \param carrier The carrier signal input
    \param VE_code VE PRN code replica input
    \param E_code Early PRN code replica input
    \param P_code Early PRN code replica input
    \param L_code Early PRN code replica input
    \param VL_code VL PRN code replica input
    \param VE_out VE correlation output
    \param E_out Early correlation output
    \param P_out Early correlation output
    \param L_out Early correlation output
    \param VL_out VL correlation output
    \param num_points The number of complex values in vectors
  */
static inline void volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5_u_sse3(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_32fc_t* input, const lv_32fc_t* carrier, const lv_32fc_t* VE_code, const lv_32fc_t* E_code, const lv_32fc_t* P_code, const lv_32fc_t* L_code, const lv_32fc_t* VL_code, unsigned int num_points)
{
	unsigned int number = 0;
    const unsigned int halfPoints = num_points / 2;

    lv_32fc_t dotProduct_VE;
    memset(&dotProduct_VE, 0x0, 2*sizeof(float));
    lv_32fc_t dotProduct_E;
    memset(&dotProduct_E, 0x0, 2*sizeof(float));
    lv_32fc_t dotProduct_P;
    memset(&dotProduct_P, 0x0, 2*sizeof(float));
    lv_32fc_t dotProduct_L;
    memset(&dotProduct_L, 0x0, 2*sizeof(float));
    lv_32fc_t dotProduct_VL;
    memset(&dotProduct_VL, 0x0, 2*sizeof(float));

    // Aux vars
    __m128 x, y, yl, yh, z, tmp1, tmp2, z_VE, z_E, z_P, z_L, z_VL;
    __m128 bb_signal_sample, bb_signal_sample_shuffled;

    z_VE = _mm_setzero_ps();
    z_E = _mm_setzero_ps();
    z_P = _mm_setzero_ps();
    z_L = _mm_setzero_ps();
    z_VL = _mm_setzero_ps();

    //input and output vectors
    const lv_32fc_t* _input = input;
    const lv_32fc_t* _carrier = carrier;
    const lv_32fc_t* _VE_code = VE_code;
    const lv_32fc_t* _E_code = E_code;
    const lv_32fc_t* _P_code = P_code;
    const lv_32fc_t* _L_code = L_code;
    const lv_32fc_t* _VL_code = VL_code;

    for(;number < halfPoints; number++)
    {
      // carrier wipe-off (vector point-to-point product)
      x = _mm_loadu_ps((float*)_input); // Load the ar + ai, br + bi as ar,ai,br,bi
      y = _mm_loadu_ps((float*)_carrier); // Load the cr + ci, dr + di as cr,ci,dr,di

      yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
      yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di

      tmp1 = _mm_mul_ps(x,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr

      x = _mm_shuffle_ps(x,x,0xB1); // Re-arrange x to be ai,ar,bi,br

      tmp2 = _mm_mul_ps(x,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di

      bb_signal_sample = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
      bb_signal_sample_shuffled = _mm_shuffle_ps(bb_signal_sample,bb_signal_sample,0xB1); // Re-arrange bb_signal_sample to be ai,ar,bi,br

      // correlation VE,E,P,L,VL (5x vector scalar product)
      // VE
      y = _mm_loadu_ps((float*)_VE_code); // Load the cr + ci, dr + di as cr,ci,dr,di
        
      yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
      yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di
        
      tmp1 = _mm_mul_ps(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
      tmp2 = _mm_mul_ps(bb_signal_sample_shuffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
        
      z = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
      z_VE = _mm_add_ps(z_VE, z); // Add the complex multiplication results together
        
      // Early
      y = _mm_loadu_ps((float*)_E_code); // Load the cr + ci, dr + di as cr,ci,dr,di
        
      yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
      yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di
        
      tmp1 = _mm_mul_ps(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
      tmp2 = _mm_mul_ps(bb_signal_sample_shuffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
        
      z = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
      z_E = _mm_add_ps(z_E, z); // Add the complex multiplication results together

      // Prompt
      y = _mm_loadu_ps((float*)_P_code); // Load the cr + ci, dr + di as cr,ci,dr,di

      yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
      yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di

      tmp1 = _mm_mul_ps(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
      tmp2 = _mm_mul_ps(bb_signal_sample_shuffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
        
      z = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
      z_P = _mm_add_ps(z_P, z); // Add the complex multiplication results together

      // Late
      y = _mm_loadu_ps((float*)_L_code); // Load the cr + ci, dr + di as cr,ci,dr,di

      yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
      yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di

      tmp1 = _mm_mul_ps(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
      tmp2 = _mm_mul_ps(bb_signal_sample_shuffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
        
      z = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
      z_L = _mm_add_ps(z_L, z); // Add the complex multiplication results together
        
      // VL
      //x = _mm_load_ps((float*)_input_BB); // Load the ar + ai, br + bi as ar,ai,br,bi
      y = _mm_loadu_ps((float*)_VL_code); // Load the cr + ci, dr + di as cr,ci,dr,di
        
      yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
      yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di
        
      tmp1 = _mm_mul_ps(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
      tmp2 = _mm_mul_ps(bb_signal_sample_shuffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
        
      z = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
      z_VL = _mm_add_ps(z_VL, z); // Add the complex multiplication results together

      /*pointer increment*/
      _carrier += 2;
      _input += 2;
      _VE_code += 2;
      _E_code += 2;
      _P_code += 2;
      _L_code +=2;
      _VL_code +=2;
    }

    __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_VE[2];
    __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_E[2];
    __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_P[2];
    __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_L[2];
    __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_VL[2];

    _mm_storeu_ps((float*)dotProductVector_VE,z_VE); // Store the results back into the dot product vector
    _mm_storeu_ps((float*)dotProductVector_E,z_E); // Store the results back into the dot product vector
    _mm_storeu_ps((float*)dotProductVector_P,z_P); // Store the results back into the dot product vector
    _mm_storeu_ps((float*)dotProductVector_L,z_L); // Store the results back into the dot product vector
    _mm_storeu_ps((float*)dotProductVector_VL,z_VL); // Store the results back into the dot product vector

    dotProduct_VE += ( dotProductVector_VE[0] + dotProductVector_VE[1] );
    dotProduct_E += ( dotProductVector_E[0] + dotProductVector_E[1] );
    dotProduct_P += ( dotProductVector_P[0] + dotProductVector_P[1] );
    dotProduct_L += ( dotProductVector_L[0] + dotProductVector_L[1] );
    dotProduct_VL += ( dotProductVector_VL[0] + dotProductVector_VL[1] );

    if((num_points % 2) != 0)
    {
  	  dotProduct_VE += (*_input) * (*_VE_code)*(*_carrier);
  	  dotProduct_E += (*_input) * (*_E_code)*(*_carrier);
  	  dotProduct_P += (*_input) * (*_P_code)*(*_carrier);
  	  dotProduct_L += (*_input) * (*_L_code)*(*_carrier);
  	  dotProduct_VL += (*_input) * (*_VL_code)*(*_carrier);
    }

    *VE_out = dotProduct_VE;
    *E_out = dotProduct_E;
    *P_out = dotProduct_P;
    *L_out = dotProduct_L;
    *VL_out = dotProduct_VL;
}

#endif /* LV_HAVE_SSE3 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Performs the carrier wipe-off mixing and the VE, Early, Prompt, Late and VL correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param VE_code VE PRN code replica input
 \param E_code Early PRN code replica input
 \param P_code Early PRN code replica input
 \param L_code Early PRN code replica input
 \param VL_code VL PRN code replica input
 \param VE_out VE correlation output
 \param E_out Early correlation output
 \param P_out Early correlation output
 \param L_out Early correlation output
 \param VL_out VL correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5_generic(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_32fc_t* input, const lv_32fc_t* carrier, const lv_32fc_t* VE_code, const lv_32fc_t* E_code, const lv_32fc_t* P_code, const lv_32fc_t* L_code, const lv_32fc_t* VL_code, unsigned int num_points)
{
    lv_32fc_t bb_signal_sample;
    
    bb_signal_sample = lv_cmake(0, 0);
    
    *VE_out = 0;
    *E_out = 0;
    *P_out = 0;
    *L_out = 0;
    *VL_out = 0;
    // perform Early, Prompt and Late correlation
    for(int i=0; i < num_points; ++i)
    {
        //Perform the carrier wipe-off
        bb_signal_sample = input[i] * carrier[i];
        // Now get early, late, and prompt values for each
        *VE_out += bb_signal_sample * VE_code[i];
        *E_out += bb_signal_sample * E_code[i];
        *P_out += bb_signal_sample * P_code[i];
        *L_out += bb_signal_sample * L_code[i];
        *VL_out += bb_signal_sample * VL_code[i];
    }
}

#endif /* LV_HAVE_GENERIC */

#endif /* INCLUDED_gnsssdr_volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5_u_H */


#ifndef INCLUDED_gnsssdr_volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5_a_H
#define INCLUDED_gnsssdr_volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5_a_H

#include <inttypes.h>
#include <stdio.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <float.h>
#include <string.h>

#ifdef LV_HAVE_SSE3
#include <pmmintrin.h>
/*!
 \brief Performs the carrier wipe-off mixing and the VE, Early, Prompt, Late and VL correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param VE_code VE PRN code replica input
 \param E_code Early PRN code replica input
 \param P_code Early PRN code replica input
 \param L_code Early PRN code replica input
 \param VL_code VL PRN code replica input
 \param VE_out VE correlation output
 \param E_out Early correlation output
 \param P_out Early correlation output
 \param L_out Early correlation output
 \param VL_out VL correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5_a_sse3(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_32fc_t* input, const lv_32fc_t* carrier, const lv_32fc_t* VE_code, const lv_32fc_t* E_code, const lv_32fc_t* P_code, const lv_32fc_t* L_code, const lv_32fc_t* VL_code, unsigned int num_points)
{
	unsigned int number = 0;
    const unsigned int halfPoints = num_points / 2;
    
    lv_32fc_t dotProduct_VE;
    memset(&dotProduct_VE, 0x0, 2*sizeof(float));
    lv_32fc_t dotProduct_E;
    memset(&dotProduct_E, 0x0, 2*sizeof(float));
    lv_32fc_t dotProduct_P;
    memset(&dotProduct_P, 0x0, 2*sizeof(float));
    lv_32fc_t dotProduct_L;
    memset(&dotProduct_L, 0x0, 2*sizeof(float));
    lv_32fc_t dotProduct_VL;
    memset(&dotProduct_VL, 0x0, 2*sizeof(float));
    
    // Aux vars
    __m128 x, y, yl, yh, z, tmp1, tmp2, z_VE, z_E, z_P, z_L, z_VL;
    __m128 bb_signal_sample, bb_signal_sample_shuffled;
    
    z_VE = _mm_setzero_ps();
    z_E = _mm_setzero_ps();
    z_P = _mm_setzero_ps();
    z_L = _mm_setzero_ps();
    z_VL = _mm_setzero_ps();
    
    //input and output vectors
    const lv_32fc_t* _input = input;
    const lv_32fc_t* _carrier = carrier;
    const lv_32fc_t* _VE_code = VE_code;
    const lv_32fc_t* _E_code = E_code;
    const lv_32fc_t* _P_code = P_code;
    const lv_32fc_t* _L_code = L_code;
    const lv_32fc_t* _VL_code = VL_code;
    
    for(;number < halfPoints; number++)
    {
        // carrier wipe-off (vector point-to-point product)
        x = _mm_load_ps((float*)_input); // Load the ar + ai, br + bi as ar,ai,br,bi
        y = _mm_load_ps((float*)_carrier); // Load the cr + ci, dr + di as cr,ci,dr,di
        
        yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
        yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di
        
        tmp1 = _mm_mul_ps(x,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
        
        x = _mm_shuffle_ps(x,x,0xB1); // Re-arrange x to be ai,ar,bi,br
        
        tmp2 = _mm_mul_ps(x,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
        
        bb_signal_sample = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
        bb_signal_sample_shuffled = _mm_shuffle_ps(bb_signal_sample,bb_signal_sample,0xB1); // Re-arrange bb_signal_sample to be ai,ar,bi,br
        
        // correlation VE,E,P,L,VL (5x vector scalar product)
        // VE
        y = _mm_load_ps((float*)_VE_code); // Load the cr + ci, dr + di as cr,ci,dr,di
        
        yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
        yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di
        
        tmp1 = _mm_mul_ps(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
        tmp2 = _mm_mul_ps(bb_signal_sample_shuffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
        
        z = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
        z_VE = _mm_add_ps(z_VE, z); // Add the complex multiplication results together
        
        // Early
        y = _mm_load_ps((float*)_E_code); // Load the cr + ci, dr + di as cr,ci,dr,di
        
        yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
        yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di
        
        tmp1 = _mm_mul_ps(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
        tmp2 = _mm_mul_ps(bb_signal_sample_shuffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
        
        z = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
        z_E = _mm_add_ps(z_E, z); // Add the complex multiplication results together
        
        // Prompt
        y = _mm_load_ps((float*)_P_code); // Load the cr + ci, dr + di as cr,ci,dr,di
        
        yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
        yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di
        
        tmp1 = _mm_mul_ps(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
        tmp2 = _mm_mul_ps(bb_signal_sample_shuffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
        
        z = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
        z_P = _mm_add_ps(z_P, z); // Add the complex multiplication results together
        
        // Late
        y = _mm_load_ps((float*)_L_code); // Load the cr + ci, dr + di as cr,ci,dr,di
        
        yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
        yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di
        
        tmp1 = _mm_mul_ps(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
        tmp2 = _mm_mul_ps(bb_signal_sample_shuffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
        
        z = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
        z_L = _mm_add_ps(z_L, z); // Add the complex multiplication results together
        
        // VL
        //x = _mm_load_ps((float*)_input_BB); // Load the ar + ai, br + bi as ar,ai,br,bi
        y = _mm_load_ps((float*)_VL_code); // Load the cr + ci, dr + di as cr,ci,dr,di
        
        yl = _mm_moveldup_ps(y); // Load yl with cr,cr,dr,dr
        yh = _mm_movehdup_ps(y); // Load yh with ci,ci,di,di
        
        tmp1 = _mm_mul_ps(bb_signal_sample,yl); // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
        tmp2 = _mm_mul_ps(bb_signal_sample_shuffled,yh); // tmp2 = ai*ci,ar*ci,bi*di,br*di
        
        z = _mm_addsub_ps(tmp1,tmp2); // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
        z_VL = _mm_add_ps(z_VL, z); // Add the complex multiplication results together
        
        /*pointer increment*/
        _carrier += 2;
        _input += 2;
        _VE_code += 2;
        _E_code += 2;
        _P_code += 2;
        _L_code +=2;
        _VL_code +=2;
    }
    
    __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_VE[2];
    __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_E[2];
    __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_P[2];
    __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_L[2];
    __VOLK_ATTR_ALIGNED(16) lv_32fc_t dotProductVector_VL[2];
    
    _mm_store_ps((float*)dotProductVector_VE,z_VE); // Store the results back into the dot product vector
    _mm_store_ps((float*)dotProductVector_E,z_E); // Store the results back into the dot product vector
    _mm_store_ps((float*)dotProductVector_P,z_P); // Store the results back into the dot product vector
    _mm_store_ps((float*)dotProductVector_L,z_L); // Store the results back into the dot product vector
    _mm_store_ps((float*)dotProductVector_VL,z_VL); // Store the results back into the dot product vector
    
    dotProduct_VE += ( dotProductVector_VE[0] + dotProductVector_VE[1] );
    dotProduct_E += ( dotProductVector_E[0] + dotProductVector_E[1] );
    dotProduct_P += ( dotProductVector_P[0] + dotProductVector_P[1] );
    dotProduct_L += ( dotProductVector_L[0] + dotProductVector_L[1] );
    dotProduct_VL += ( dotProductVector_VL[0] + dotProductVector_VL[1] );
    
    if((num_points % 2) != 0)
    {
        dotProduct_VE += (*_input) * (*_VE_code)*(*_carrier);
        dotProduct_E += (*_input) * (*_E_code)*(*_carrier);
        dotProduct_P += (*_input) * (*_P_code)*(*_carrier);
        dotProduct_L += (*_input) * (*_L_code)*(*_carrier);
        dotProduct_VL += (*_input) * (*_VL_code)*(*_carrier);
    }
    
    *VE_out = dotProduct_VE;
    *E_out = dotProduct_E;
    *P_out = dotProduct_P;
    *L_out = dotProduct_L;
    *VL_out = dotProduct_VL;

}

#endif /* LV_HAVE_SSE3 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Performs the carrier wipe-off mixing and the VE, Early, Prompt, Late and VL correlation
 \param input The input signal input
 \param carrier The carrier signal input
 \param VE_code VE PRN code replica input
 \param E_code Early PRN code replica input
 \param P_code Early PRN code replica input
 \param L_code Early PRN code replica input
 \param VL_code VL PRN code replica input
 \param VE_out VE correlation output
 \param E_out Early correlation output
 \param P_out Early correlation output
 \param L_out Early correlation output
 \param VL_out VL correlation output
 \param num_points The number of complex values in vectors
 */
static inline void volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5_a_generic(lv_32fc_t* VE_out, lv_32fc_t* E_out, lv_32fc_t* P_out, lv_32fc_t* L_out, lv_32fc_t* VL_out, const lv_32fc_t* input, const lv_32fc_t* carrier, const lv_32fc_t* VE_code, const lv_32fc_t* E_code, const lv_32fc_t* P_code, const lv_32fc_t* L_code, const lv_32fc_t* VL_code, unsigned int num_points)
{
    lv_32fc_t bb_signal_sample;
    
    bb_signal_sample = lv_cmake(0, 0);
    
    *VE_out = 0;
    *E_out = 0;
    *P_out = 0;
    *L_out = 0;
    *VL_out = 0;
    // perform Early, Prompt and Late correlation
    for(int i=0; i < num_points; ++i)
    {
        //Perform the carrier wipe-off
        bb_signal_sample = input[i] * carrier[i];
        // Now get early, late, and prompt values for each
        *VE_out += bb_signal_sample * VE_code[i];
        *E_out += bb_signal_sample * E_code[i];
        *P_out += bb_signal_sample * P_code[i];
        *L_out += bb_signal_sample * L_code[i];
        *VL_out += bb_signal_sample * VL_code[i];
    }
}

#endif /* LV_HAVE_GENERIC */

#endif /* INCLUDED_gnsssdr_volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5_a_H */
