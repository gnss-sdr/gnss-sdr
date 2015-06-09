/*!
 * \file volk_gnsssdr_32fc_s32f_x2_update_local_carrier_32fc
 * \brief Volk protokernel: replaces the tracking function for update_local_carrier. Algorithm by Julien Pommier and Giovanni Garberoglio, modified by Andres Cecilia.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that replaces the tracking function for update_local_carrier. Algorithm by Julien Pommier and Giovanni Garberoglio, modified by Andres Cecilia.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2007  Julien Pommier
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * (this is the zlib license)
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012 Giovanni Garberoglio
 * Interdisciplinary Laboratory for Computational Science (LISC)
 * Fondazione Bruno Kessler and University of Trento
 * via Sommarive, 18
 * I-38123 Trento (Italy)
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

#ifndef INCLUDED_volk_gnsssdr_32fc_s32f_x2_update_local_carrier_32fc_u_H
#define INCLUDED_volk_gnsssdr_32fc_s32f_x2_update_local_carrier_32fc_u_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <inttypes.h>
#include <stdio.h>

#ifdef LV_HAVE_AVX
#include <immintrin.h>
/*!
 \brief Accumulates the values in the input buffer
 \param result The accumulated result
 \param inputBuffer The buffer of data to be accumulated
 \param num_points The number of values in inputBuffer to be accumulated
 */
static inline void volk_gnsssdr_s32f_x2_update_local_carrier_32fc_u_avx(lv_32fc_t* d_carr_sign, const float phase_rad_init, const float phase_step_rad, unsigned int num_points)
{
    //    float* pointer1 = (float*)&phase_rad_init;
    //    *pointer1 = 0;
    //    float* pointer2 = (float*)&phase_step_rad;
    //    *pointer2 = 0.5;

    const unsigned int sse_iters = num_points / 8;

    __m256 _ps256_minus_cephes_DP1 = _mm256_set1_ps(-0.78515625f);
    __m256 _ps256_minus_cephes_DP2 = _mm256_set1_ps(-2.4187564849853515625e-4f);
    __m256 _ps256_minus_cephes_DP3 = _mm256_set1_ps(-3.77489497744594108e-8f);
    __m256 _ps256_sign_mask = _mm256_set1_ps(-0.f);
    __m128i _pi32avx_1 = _mm_set1_epi32(1);
    __m128i _pi32avx_inv1 = _mm_set1_epi32(~1);
    __m128i _pi32avx_2 = _mm_set1_epi32(2);
    __m128i _pi32avx_4 = _mm_set1_epi32(4);
    __m256 _ps256_cephes_FOPI = _mm256_set1_ps(1.27323954473516f); // 4 / PI
    __m256 _ps256_sincof_p0 = _mm256_set1_ps(-1.9515295891E-4f);
    __m256 _ps256_sincof_p1 = _mm256_set1_ps( 8.3321608736E-3f);
    __m256 _ps256_sincof_p2 = _mm256_set1_ps(-1.6666654611E-1f);
    __m256 _ps256_coscof_p0 = _mm256_set1_ps( 2.443315711809948E-005f);
    __m256 _ps256_coscof_p1 = _mm256_set1_ps(-1.388731625493765E-003f);
    __m256 _ps256_coscof_p2 = _mm256_set1_ps( 4.166664568298827E-002f);
    __m256 _ps256_1 = _mm256_set1_ps(1.f);
    __m256 _ps256_0p5 = _mm256_set1_ps(0.5f);

    __m256 phase_step_rad_array = _mm256_set1_ps(8*phase_step_rad);

    __m256 phase_rad_array, x, s, c, swap_sign_bit_sin, sign_bit_cos, poly_mask, z, tmp, y, y2, ysin1, ysin2;
    __m256 xmm1, xmm2, xmm3, sign_bit_sin;
    __m256i imm0, imm2, imm4, tmp256i;
    __m128i imm0_1, imm0_2, imm2_1, imm2_2, imm4_1, imm4_2;
    __VOLK_ATTR_ALIGNED(32) float sin_value[8];
    __VOLK_ATTR_ALIGNED(32) float cos_value[8];

    phase_rad_array = _mm256_set_ps (phase_rad_init+7*phase_step_rad, phase_rad_init+6*phase_step_rad, phase_rad_init+5*phase_step_rad, phase_rad_init+4*phase_step_rad, phase_rad_init+3*phase_step_rad, phase_rad_init+2*phase_step_rad, phase_rad_init+phase_step_rad, phase_rad_init);

    for(int i = 0; i < sse_iters; i++)
        {
            x = phase_rad_array;

            /* extract the sign bit (upper one) */
            sign_bit_sin = _mm256_and_ps(x, _ps256_sign_mask);

            /* take the absolute value */
            x = _mm256_xor_ps(x, sign_bit_sin);

            /* scale by 4/Pi */
            y = _mm256_mul_ps(x, _ps256_cephes_FOPI);

            /* we use SSE2 routines to perform the integer ops */

            //COPY_IMM_TO_XMM(_mm256_cvttps_epi32(y),imm2_1,imm2_2);
            tmp256i = _mm256_cvttps_epi32(y);
            imm2_1 = _mm256_extractf128_si256 (tmp256i, 0);
            imm2_2 = _mm256_extractf128_si256 (tmp256i, 1);

            imm2_1 = _mm_add_epi32(imm2_1, _pi32avx_1);
            imm2_2 = _mm_add_epi32(imm2_2, _pi32avx_1);

            imm2_1 = _mm_and_si128(imm2_1, _pi32avx_inv1);
            imm2_2 = _mm_and_si128(imm2_2, _pi32avx_inv1);

            //COPY_XMM_TO_IMM(imm2_1,imm2_2,imm2);
            //_mm256_set_m128i not defined in some versions of immintrin.h
            //imm2 = _mm256_set_m128i (imm2_2, imm2_1);
            imm2 = _mm256_insertf128_si256(_mm256_castsi128_si256(imm2_1),(imm2_2),1);

            y = _mm256_cvtepi32_ps(imm2);

            imm4_1 = imm2_1;
            imm4_2 = imm2_2;

            imm0_1 = _mm_and_si128(imm2_1, _pi32avx_4);
            imm0_2 = _mm_and_si128(imm2_2, _pi32avx_4);

            imm0_1 = _mm_slli_epi32(imm0_1, 29);
            imm0_2 = _mm_slli_epi32(imm0_2, 29);

            //COPY_XMM_TO_IMM(imm0_1, imm0_2, imm0);
            //_mm256_set_m128i not defined in some versions of immintrin.h
            //imm0 = _mm256_set_m128i (imm0_2, imm0_1);
            imm0 = _mm256_insertf128_si256(_mm256_castsi128_si256(imm0_1),(imm0_2),1);

            imm2_1 = _mm_and_si128(imm2_1, _pi32avx_2);
            imm2_2 = _mm_and_si128(imm2_2, _pi32avx_2);

            imm2_1 = _mm_cmpeq_epi32(imm2_1, _mm_setzero_si128());
            imm2_2 = _mm_cmpeq_epi32(imm2_2, _mm_setzero_si128());

            //COPY_XMM_TO_IMM(imm2_1, imm2_2, imm2);
            //_mm256_set_m128i not defined in some versions of immintrin.h
            //imm2 = _mm256_set_m128i (imm2_2, imm2_1);
            imm2 = _mm256_insertf128_si256(_mm256_castsi128_si256(imm2_1),(imm2_2),1);

            swap_sign_bit_sin = _mm256_castsi256_ps(imm0);
            poly_mask = _mm256_castsi256_ps(imm2);

            /* The magic pass: "Extended precision modular arithmetic"
         x = ((x - y * DP1) - y * DP2) - y * DP3; */
            xmm1 = _ps256_minus_cephes_DP1;
            xmm2 = _ps256_minus_cephes_DP2;
            xmm3 = _ps256_minus_cephes_DP3;
            xmm1 = _mm256_mul_ps(y, xmm1);
            xmm2 = _mm256_mul_ps(y, xmm2);
            xmm3 = _mm256_mul_ps(y, xmm3);
            x = _mm256_add_ps(x, xmm1);
            x = _mm256_add_ps(x, xmm2);
            x = _mm256_add_ps(x, xmm3);

            imm4_1 = _mm_sub_epi32(imm4_1, _pi32avx_2);
            imm4_2 = _mm_sub_epi32(imm4_2, _pi32avx_2);

            imm4_1 = _mm_andnot_si128(imm4_1, _pi32avx_4);
            imm4_2 = _mm_andnot_si128(imm4_2, _pi32avx_4);

            imm4_1 = _mm_slli_epi32(imm4_1, 29);
            imm4_2 = _mm_slli_epi32(imm4_2, 29);

            //COPY_XMM_TO_IMM(imm4_1, imm4_2, imm4);
            //_mm256_set_m128i not defined in some versions of immintrin.h
            //imm4 = _mm256_set_m128i (imm4_2, imm4_1);
            imm4 = _mm256_insertf128_si256(_mm256_castsi128_si256(imm4_1),(imm4_2),1);

            sign_bit_cos = _mm256_castsi256_ps(imm4);

            sign_bit_sin = _mm256_xor_ps(sign_bit_sin, swap_sign_bit_sin);

            /* Evaluate the first polynom  (0 <= x <= Pi/4) */
            z = _mm256_mul_ps(x,x);
            y = _ps256_coscof_p0;

            y = _mm256_mul_ps(y, z);
            y = _mm256_add_ps(y, _ps256_coscof_p1);
            y = _mm256_mul_ps(y, z);
            y = _mm256_add_ps(y, _ps256_coscof_p2);
            y = _mm256_mul_ps(y, z);
            y = _mm256_mul_ps(y, z);
            tmp = _mm256_mul_ps(z, _ps256_0p5);
            y = _mm256_sub_ps(y, tmp);
            y = _mm256_add_ps(y, _ps256_1);

            /* Evaluate the second polynom  (Pi/4 <= x <= 0) */

            y2 = _ps256_sincof_p0;
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_add_ps(y2, _ps256_sincof_p1);
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_add_ps(y2, _ps256_sincof_p2);
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_mul_ps(y2, x);
            y2 = _mm256_add_ps(y2, x);

            /* select the correct result from the two polynoms */
            xmm3 = poly_mask;
            ysin2 = _mm256_and_ps(xmm3, y2);
            ysin1 = _mm256_andnot_ps(xmm3, y);
            y2 = _mm256_sub_ps(y2,ysin2);
            y = _mm256_sub_ps(y, ysin1);

            xmm1 = _mm256_add_ps(ysin1,ysin2);
            xmm2 = _mm256_add_ps(y,y2);

            /* update the sign */
            s = _mm256_xor_ps(xmm1, sign_bit_sin);
            c = _mm256_xor_ps(xmm2, sign_bit_cos);

            //GNSS-SDR needs to return -sin
            s = _mm256_xor_ps(s, _ps256_sign_mask);

            _mm256_storeu_ps ((float*)sin_value, s);
            _mm256_storeu_ps ((float*)cos_value, c);

            for(int i = 0; i < 8; i++)
                {
                    d_carr_sign[i] = lv_cmake(cos_value[i], sin_value[i]);
                }
            d_carr_sign += 8;

            phase_rad_array = _mm256_add_ps (phase_rad_array, phase_step_rad_array);
        }

    if (num_points%8!=0)
        {
            __VOLK_ATTR_ALIGNED(32) float phase_rad_store[8];
            _mm256_storeu_ps ((float*)phase_rad_store, phase_rad_array);

            float phase_rad = phase_rad_store[0];

            for(int i = 0; i < num_points%8; i++)
                {
                    *d_carr_sign = lv_cmake(cos(phase_rad), -sin(phase_rad));
                    d_carr_sign++;
                    phase_rad += phase_step_rad;
                }
        }
}
#endif /* LV_HAVE_AVX */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/*!
  \brief Accumulates the values in the input buffer
  \param result The accumulated result
  \param inputBuffer The buffer of data to be accumulated
  \param num_points The number of values in inputBuffer to be accumulated
 */
static inline void volk_gnsssdr_s32f_x2_update_local_carrier_32fc_u_sse2(lv_32fc_t* d_carr_sign, const float phase_rad_init, const float phase_step_rad, unsigned int num_points)
{
    //    float* pointer1 = (float*)&phase_rad_init;
    //    *pointer1 = 0;
    //    float* pointer2 = (float*)&phase_step_rad;
    //    *pointer2 = 0.5;

    const unsigned int sse_iters = num_points / 4;

    __m128 _ps_minus_cephes_DP1 = _mm_set1_ps(-0.78515625f);
    __m128 _ps_minus_cephes_DP2 = _mm_set1_ps(-2.4187564849853515625e-4f);
    __m128 _ps_minus_cephes_DP3 = _mm_set1_ps(-3.77489497744594108e-8f);
    __m128 _ps_sign_mask = _mm_set1_ps(-0.f);
    __m128i _pi32_1 = _mm_set1_epi32(1);
    __m128i _pi32_inv1 = _mm_set1_epi32(~1);
    __m128i _pi32_2 = _mm_set1_epi32(2);
    __m128i _pi32_4 = _mm_set1_epi32(4);
    __m128 _ps_cephes_FOPI = _mm_set1_ps(1.27323954473516f); // 4 / PI
    __m128 _ps_sincof_p0 = _mm_set1_ps(-1.9515295891E-4f);
    __m128 _ps_sincof_p1 = _mm_set1_ps( 8.3321608736E-3f);
    __m128 _ps_sincof_p2 = _mm_set1_ps(-1.6666654611E-1f);
    __m128 _ps_coscof_p0 = _mm_set1_ps( 2.443315711809948E-005f);
    __m128 _ps_coscof_p1 = _mm_set1_ps(-1.388731625493765E-003f);
    __m128 _ps_coscof_p2 = _mm_set1_ps( 4.166664568298827E-002f);
    __m128 _ps_1 = _mm_set1_ps(1.f);
    __m128 _ps_0p5 = _mm_set1_ps(0.5f);

    __m128 phase_step_rad_array = _mm_set1_ps(4*phase_step_rad);

    __m128 phase_rad_array, x, s, c, swap_sign_bit_sin, sign_bit_cos, poly_mask, z, tmp, y, y2, ysin1, ysin2;
    __m128 xmm1, xmm2, xmm3, sign_bit_sin;
    __m128i emm0, emm2, emm4;
    __VOLK_ATTR_ALIGNED(16) float sin_value[4];
    __VOLK_ATTR_ALIGNED(16) float cos_value[4];

    phase_rad_array = _mm_set_ps (phase_rad_init+3*phase_step_rad, phase_rad_init+2*phase_step_rad, phase_rad_init+phase_step_rad, phase_rad_init);

    for(unsigned int i = 0; i < sse_iters; i++)
        {
            x = phase_rad_array;

            /* extract the sign bit (upper one) */
            sign_bit_sin = _mm_and_ps(x, _ps_sign_mask);

            /* take the absolute value */
            x = _mm_xor_ps(x, sign_bit_sin);

            /* scale by 4/Pi */
            y = _mm_mul_ps(x, _ps_cephes_FOPI);

            /* store the integer part of y in emm2 */
            emm2 = _mm_cvttps_epi32(y);

            /* j=(j+1) & (~1) (see the cephes sources) */
            emm2 = _mm_add_epi32(emm2, _pi32_1);
            emm2 = _mm_and_si128(emm2, _pi32_inv1);
            y = _mm_cvtepi32_ps(emm2);

            emm4 = emm2;

            /* get the swap sign flag for the sine */
            emm0 = _mm_and_si128(emm2, _pi32_4);
            emm0 = _mm_slli_epi32(emm0, 29);
            swap_sign_bit_sin = _mm_castsi128_ps(emm0);

            /* get the polynom selection mask for the sine*/
            emm2 = _mm_and_si128(emm2, _pi32_2);
            emm2 = _mm_cmpeq_epi32(emm2, _mm_setzero_si128());
            poly_mask = _mm_castsi128_ps(emm2);

            /* The magic pass: "Extended precision modular arithmetic"
        x = ((x - y * DP1) - y * DP2) - y * DP3; */
            xmm1 = _mm_mul_ps(y, _ps_minus_cephes_DP1);
            xmm2 = _mm_mul_ps(y, _ps_minus_cephes_DP2);
            xmm3 = _mm_mul_ps(y, _ps_minus_cephes_DP3);
            x = _mm_add_ps(_mm_add_ps(x, xmm1), _mm_add_ps(xmm2, xmm3));

            emm4 = _mm_sub_epi32(emm4, _pi32_2);
            emm4 = _mm_andnot_si128(emm4, _pi32_4);
            emm4 = _mm_slli_epi32(emm4, 29);
            sign_bit_cos = _mm_castsi128_ps(emm4);

            sign_bit_sin = _mm_xor_ps(sign_bit_sin, swap_sign_bit_sin);

            /* Evaluate the first polynom  (0 <= x <= Pi/4) */
            z = _mm_mul_ps(x,x);
            y = _ps_coscof_p0;
            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, _ps_coscof_p1);
            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, _ps_coscof_p2);
            y = _mm_mul_ps(y, _mm_mul_ps(z, z));
            tmp = _mm_mul_ps(z, _ps_0p5);
            y = _mm_sub_ps(y, tmp);
            y = _mm_add_ps(y, _ps_1);

            /* Evaluate the second polynom  (Pi/4 <= x <= 0) */
            y2 = _ps_sincof_p0;
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, _ps_sincof_p1);
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, _ps_sincof_p2);
            y2 = _mm_mul_ps(y2, _mm_mul_ps(z, x));
            y2 = _mm_add_ps(y2, x);

            /* select the correct result from the two polynoms */
            xmm3 = poly_mask;
            ysin2 = _mm_and_ps(xmm3, y2);
            ysin1 = _mm_andnot_ps(xmm3, y);
            y2 = _mm_sub_ps(y2,ysin2);
            y = _mm_sub_ps(y, ysin1);

            xmm1 = _mm_add_ps(ysin1,ysin2);
            xmm2 = _mm_add_ps(y,y2);

            /* update the sign */
            s = _mm_xor_ps(xmm1, sign_bit_sin);
            c = _mm_xor_ps(xmm2, sign_bit_cos);

            //GNSS-SDR needs to return -sin
            s = _mm_xor_ps(s, _ps_sign_mask);

            _mm_storeu_ps ((float*)sin_value, s);
            _mm_storeu_ps ((float*)cos_value, c);

            for(unsigned int e = 0; e < 4; e++)
                {
                    d_carr_sign[e] = lv_cmake(cos_value[e], sin_value[e]);
                }
            d_carr_sign += 4;

            phase_rad_array = _mm_add_ps (phase_rad_array, phase_step_rad_array);
        }

    if (num_points%4!=0)
        {
            __VOLK_ATTR_ALIGNED(16) float phase_rad_store[4];
            _mm_storeu_ps ((float*)phase_rad_store, phase_rad_array);

            float phase_rad = phase_rad_store[0];

            for(unsigned int i = 0; i < num_points%4; i++)
                {
                    *d_carr_sign = lv_cmake(cos(phase_rad), -sin(phase_rad));
                    d_carr_sign++;
                    phase_rad += phase_step_rad;
                }
        }
}
#endif /* LV_HAVE_SSE2 */

#ifdef LV_HAVE_GENERIC
/*!
  \brief Accumulates the values in the input buffer
  \param result The accumulated result
  \param inputBuffer The buffer of data to be accumulated
  \param num_points The number of values in inputBuffer to be accumulated
 */
static inline void volk_gnsssdr_s32f_x2_update_local_carrier_32fc_generic(lv_32fc_t* d_carr_sign, const float phase_rad_init, const float phase_step_rad, unsigned int num_points)
{
    //    float* pointer1 = (float*)&phase_rad_init;
    //    *pointer1 = 0;
    //    float* pointer2 = (float*)&phase_step_rad;
    //    *pointer2 = 0.5;

    float phase_rad = phase_rad_init;
    for(unsigned int i = 0; i < num_points; i++)
        {
            *d_carr_sign = lv_cmake(cos(phase_rad), -sin(phase_rad));
            d_carr_sign++;
            phase_rad += phase_step_rad;
        }
}
#endif /* LV_HAVE_GENERIC */
#endif /* INCLUDED_volk_gnsssdr_32fc_s32f_x2_update_local_carrier_32fc_u_H */


#ifndef INCLUDED_volk_gnsssdr_32fc_s32f_x2_update_local_carrier_32fc_a_H
#define INCLUDED_volk_gnsssdr_32fc_s32f_x2_update_local_carrier_32fc_a_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <inttypes.h>
#include <stdio.h>

#ifdef LV_HAVE_AVX
#include <immintrin.h>
/*!
 \brief Accumulates the values in the input buffer
 \param result The accumulated result
 \param inputBuffer The buffer of data to be accumulated
 \param num_points The number of values in inputBuffer to be accumulated
 */
static inline void volk_gnsssdr_s32f_x2_update_local_carrier_32fc_a_avx(lv_32fc_t* d_carr_sign, const float phase_rad_init, const float phase_step_rad, unsigned int num_points)
{
    //    float* pointer1 = (float*)&phase_rad_init;
    //    *pointer1 = 0;
    //    float* pointer2 = (float*)&phase_step_rad;
    //    *pointer2 = 0.5;

    const unsigned int sse_iters = num_points / 8;

    __m256 _ps256_minus_cephes_DP1 = _mm256_set1_ps(-0.78515625f);
    __m256 _ps256_minus_cephes_DP2 = _mm256_set1_ps(-2.4187564849853515625e-4f);
    __m256 _ps256_minus_cephes_DP3 = _mm256_set1_ps(-3.77489497744594108e-8f);
    __m256 _ps256_sign_mask = _mm256_set1_ps(-0.f);
    __m128i _pi32avx_1 = _mm_set1_epi32(1);
    __m128i _pi32avx_inv1 = _mm_set1_epi32(~1);
    __m128i _pi32avx_2 = _mm_set1_epi32(2);
    __m128i _pi32avx_4 = _mm_set1_epi32(4);
    __m256 _ps256_cephes_FOPI = _mm256_set1_ps(1.27323954473516f); // 4 / PI
    __m256 _ps256_sincof_p0 = _mm256_set1_ps(-1.9515295891E-4f);
    __m256 _ps256_sincof_p1 = _mm256_set1_ps( 8.3321608736E-3f);
    __m256 _ps256_sincof_p2 = _mm256_set1_ps(-1.6666654611E-1f);
    __m256 _ps256_coscof_p0 = _mm256_set1_ps( 2.443315711809948E-005f);
    __m256 _ps256_coscof_p1 = _mm256_set1_ps(-1.388731625493765E-003f);
    __m256 _ps256_coscof_p2 = _mm256_set1_ps( 4.166664568298827E-002f);
    __m256 _ps256_1 = _mm256_set1_ps(1.f);
    __m256 _ps256_0p5 = _mm256_set1_ps(0.5f);

    __m256 phase_step_rad_array = _mm256_set1_ps(8*phase_step_rad);

    __m256 phase_rad_array, x, s, c, swap_sign_bit_sin, sign_bit_cos, poly_mask, z, tmp, y, y2, ysin1, ysin2;
    __m256 xmm1, xmm2, xmm3, sign_bit_sin;
    __m256i imm0, imm2, imm4, tmp256i;
    __m128i imm0_1, imm0_2, imm2_1, imm2_2, imm4_1, imm4_2;
    __VOLK_ATTR_ALIGNED(32) float sin_value[8];
    __VOLK_ATTR_ALIGNED(32) float cos_value[8];

    phase_rad_array = _mm256_set_ps (phase_rad_init + 7*phase_step_rad, phase_rad_init + 6*phase_step_rad, phase_rad_init + 5*phase_step_rad, phase_rad_init + 4*phase_step_rad, phase_rad_init + 3*phase_step_rad, phase_rad_init + 2*phase_step_rad, phase_rad_init + phase_step_rad, phase_rad_init);

    for(int i = 0; i < sse_iters; i++)
        {

            x = phase_rad_array;

            /* extract the sign bit (upper one) */
            sign_bit_sin = _mm256_and_ps(x, _ps256_sign_mask);

            /* take the absolute value */
            x = _mm256_xor_ps(x, sign_bit_sin);

            /* scale by 4/Pi */
            y = _mm256_mul_ps(x, _ps256_cephes_FOPI);

            /* we use SSE2 routines to perform the integer ops */

            //COPY_IMM_TO_XMM(_mm256_cvttps_epi32(y),imm2_1,imm2_2);
            tmp256i = _mm256_cvttps_epi32(y);
            imm2_1 = _mm256_extractf128_si256 (tmp256i, 0);
            imm2_2 = _mm256_extractf128_si256 (tmp256i, 1);

            imm2_1 = _mm_add_epi32(imm2_1, _pi32avx_1);
            imm2_2 = _mm_add_epi32(imm2_2, _pi32avx_1);

            imm2_1 = _mm_and_si128(imm2_1, _pi32avx_inv1);
            imm2_2 = _mm_and_si128(imm2_2, _pi32avx_inv1);

            //COPY_XMM_TO_IMM(imm2_1,imm2_2,imm2);
            //_mm256_set_m128i not defined in some versions of immintrin.h
            //imm2 = _mm256_set_m128i (imm2_2, imm2_1);
            imm2 = _mm256_insertf128_si256(_mm256_castsi128_si256(imm2_1),(imm2_2),1);

            y = _mm256_cvtepi32_ps(imm2);

            imm4_1 = imm2_1;
            imm4_2 = imm2_2;

            imm0_1 = _mm_and_si128(imm2_1, _pi32avx_4);
            imm0_2 = _mm_and_si128(imm2_2, _pi32avx_4);

            imm0_1 = _mm_slli_epi32(imm0_1, 29);
            imm0_2 = _mm_slli_epi32(imm0_2, 29);

            //COPY_XMM_TO_IMM(imm0_1, imm0_2, imm0);
            //_mm256_set_m128i not defined in some versions of immintrin.h
            //imm0 = _mm256_set_m128i (imm0_2, imm0_1);
            imm0 = _mm256_insertf128_si256(_mm256_castsi128_si256(imm0_1),(imm0_2),1);

            imm2_1 = _mm_and_si128(imm2_1, _pi32avx_2);
            imm2_2 = _mm_and_si128(imm2_2, _pi32avx_2);

            imm2_1 = _mm_cmpeq_epi32(imm2_1, _mm_setzero_si128());
            imm2_2 = _mm_cmpeq_epi32(imm2_2, _mm_setzero_si128());

            //COPY_XMM_TO_IMM(imm2_1, imm2_2, imm2);
            //_mm256_set_m128i not defined in some versions of immintrin.h
            //imm2 = _mm256_set_m128i (imm2_2, imm2_1);
            imm2 = _mm256_insertf128_si256(_mm256_castsi128_si256(imm2_1),(imm2_2),1);

            swap_sign_bit_sin = _mm256_castsi256_ps(imm0);
            poly_mask = _mm256_castsi256_ps(imm2);

            /* The magic pass: "Extended precision modular arithmetic"
         x = ((x - y * DP1) - y * DP2) - y * DP3; */
            xmm1 = _ps256_minus_cephes_DP1;
            xmm2 = _ps256_minus_cephes_DP2;
            xmm3 = _ps256_minus_cephes_DP3;
            xmm1 = _mm256_mul_ps(y, xmm1);
            xmm2 = _mm256_mul_ps(y, xmm2);
            xmm3 = _mm256_mul_ps(y, xmm3);
            x = _mm256_add_ps(x, xmm1);
            x = _mm256_add_ps(x, xmm2);
            x = _mm256_add_ps(x, xmm3);

            imm4_1 = _mm_sub_epi32(imm4_1, _pi32avx_2);
            imm4_2 = _mm_sub_epi32(imm4_2, _pi32avx_2);

            imm4_1 = _mm_andnot_si128(imm4_1, _pi32avx_4);
            imm4_2 = _mm_andnot_si128(imm4_2, _pi32avx_4);

            imm4_1 = _mm_slli_epi32(imm4_1, 29);
            imm4_2 = _mm_slli_epi32(imm4_2, 29);

            //COPY_XMM_TO_IMM(imm4_1, imm4_2, imm4);
            //_mm256_set_m128i not defined in some versions of immintrin.h
            //imm4 = _mm256_set_m128i (imm4_2, imm4_1);
            imm4 = _mm256_insertf128_si256(_mm256_castsi128_si256(imm4_1),(imm4_2),1);

            sign_bit_cos = _mm256_castsi256_ps(imm4);

            sign_bit_sin = _mm256_xor_ps(sign_bit_sin, swap_sign_bit_sin);

            /* Evaluate the first polynom  (0 <= x <= Pi/4) */
            z = _mm256_mul_ps(x,x);
            y = _ps256_coscof_p0;

            y = _mm256_mul_ps(y, z);
            y = _mm256_add_ps(y, _ps256_coscof_p1);
            y = _mm256_mul_ps(y, z);
            y = _mm256_add_ps(y, _ps256_coscof_p2);
            y = _mm256_mul_ps(y, z);
            y = _mm256_mul_ps(y, z);
            tmp = _mm256_mul_ps(z, _ps256_0p5);
            y = _mm256_sub_ps(y, tmp);
            y = _mm256_add_ps(y, _ps256_1);

            /* Evaluate the second polynom  (Pi/4 <= x <= 0) */

            y2 = _ps256_sincof_p0;
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_add_ps(y2, _ps256_sincof_p1);
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_add_ps(y2, _ps256_sincof_p2);
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_mul_ps(y2, x);
            y2 = _mm256_add_ps(y2, x);

            /* select the correct result from the two polynoms */
            xmm3 = poly_mask;
            ysin2 = _mm256_and_ps(xmm3, y2);
            ysin1 = _mm256_andnot_ps(xmm3, y);
            y2 = _mm256_sub_ps(y2,ysin2);
            y = _mm256_sub_ps(y, ysin1);

            xmm1 = _mm256_add_ps(ysin1,ysin2);
            xmm2 = _mm256_add_ps(y,y2);

            /* update the sign */
            s = _mm256_xor_ps(xmm1, sign_bit_sin);
            c = _mm256_xor_ps(xmm2, sign_bit_cos);

            //GNSS-SDR needs to return -sin
            s = _mm256_xor_ps(s, _ps256_sign_mask);

            _mm256_store_ps ((float*)sin_value, s);
            _mm256_store_ps ((float*)cos_value, c);

            for(int i = 0; i < 8; i++)
                {
                    d_carr_sign[i] = lv_cmake(cos_value[i], sin_value[i]);
                }
            d_carr_sign += 8;

            phase_rad_array = _mm256_add_ps (phase_rad_array, phase_step_rad_array);
        }

    if (num_points%8!=0)
        {
            __VOLK_ATTR_ALIGNED(32) float phase_rad_store[8];
            _mm256_store_ps ((float*)phase_rad_store, phase_rad_array);

            float phase_rad = phase_rad_store[0];

            for(int i = 0; i < num_points%8; i++)
                {
                    *d_carr_sign = lv_cmake(cos(phase_rad), -sin(phase_rad));
                    d_carr_sign++;
                    phase_rad += phase_step_rad;
                }
        }
}
#endif /* LV_HAVE_AVX */

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/*!
 \brief Accumulates the values in the input buffer
 \param result The accumulated result
 \param inputBuffer The buffer of data to be accumulated
 \param num_points The number of values in inputBuffer to be accumulated
 */
static inline void volk_gnsssdr_s32f_x2_update_local_carrier_32fc_a_sse2(lv_32fc_t* d_carr_sign, const float phase_rad_init, const float phase_step_rad, unsigned int num_points)
{
    //    float* pointer1 = (float*)&phase_rad_init;
    //    *pointer1 = 0;
    //    float* pointer2 = (float*)&phase_step_rad;
    //    *pointer2 = 0.5;

    const unsigned int sse_iters = num_points / 4;

    __m128 _ps_minus_cephes_DP1 = _mm_set1_ps(-0.78515625f);
    __m128 _ps_minus_cephes_DP2 = _mm_set1_ps(-2.4187564849853515625e-4f);
    __m128 _ps_minus_cephes_DP3 = _mm_set1_ps(-3.77489497744594108e-8f);
    __m128 _ps_sign_mask = _mm_set1_ps(-0.f);
    __m128i _pi32_1 = _mm_set1_epi32(1);
    __m128i _pi32_inv1 = _mm_set1_epi32(~1);
    __m128i _pi32_2 = _mm_set1_epi32(2);
    __m128i _pi32_4 = _mm_set1_epi32(4);
    __m128 _ps_cephes_FOPI = _mm_set1_ps(1.27323954473516f); // 4 / PI
    __m128 _ps_sincof_p0 = _mm_set1_ps(-1.9515295891E-4f);
    __m128 _ps_sincof_p1 = _mm_set1_ps( 8.3321608736E-3f);
    __m128 _ps_sincof_p2 = _mm_set1_ps(-1.6666654611E-1f);
    __m128 _ps_coscof_p0 = _mm_set1_ps( 2.443315711809948E-005f);
    __m128 _ps_coscof_p1 = _mm_set1_ps(-1.388731625493765E-003f);
    __m128 _ps_coscof_p2 = _mm_set1_ps( 4.166664568298827E-002f);
    __m128 _ps_1 = _mm_set1_ps(1.f);
    __m128 _ps_0p5 = _mm_set1_ps(0.5f);

    __m128 phase_step_rad_array = _mm_set1_ps(4*phase_step_rad);

    __m128 phase_rad_array, x, s, c, swap_sign_bit_sin, sign_bit_cos, poly_mask, z, tmp, y, y2, ysin1, ysin2;
    __m128 xmm1, xmm2, xmm3, sign_bit_sin;
    __m128i emm0, emm2, emm4;
    __VOLK_ATTR_ALIGNED(16) float sin_value[4];
    __VOLK_ATTR_ALIGNED(16) float cos_value[4];

    phase_rad_array = _mm_set_ps (phase_rad_init+3*phase_step_rad, phase_rad_init+2*phase_step_rad, phase_rad_init+phase_step_rad, phase_rad_init);

    for(unsigned int i = 0; i < sse_iters; i++)
        {
            x = phase_rad_array;

            /* extract the sign bit (upper one) */
            sign_bit_sin = _mm_and_ps(x, _ps_sign_mask);

            /* take the absolute value */
            x = _mm_xor_ps(x, sign_bit_sin);

            /* scale by 4/Pi */
            y = _mm_mul_ps(x, _ps_cephes_FOPI);

            /* store the integer part of y in emm2 */
            emm2 = _mm_cvttps_epi32(y);

            /* j=(j+1) & (~1) (see the cephes sources) */
            emm2 = _mm_add_epi32(emm2, _pi32_1);
            emm2 = _mm_and_si128(emm2, _pi32_inv1);
            y = _mm_cvtepi32_ps(emm2);

            emm4 = emm2;

            /* get the swap sign flag for the sine */
            emm0 = _mm_and_si128(emm2, _pi32_4);
            emm0 = _mm_slli_epi32(emm0, 29);
            swap_sign_bit_sin = _mm_castsi128_ps(emm0);

            /* get the polynom selection mask for the sine*/
            emm2 = _mm_and_si128(emm2, _pi32_2);
            emm2 = _mm_cmpeq_epi32(emm2, _mm_setzero_si128());
            poly_mask = _mm_castsi128_ps(emm2);

            /* The magic pass: "Extended precision modular arithmetic"
         x = ((x - y * DP1) - y * DP2) - y * DP3; */
            xmm1 = _mm_mul_ps(y, _ps_minus_cephes_DP1);
            xmm2 = _mm_mul_ps(y, _ps_minus_cephes_DP2);
            xmm3 = _mm_mul_ps(y, _ps_minus_cephes_DP3);
            x = _mm_add_ps(_mm_add_ps(x, xmm1), _mm_add_ps(xmm2, xmm3));

            emm4 = _mm_sub_epi32(emm4, _pi32_2);
            emm4 = _mm_andnot_si128(emm4, _pi32_4);
            emm4 = _mm_slli_epi32(emm4, 29);
            sign_bit_cos = _mm_castsi128_ps(emm4);

            sign_bit_sin = _mm_xor_ps(sign_bit_sin, swap_sign_bit_sin);

            /* Evaluate the first polynom  (0 <= x <= Pi/4) */
            z = _mm_mul_ps(x,x);
            y = _ps_coscof_p0;
            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, _ps_coscof_p1);
            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, _ps_coscof_p2);
            y = _mm_mul_ps(y, _mm_mul_ps(z, z));
            tmp = _mm_mul_ps(z, _ps_0p5);
            y = _mm_sub_ps(y, tmp);
            y = _mm_add_ps(y, _ps_1);

            /* Evaluate the second polynom  (Pi/4 <= x <= 0) */
            y2 = _ps_sincof_p0;
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, _ps_sincof_p1);
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, _ps_sincof_p2);
            y2 = _mm_mul_ps(y2, _mm_mul_ps(z, x));
            y2 = _mm_add_ps(y2, x);

            /* select the correct result from the two polynoms */
            xmm3 = poly_mask;
            ysin2 = _mm_and_ps(xmm3, y2);
            ysin1 = _mm_andnot_ps(xmm3, y);
            y2 = _mm_sub_ps(y2,ysin2);
            y = _mm_sub_ps(y, ysin1);

            xmm1 = _mm_add_ps(ysin1,ysin2);
            xmm2 = _mm_add_ps(y,y2);

            /* update the sign */
            s = _mm_xor_ps(xmm1, sign_bit_sin);
            c = _mm_xor_ps(xmm2, sign_bit_cos);

            //GNSS-SDR needs to return -sin
            s = _mm_xor_ps(s, _ps_sign_mask);

            _mm_store_ps ((float*)sin_value, s);
            _mm_store_ps ((float*)cos_value, c);

            for(unsigned int e = 0; e < 4; e++)
                {
                    d_carr_sign[e] = lv_cmake(cos_value[e], sin_value[e]);
                }
            d_carr_sign += 4;

            phase_rad_array = _mm_add_ps (phase_rad_array, phase_step_rad_array);
        }

    if (num_points % 4 != 0)
        {
            __VOLK_ATTR_ALIGNED(16) float phase_rad_store[4];
            _mm_store_ps ((float*)phase_rad_store, phase_rad_array);

            float phase_rad = phase_rad_store[0];

            for(unsigned int i = 0; i < num_points%4; i++)
                {
                    *d_carr_sign = lv_cmake(cos(phase_rad), -sin(phase_rad));
                    d_carr_sign++;
                    phase_rad += phase_step_rad;
                }
        }
}
#endif /* LV_HAVE_SSE2 */

#ifdef LV_HAVE_GENERIC
/*!
 \brief Accumulates the values in the input buffer
 \param result The accumulated result
 \param inputBuffer The buffer of data to be accumulated
 \param num_points The number of values in inputBuffer to be accumulated
 */
static inline void volk_gnsssdr_s32f_x2_update_local_carrier_32fc_a_generic(lv_32fc_t* d_carr_sign, const float phase_rad_init, const float phase_step_rad, unsigned int num_points)
{
    //    float* pointer1 = (float*)&phase_rad_init;
    //    *pointer1 = 0;
    //    float* pointer2 = (float*)&phase_step_rad;
    //    *pointer2 = 0.5;

    float phase_rad = phase_rad_init;
    for(unsigned int i = 0; i < num_points; i++)
        {
            *d_carr_sign = lv_cmake(cos(phase_rad), -sin(phase_rad));
            d_carr_sign++;
            phase_rad += phase_step_rad;
        }
}
#endif /* LV_HAVE_GENERIC */
#endif /* INCLUDED_volk_gnsssdr_32fc_s32f_x2_update_local_carrier_32fc_a_H */

