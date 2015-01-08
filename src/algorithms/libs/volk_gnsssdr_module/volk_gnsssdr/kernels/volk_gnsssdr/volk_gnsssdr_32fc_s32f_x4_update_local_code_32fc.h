/*!
 * \file volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc
 * \brief Volk protokernel: replaces the tracking function for update_local_code
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * Volk protokernel that replaces the tracking function for update_local_code
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

#ifndef INCLUDED_volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc_u_H
#define INCLUDED_volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc_u_H

#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include "volk_gnsssdr/volk_gnsssdr_complex.h"

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
/*!
    \brief Takes the conjugate of a complex vector.
    \param cVector The vector where the results will be stored
    \param aVector Vector to be conjugated
    \param num_points The number of complex values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc_u_sse4_1(lv_32fc_t* d_very_early_code, const float d_very_early_late_spc_chips, const float code_length_half_chips, const float code_phase_step_half_chips, const float tcode_half_chips_input, const lv_32fc_t* d_ca_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;

    __m128 tquot, fmod_num, fmod_result, associated_chip_index_array;

    __m128 tcode_half_chips_array = _mm_set_ps (tcode_half_chips_input+3*code_phase_step_half_chips, tcode_half_chips_input+2*code_phase_step_half_chips, tcode_half_chips_input+code_phase_step_half_chips, tcode_half_chips_input);
    __m128 code_phase_step_half_chips_array = _mm_set1_ps (code_phase_step_half_chips*4);
    __m128 d_very_early_late_spc_chips_Multiplied_by_2 =  _mm_set1_ps (2*d_very_early_late_spc_chips);
    __m128 code_length_half_chips_array =  _mm_set1_ps (code_length_half_chips);
    __m128 twos =  _mm_set1_ps (2);
    __m128i associated_chip_index_array_int;

    __VOLK_ATTR_ALIGNED(16) int32_t output[4];

    for (unsigned int i = 0; i < sse_iters; i++)
        {
            //fmod = numer - tquot * denom; tquot = numer/denom truncated
            //associated_chip_index = 2 + round(fmod(tcode_half_chips - 2*d_very_early_late_spc_chips, code_length_half_chips));
            fmod_num = _mm_sub_ps (tcode_half_chips_array, d_very_early_late_spc_chips_Multiplied_by_2);
            tquot = _mm_div_ps (fmod_num, code_length_half_chips_array);
            tquot = _mm_round_ps (tquot, (_MM_FROUND_TO_ZERO |_MM_FROUND_NO_EXC) );
            fmod_result = _mm_sub_ps (fmod_num, _mm_mul_ps (tquot, code_length_half_chips_array));

            associated_chip_index_array = _mm_round_ps (fmod_result, (_MM_FROUND_TO_NEAREST_INT |_MM_FROUND_NO_EXC));
            associated_chip_index_array = _mm_add_ps(twos, associated_chip_index_array);
            associated_chip_index_array_int = _mm_cvtps_epi32 (associated_chip_index_array);
            _mm_storeu_si128 ((__m128i*)output, associated_chip_index_array_int);

            //d_very_early_code[i] = d_ca_code[associated_chip_index];
            *d_very_early_code++ = d_ca_code[output[0]];
            *d_very_early_code++ = d_ca_code[output[1]];
            *d_very_early_code++ = d_ca_code[output[2]];
            *d_very_early_code++ = d_ca_code[output[3]];

            //tcode_half_chips = tcode_half_chips + code_phase_step_half_chips;
            tcode_half_chips_array = _mm_add_ps (tcode_half_chips_array, code_phase_step_half_chips_array);
        }

    if (num_points % 4 != 0)
        {
            __VOLK_ATTR_ALIGNED(16) float tcode_half_chips_stored[4];
            _mm_storeu_ps ((float*)tcode_half_chips_stored, tcode_half_chips_array);

            int associated_chip_index;
            float tcode_half_chips = tcode_half_chips_stored[0];
            float d_very_early_late_spc_chips_multiplied_by_2 = 2*d_very_early_late_spc_chips;

            for (unsigned int i = 0; i < num_points%4; i++)
                {
                    associated_chip_index = 2 + round(fmod(tcode_half_chips - d_very_early_late_spc_chips_multiplied_by_2, code_length_half_chips));
                    d_very_early_code[i] = d_ca_code[associated_chip_index];
                    tcode_half_chips = tcode_half_chips + code_phase_step_half_chips;
                }
        }
}
#endif /* LV_HAVE_SSE4_1 */

#ifdef LV_HAVE_GENERIC
/*!
    \brief Takes the conjugate of a complex vector.
    \param cVector The vector where the results will be stored
    \param aVector Vector to be conjugated
    \param num_points The number of complex values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc_generic(lv_32fc_t* d_very_early_code, const float d_very_early_late_spc_chips, const float code_length_half_chips, const float code_phase_step_half_chips, const float tcode_half_chips_input, const lv_32fc_t* d_ca_code, unsigned int num_points)
{
    int associated_chip_index;
    float tcode_half_chips = tcode_half_chips_input;
    float d_very_early_late_spc_chips_multiplied_by_2 = 2*d_very_early_late_spc_chips;

    for (unsigned int i = 0; i < num_points; i++)
        {
            associated_chip_index = 2 + round(fmod(tcode_half_chips - d_very_early_late_spc_chips_multiplied_by_2, code_length_half_chips));
            d_very_early_code[i] = d_ca_code[associated_chip_index];
            tcode_half_chips = tcode_half_chips + code_phase_step_half_chips;
        }
}
#endif /* LV_HAVE_GENERIC */


#endif /* INCLUDED_volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc_u_H */
#ifndef INCLUDED_volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc_a_H
#define INCLUDED_volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc_a_H

#include <inttypes.h>
#include <stdio.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <float.h>

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
/*!
    \brief Takes the conjugate of a complex vector.
    \param cVector The vector where the results will be stored
    \param aVector Vector to be conjugated
    \param num_points The number of complex values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc_a_sse4_1(lv_32fc_t* d_very_early_code, const float d_very_early_late_spc_chips, const float code_length_half_chips, const float code_phase_step_half_chips, const float tcode_half_chips_input, const lv_32fc_t* d_ca_code, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;

    __m128 tquot, fmod_num, fmod_result, associated_chip_index_array;

    __m128 tcode_half_chips_array = _mm_set_ps (tcode_half_chips_input+3*code_phase_step_half_chips, tcode_half_chips_input+2*code_phase_step_half_chips, tcode_half_chips_input+code_phase_step_half_chips, tcode_half_chips_input);
    __m128 code_phase_step_half_chips_array = _mm_set1_ps (code_phase_step_half_chips*4);
    __m128 d_very_early_late_spc_chips_Multiplied_by_2 =  _mm_set1_ps (2*d_very_early_late_spc_chips);
    __m128 code_length_half_chips_array =  _mm_set1_ps (code_length_half_chips);
    __m128 twos =  _mm_set1_ps (2);
    __m128i associated_chip_index_array_int;

    __VOLK_ATTR_ALIGNED(16) int32_t output[4];

    for (unsigned int i = 0; i < sse_iters; i++)
        {
            //fmod = numer - tquot * denom; tquot = numer/denom truncated
            //associated_chip_index = 2 + round(fmod(tcode_half_chips - 2*d_very_early_late_spc_chips, code_length_half_chips));
            fmod_num = _mm_sub_ps (tcode_half_chips_array, d_very_early_late_spc_chips_Multiplied_by_2);
            tquot = _mm_div_ps (fmod_num, code_length_half_chips_array);
            tquot = _mm_round_ps (tquot, (_MM_FROUND_TO_ZERO |_MM_FROUND_NO_EXC) );
            fmod_result = _mm_sub_ps (fmod_num, _mm_mul_ps (tquot, code_length_half_chips_array));

            associated_chip_index_array = _mm_round_ps (fmod_result, (_MM_FROUND_TO_NEAREST_INT |_MM_FROUND_NO_EXC));
            associated_chip_index_array = _mm_add_ps(twos, associated_chip_index_array);
            associated_chip_index_array_int = _mm_cvtps_epi32 (associated_chip_index_array);
            _mm_store_si128 ((__m128i*)output, associated_chip_index_array_int);

            //d_very_early_code[i] = d_ca_code[associated_chip_index];
            *d_very_early_code++ = d_ca_code[output[0]];
            *d_very_early_code++ = d_ca_code[output[1]];
            *d_very_early_code++ = d_ca_code[output[2]];
            *d_very_early_code++ = d_ca_code[output[3]];

            //tcode_half_chips = tcode_half_chips + code_phase_step_half_chips;
            tcode_half_chips_array = _mm_add_ps (tcode_half_chips_array, code_phase_step_half_chips_array);
        }

    if (num_points % 4 != 0)
        {
            __VOLK_ATTR_ALIGNED(16) float tcode_half_chips_stored[4];
            _mm_storeu_ps ((float*)tcode_half_chips_stored, tcode_half_chips_array);

            int associated_chip_index;
            float tcode_half_chips = tcode_half_chips_stored[0];
            float d_very_early_late_spc_chips_multiplied_by_2 = 2*d_very_early_late_spc_chips;

            for (unsigned int i = 0; i < num_points%4; i++)
                {
                    associated_chip_index = 2 + round(fmod(tcode_half_chips - d_very_early_late_spc_chips_multiplied_by_2, code_length_half_chips));
                    d_very_early_code[i] = d_ca_code[associated_chip_index];
                    tcode_half_chips = tcode_half_chips + code_phase_step_half_chips;
                }
        }

}
#endif /* LV_HAVE_SSE4_1 */

#ifdef LV_HAVE_GENERIC
/*!
    \brief Takes the conjugate of a complex vector.
    \param cVector The vector where the results will be stored
    \param aVector Vector to be conjugated
    \param num_points The number of complex values in aVector to be conjugated and stored into cVector
 */
static inline void volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc_a_generic(lv_32fc_t* d_very_early_code, const float d_very_early_late_spc_chips, const float code_length_half_chips, const float code_phase_step_half_chips, const float tcode_half_chips_input, const lv_32fc_t* d_ca_code, unsigned int num_points)
{
    int associated_chip_index;
    float tcode_half_chips = tcode_half_chips_input;
    float d_very_early_late_spc_chips_multiplied_by_2 = 2*d_very_early_late_spc_chips;

    for (unsigned int i = 0; i < num_points; i++)
        {
            associated_chip_index = 2 + round(fmod(tcode_half_chips - d_very_early_late_spc_chips_multiplied_by_2, code_length_half_chips));
            d_very_early_code[i] = d_ca_code[associated_chip_index];
            tcode_half_chips = tcode_half_chips + code_phase_step_half_chips;
        }
}
#endif /* LV_HAVE_GENERIC */

#endif /* INCLUDED_volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc_a_H */
