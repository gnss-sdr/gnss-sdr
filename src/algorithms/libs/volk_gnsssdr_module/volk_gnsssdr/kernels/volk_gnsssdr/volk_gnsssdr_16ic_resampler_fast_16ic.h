/*!
 * \file volk_gnsssdr_16ic_resampler_fast_16ic.h
 * \brief VOLK_GNSSSDR kernel: resamples a 16 bits complex vector.
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that multiplies two 16 bits vectors (8 bits the real part
 * and 8 bits the imaginary part) and accumulates them
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
 * \page volk_gnsssdr_16ic_resampler_fast_16ic
 *
 * \b Overview
 *
 * Resamples a complex vector (16-bit integer each component).
 * WARNING: \p phase cannot reach more that twice the length of \p local_code, either positive or negative.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_16ic_resampler_fast_16ic(lv_16sc_t* result, const lv_16sc_t* local_code, float rem_code_phase_chips, float code_phase_step_chips, int code_length_chips, unsigned int num_output_samples)
 * \endcode
 *
 * \b Inputs
 * \li local_code:            Vector to be resampled.
 * \li rem_code_phase_chips:  Remnant code phase [chips]
 * \li code_phase_step_chips: Phase increment per sample [chips/sample]
 * \li code_length_chips:     Code length in chips.
 * \li num_points:            The number of data values to be in the resampled vector.
 *
 * \b Outputs
 * \li result:                Pointer to the resampled vector.
 *
 */

#ifndef INCLUDED_volk_gnsssdr_16ic_resampler_fast_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_resampler_fast_16ic_H

#include <math.h>
#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_resampler_fast_16ic_generic(lv_16sc_t* result, const lv_16sc_t* local_code, float rem_code_phase_chips, float code_phase_step_chips, int code_length_chips, unsigned int num_output_samples)
{
    int local_code_chip_index;
    unsigned int n;
    //fesetround(FE_TONEAREST);
    for (n = 0; n < num_output_samples; n++)
        {
            // resample code for current tap
            local_code_chip_index = round(code_phase_step_chips * (float)n + rem_code_phase_chips - 0.5f);
            if (local_code_chip_index < 0.0) local_code_chip_index += code_length_chips;
            if (local_code_chip_index > (code_length_chips - 1)) local_code_chip_index -= code_length_chips;
            result[n] = local_code[local_code_chip_index];
        }
}

#endif /*LV_HAVE_GENERIC*/


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_resampler_fast_16ic_a_sse2(lv_16sc_t* result, const lv_16sc_t* local_code, float rem_code_phase_chips, float code_phase_step_chips, int code_length_chips, unsigned int num_output_samples)  //, int* scratch_buffer, float* scratch_buffer_float)
{
    _MM_SET_ROUNDING_MODE(_MM_ROUND_NEAREST);  //_MM_ROUND_NEAREST, _MM_ROUND_DOWN, _MM_ROUND_UP, _MM_ROUND_TOWARD_ZERO
    unsigned int number;
    const unsigned int quarterPoints = num_output_samples / 4;

    lv_16sc_t* _result = result;

    __VOLK_ATTR_ALIGNED(16)
    int local_code_chip_index[4];
    __m128 _rem_code_phase, _code_phase_step_chips;
    __m128i _code_length_chips, _code_length_chips_minus1;
    __m128 _code_phase_out, _code_phase_out_with_offset;
    rem_code_phase_chips = rem_code_phase_chips - 0.5f;

    _rem_code_phase = _mm_load1_ps(&rem_code_phase_chips);          //load float to all four float values in m128 register
    _code_phase_step_chips = _mm_load1_ps(&code_phase_step_chips);  //load float to all four float values in m128 register
    __VOLK_ATTR_ALIGNED(16)
    int four_times_code_length_chips_minus1[4];
    four_times_code_length_chips_minus1[0] = code_length_chips - 1;
    four_times_code_length_chips_minus1[1] = code_length_chips - 1;
    four_times_code_length_chips_minus1[2] = code_length_chips - 1;
    four_times_code_length_chips_minus1[3] = code_length_chips - 1;

    __VOLK_ATTR_ALIGNED(16)
    int four_times_code_length_chips[4];
    four_times_code_length_chips[0] = code_length_chips;
    four_times_code_length_chips[1] = code_length_chips;
    four_times_code_length_chips[2] = code_length_chips;
    four_times_code_length_chips[3] = code_length_chips;

    _code_length_chips = _mm_load_si128((__m128i*)&four_times_code_length_chips);                //load float to all four float values in m128 register
    _code_length_chips_minus1 = _mm_load_si128((__m128i*)&four_times_code_length_chips_minus1);  //load float to all four float values in m128 register

    __m128i negative_indexes, overflow_indexes, _code_phase_out_int, _code_phase_out_int_neg, _code_phase_out_int_over;

    __m128i zero = _mm_setzero_si128();

    __VOLK_ATTR_ALIGNED(16)
    float init_idx_float[4] = {0.0f, 1.0f, 2.0f, 3.0f};
    __m128 _4output_index = _mm_load_ps(init_idx_float);
    __VOLK_ATTR_ALIGNED(16)
    float init_4constant_float[4] = {4.0f, 4.0f, 4.0f, 4.0f};
    __m128 _4constant_float = _mm_load_ps(init_4constant_float);

    for (number = 0; number < quarterPoints; number++)
        {
            _code_phase_out = _mm_mul_ps(_code_phase_step_chips, _4output_index);        //compute the code phase point with the phase step
            _code_phase_out_with_offset = _mm_add_ps(_code_phase_out, _rem_code_phase);  //add the phase offset
            _code_phase_out_int = _mm_cvtps_epi32(_code_phase_out_with_offset);          //convert to integer

            negative_indexes = _mm_cmplt_epi32(_code_phase_out_int, zero);                     //test for negative values
            _code_phase_out_int_neg = _mm_add_epi32(_code_phase_out_int, _code_length_chips);  //the negative values branch
            _code_phase_out_int_neg = _mm_xor_si128(_code_phase_out_int, _mm_and_si128(negative_indexes, _mm_xor_si128(_code_phase_out_int_neg, _code_phase_out_int)));

            overflow_indexes = _mm_cmpgt_epi32(_code_phase_out_int_neg, _code_length_chips_minus1);  //test for overflow values
            _code_phase_out_int_over = _mm_sub_epi32(_code_phase_out_int_neg, _code_length_chips);   //the negative values branch
            _code_phase_out_int_over = _mm_xor_si128(_code_phase_out_int_neg, _mm_and_si128(overflow_indexes, _mm_xor_si128(_code_phase_out_int_over, _code_phase_out_int_neg)));

            _mm_store_si128((__m128i*)local_code_chip_index, _code_phase_out_int_over);  // Store the results back

            //todo: optimize the local code lookup table with intrinsics, if possible
            *_result++ = local_code[local_code_chip_index[0]];
            *_result++ = local_code[local_code_chip_index[1]];
            *_result++ = local_code[local_code_chip_index[2]];
            *_result++ = local_code[local_code_chip_index[3]];

            _4output_index = _mm_add_ps(_4output_index, _4constant_float);
        }

    for (number = quarterPoints * 4; number < num_output_samples; number++)
        {
            local_code_chip_index[0] = (int)(code_phase_step_chips * (float)number + rem_code_phase_chips + 0.5f);
            if (local_code_chip_index[0] < 0.0) local_code_chip_index[0] += code_length_chips - 1;
            if (local_code_chip_index[0] > (code_length_chips - 1)) local_code_chip_index[0] -= code_length_chips;
            *_result++ = local_code[local_code_chip_index[0]];
        }
}

#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_resampler_fast_16ic_u_sse2(lv_16sc_t* result, const lv_16sc_t* local_code, float rem_code_phase_chips, float code_phase_step_chips, int code_length_chips, unsigned int num_output_samples)  //, int* scratch_buffer, float* scratch_buffer_float)
{
    _MM_SET_ROUNDING_MODE(_MM_ROUND_NEAREST);  //_MM_ROUND_NEAREST, _MM_ROUND_DOWN, _MM_ROUND_UP, _MM_ROUND_TOWARD_ZERO
    unsigned int number;
    const unsigned int quarterPoints = num_output_samples / 4;

    lv_16sc_t* _result = result;

    __VOLK_ATTR_ALIGNED(16)
    int local_code_chip_index[4];
    __m128 _rem_code_phase, _code_phase_step_chips;
    __m128i _code_length_chips, _code_length_chips_minus1;
    __m128 _code_phase_out, _code_phase_out_with_offset;
    rem_code_phase_chips = rem_code_phase_chips - 0.5f;

    _rem_code_phase = _mm_load1_ps(&rem_code_phase_chips);          //load float to all four float values in m128 register
    _code_phase_step_chips = _mm_load1_ps(&code_phase_step_chips);  //load float to all four float values in m128 register
    __VOLK_ATTR_ALIGNED(16)
    int four_times_code_length_chips_minus1[4];
    four_times_code_length_chips_minus1[0] = code_length_chips - 1;
    four_times_code_length_chips_minus1[1] = code_length_chips - 1;
    four_times_code_length_chips_minus1[2] = code_length_chips - 1;
    four_times_code_length_chips_minus1[3] = code_length_chips - 1;

    __VOLK_ATTR_ALIGNED(16)
    int four_times_code_length_chips[4];
    four_times_code_length_chips[0] = code_length_chips;
    four_times_code_length_chips[1] = code_length_chips;
    four_times_code_length_chips[2] = code_length_chips;
    four_times_code_length_chips[3] = code_length_chips;

    _code_length_chips = _mm_loadu_si128((__m128i*)&four_times_code_length_chips);                //load float to all four float values in m128 register
    _code_length_chips_minus1 = _mm_loadu_si128((__m128i*)&four_times_code_length_chips_minus1);  //load float to all four float values in m128 register

    __m128i negative_indexes, overflow_indexes, _code_phase_out_int, _code_phase_out_int_neg, _code_phase_out_int_over;

    __m128i zero = _mm_setzero_si128();

    __VOLK_ATTR_ALIGNED(16)
    float init_idx_float[4] = {0.0f, 1.0f, 2.0f, 3.0f};
    __m128 _4output_index = _mm_loadu_ps(init_idx_float);
    __VOLK_ATTR_ALIGNED(16)
    float init_4constant_float[4] = {4.0f, 4.0f, 4.0f, 4.0f};
    __m128 _4constant_float = _mm_loadu_ps(init_4constant_float);

    for (number = 0; number < quarterPoints; number++)
        {
            _code_phase_out = _mm_mul_ps(_code_phase_step_chips, _4output_index);        //compute the code phase point with the phase step
            _code_phase_out_with_offset = _mm_add_ps(_code_phase_out, _rem_code_phase);  //add the phase offset
            _code_phase_out_int = _mm_cvtps_epi32(_code_phase_out_with_offset);          //convert to integer

            negative_indexes = _mm_cmplt_epi32(_code_phase_out_int, zero);                     //test for negative values
            _code_phase_out_int_neg = _mm_add_epi32(_code_phase_out_int, _code_length_chips);  //the negative values branch
            _code_phase_out_int_neg = _mm_xor_si128(_code_phase_out_int, _mm_and_si128(negative_indexes, _mm_xor_si128(_code_phase_out_int_neg, _code_phase_out_int)));

            overflow_indexes = _mm_cmpgt_epi32(_code_phase_out_int_neg, _code_length_chips_minus1);  //test for overflow values
            _code_phase_out_int_over = _mm_sub_epi32(_code_phase_out_int_neg, _code_length_chips);   //the negative values branch
            _code_phase_out_int_over = _mm_xor_si128(_code_phase_out_int_neg, _mm_and_si128(overflow_indexes, _mm_xor_si128(_code_phase_out_int_over, _code_phase_out_int_neg)));

            _mm_storeu_si128((__m128i*)local_code_chip_index, _code_phase_out_int_over);  // Store the results back

            //todo: optimize the local code lookup table with intrinsics, if possible
            *_result++ = local_code[local_code_chip_index[0]];
            *_result++ = local_code[local_code_chip_index[1]];
            *_result++ = local_code[local_code_chip_index[2]];
            *_result++ = local_code[local_code_chip_index[3]];

            _4output_index = _mm_add_ps(_4output_index, _4constant_float);
        }

    for (number = quarterPoints * 4; number < num_output_samples; number++)
        {
            local_code_chip_index[0] = (int)(code_phase_step_chips * (float)number + rem_code_phase_chips + 0.5f);
            if (local_code_chip_index[0] < 0.0) local_code_chip_index[0] += code_length_chips - 1;
            if (local_code_chip_index[0] > (code_length_chips - 1)) local_code_chip_index[0] -= code_length_chips;
            *_result++ = local_code[local_code_chip_index[0]];
        }
}

#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_NEONV7
#include <arm_neon.h>

static inline void volk_gnsssdr_16ic_resampler_fast_16ic_neon(lv_16sc_t* result, const lv_16sc_t* local_code, float rem_code_phase_chips, float code_phase_step_chips, int code_length_chips, unsigned int num_output_samples)  //, int* scratch_buffer, float* scratch_buffer_float)
{
    unsigned int number;
    const unsigned int quarterPoints = num_output_samples / 4;
    float32x4_t half = vdupq_n_f32(0.5f);

    lv_16sc_t* _result = result;

    __VOLK_ATTR_ALIGNED(16)
    int local_code_chip_index[4];
    float32x4_t _rem_code_phase, _code_phase_step_chips;
    int32x4_t _code_length_chips, _code_length_chips_minus1;
    float32x4_t _code_phase_out, _code_phase_out_with_offset;
    rem_code_phase_chips = rem_code_phase_chips - 0.5f;
    float32x4_t sign, PlusHalf, Round;

    _rem_code_phase = vld1q_dup_f32(&rem_code_phase_chips);          //load float to all four float values in m128 register
    _code_phase_step_chips = vld1q_dup_f32(&code_phase_step_chips);  //load float to all four float values in m128 register
    __VOLK_ATTR_ALIGNED(16)
    int four_times_code_length_chips_minus1[4];
    four_times_code_length_chips_minus1[0] = code_length_chips - 1;
    four_times_code_length_chips_minus1[1] = code_length_chips - 1;
    four_times_code_length_chips_minus1[2] = code_length_chips - 1;
    four_times_code_length_chips_minus1[3] = code_length_chips - 1;

    __VOLK_ATTR_ALIGNED(16)
    int four_times_code_length_chips[4];
    four_times_code_length_chips[0] = code_length_chips;
    four_times_code_length_chips[1] = code_length_chips;
    four_times_code_length_chips[2] = code_length_chips;
    four_times_code_length_chips[3] = code_length_chips;

    _code_length_chips = vld1q_s32((int32_t*)&four_times_code_length_chips);                //load float to all four float values in m128 register
    _code_length_chips_minus1 = vld1q_s32((int32_t*)&four_times_code_length_chips_minus1);  //load float to all four float values in m128 register

    int32x4_t _code_phase_out_int, _code_phase_out_int_neg, _code_phase_out_int_over;
    uint32x4_t negative_indexes, overflow_indexes;
    int32x4_t zero = vmovq_n_s32(0);

    __VOLK_ATTR_ALIGNED(16)
    float init_idx_float[4] = {0.0f, 1.0f, 2.0f, 3.0f};
    float32x4_t _4output_index = vld1q_f32(init_idx_float);
    __VOLK_ATTR_ALIGNED(16)
    float init_4constant_float[4] = {4.0f, 4.0f, 4.0f, 4.0f};
    float32x4_t _4constant_float = vld1q_f32(init_4constant_float);

    for (number = 0; number < quarterPoints; number++)
        {
            _code_phase_out = vmulq_f32(_code_phase_step_chips, _4output_index);        //compute the code phase point with the phase step
            _code_phase_out_with_offset = vaddq_f32(_code_phase_out, _rem_code_phase);  //add the phase offset
            sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(_code_phase_out_with_offset), 31)));
            PlusHalf = vaddq_f32(_code_phase_out_with_offset, half);
            Round = vsubq_f32(PlusHalf, sign);
            _code_phase_out_int = vcvtq_s32_f32(Round);

            negative_indexes = vcltq_s32(_code_phase_out_int, zero);                       //test for negative values
            _code_phase_out_int_neg = vaddq_s32(_code_phase_out_int, _code_length_chips);  //the negative values branch
            _code_phase_out_int_neg = veorq_s32(_code_phase_out_int, vandq_s32((int32x4_t)negative_indexes, veorq_s32(_code_phase_out_int_neg, _code_phase_out_int)));

            overflow_indexes = vcgtq_s32(_code_phase_out_int_neg, _code_length_chips_minus1);   //test for overflow values
            _code_phase_out_int_over = vsubq_s32(_code_phase_out_int_neg, _code_length_chips);  //the negative values branch
            _code_phase_out_int_over = veorq_s32(_code_phase_out_int_neg, vandq_s32((int32x4_t)overflow_indexes, veorq_s32(_code_phase_out_int_over, _code_phase_out_int_neg)));

            vst1q_s32((int32_t*)local_code_chip_index, _code_phase_out_int_over);  // Store the results back

            //todo: optimize the local code lookup table with intrinsics, if possible
            *_result++ = local_code[local_code_chip_index[0]];
            *_result++ = local_code[local_code_chip_index[1]];
            *_result++ = local_code[local_code_chip_index[2]];
            *_result++ = local_code[local_code_chip_index[3]];

            _4output_index = vaddq_f32(_4output_index, _4constant_float);
        }

    for (number = quarterPoints * 4; number < num_output_samples; number++)
        {
            local_code_chip_index[0] = (int)(code_phase_step_chips * (float)number + rem_code_phase_chips + 0.5f);
            if (local_code_chip_index[0] < 0.0) local_code_chip_index[0] += code_length_chips - 1;
            if (local_code_chip_index[0] > (code_length_chips - 1)) local_code_chip_index[0] -= code_length_chips;
            *_result++ = local_code[local_code_chip_index[0]];
        }
}

#endif /* LV_HAVE_NEONV7 */

#endif /*INCLUDED_volk_gnsssdr_16ic_resampler_fast_16ic_H*/
