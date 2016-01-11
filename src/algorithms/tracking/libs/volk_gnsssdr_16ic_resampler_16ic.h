/*!
 * \file volk_gnsssdr_16ic_resampler_16ic.h
 * \brief Volk protokernel: resample a 16 bits complex vector
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * Volk protokernel that multiplies two 16 bits vectors (8 bits the real part 
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

#ifndef INCLUDED_volk_gnsssdr_16ic_resampler_16ic_a_H
#define INCLUDED_volk_gnsssdr_16ic_resampler_16ic_a_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <cmath>
//#pragma STDC FENV_ACCESS ON

#ifdef LV_HAVE_GENERIC

//int round_int( float r ) {
//    return (r > 0.0) ? (r + 0.5) : (r - 0.5);
//}
/*!
 \brief Multiplies the two input complex vectors, point-by-point, storing the result in the third vector
 \param cVector The vector where the result will be stored
 \param aVector One of the vectors to be multiplied
 \param bVector One of the vectors to be multiplied
 \param num_points The number of complex values in aVector and bVector to be multiplied together, accumulated and stored into cVector
 */
static inline void volk_gnsssdr_16ic_resampler_16ic_generic(lv_16sc_t* result, const lv_16sc_t* local_code, float rem_code_phase_chips ,float code_phase_step_chips, unsigned int num_output_samples, unsigned int code_length_chips)
{
    int local_code_chip_index;
    //fesetround(FE_TONEAREST);
    for (unsigned int n = 0; n < num_output_samples; n++)
        {
			// resample code for current tap
			local_code_chip_index = round(code_phase_step_chips*static_cast<float>(n) + rem_code_phase_chips-0.5f);
	    	if (local_code_chip_index < 0.0) local_code_chip_index += code_length_chips;
	    	if (local_code_chip_index > (code_length_chips-1)) local_code_chip_index -= code_length_chips;
			//std::cout<<"g["<<n<<"]="<<code_phase_step_chips*static_cast<float>(n) + rem_code_phase_chips-0.5f<<","<<local_code_chip_index<<" ";
			result[n] = local_code[local_code_chip_index];
        }
    //std::cout<<std::endl;
}

#endif /*LV_HAVE_GENERIC*/


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
static inline void volk_gnsssdr_16ic_resampler_16ic_sse2(lv_16sc_t* result, const lv_16sc_t* local_code, float rem_code_phase_chips ,float code_phase_step_chips, unsigned int num_output_samples, int code_length_chips)//, int* scratch_buffer, float* scratch_buffer_float)
{

	_MM_SET_ROUNDING_MODE (_MM_ROUND_NEAREST);//_MM_ROUND_NEAREST, _MM_ROUND_DOWN, _MM_ROUND_UP, _MM_ROUND_TOWARD_ZERO
    unsigned int number;
    const unsigned int quarterPoints = num_output_samples / 4;

	lv_16sc_t* _result = result;

    __attribute__((aligned(16))) int local_code_chip_index[4];
    __m128 _rem_code_phase,_code_phase_step_chips;
    __m128i _code_length_chips,_code_length_chips_minus1;
    __m128 _code_phase_out,_code_phase_out_with_offset;
    rem_code_phase_chips=rem_code_phase_chips-0.5f;

    _rem_code_phase =  _mm_load1_ps(&rem_code_phase_chips); //load float to all four float values in m128 register
    _code_phase_step_chips = _mm_load1_ps(&code_phase_step_chips); //load float to all four float values in m128 register
    __attribute__((aligned(16))) int four_times_code_length_chips_minus1[4];
    four_times_code_length_chips_minus1[0]=code_length_chips-1;
    four_times_code_length_chips_minus1[1]=code_length_chips-1;
    four_times_code_length_chips_minus1[2]=code_length_chips-1;
    four_times_code_length_chips_minus1[3]=code_length_chips-1;

    __attribute__((aligned(16))) int four_times_code_length_chips[4];
    four_times_code_length_chips[0]=code_length_chips;
    four_times_code_length_chips[1]=code_length_chips;
    four_times_code_length_chips[2]=code_length_chips;
    four_times_code_length_chips[3]=code_length_chips;

    _code_length_chips = _mm_loadu_si128((__m128i*)&four_times_code_length_chips); //load float to all four float values in m128 register
    _code_length_chips_minus1 = _mm_loadu_si128((__m128i*)&four_times_code_length_chips_minus1); //load float to all four float values in m128 register

    __m128i negative_indexes, overflow_indexes,_code_phase_out_int, _code_phase_out_int_neg,_code_phase_out_int_over;

    __m128i zero=_mm_setzero_si128();


    __attribute__((aligned(16))) float init_idx_float[4] = { 0.0f, 1.0f, 2.0f, 3.0f };
    __m128 _4output_index=_mm_load_ps(init_idx_float);
    __attribute__((aligned(16))) float init_4constant_float[4] = { 4.0f, 4.0f, 4.0f, 4.0f };
    __m128 _4constant_float=_mm_load_ps(init_4constant_float);

    //__attribute__((aligned(16))) int output_indexes[4];

    for(number=0;number < quarterPoints; number++){
    	_code_phase_out = _mm_mul_ps(_code_phase_step_chips, _4output_index); //compute the code phase point with the phase step
    	_code_phase_out_with_offset = _mm_add_ps(_code_phase_out,_rem_code_phase); //add the phase offset
    	_code_phase_out_int=_mm_cvtps_epi32(_code_phase_out_with_offset); //convert to integer

    	negative_indexes=_mm_cmplt_epi32 (_code_phase_out_int, zero); //test for negative values
    	_code_phase_out_int_neg=_mm_add_epi32(_code_phase_out_int,_code_length_chips); //the negative values branch
    	//_code_phase_out_int_over=_mm_or_si128(_mm_and_si128(_code_phase_out_int_neg,_code_phase_out_int),_mm_andnot_si128(negative_indexes,_code_phase_out_int));
    	_code_phase_out_int_neg=_mm_xor_si128(_code_phase_out_int,_mm_and_si128( negative_indexes,_mm_xor_si128( _code_phase_out_int_neg, _code_phase_out_int )));

    	overflow_indexes=_mm_cmpgt_epi32  (_code_phase_out_int_neg, _code_length_chips_minus1); //test for overflow values
    	_code_phase_out_int_over=_mm_sub_epi32(_code_phase_out_int_neg,_code_length_chips); //the negative values branch
    	_code_phase_out_int_over=_mm_xor_si128(_code_phase_out_int_neg,_mm_and_si128( overflow_indexes,_mm_xor_si128( _code_phase_out_int_over, _code_phase_out_int_neg )));

    	_mm_storeu_si128((__m128i*)local_code_chip_index,_code_phase_out_int_over); // Store the results back

    	//_mm_store_ps((float*)_scratch_buffer_float,_code_phase_out_with_offset);

    	//todo: optimize the local code lookup table with intrinsics, if possible
    	*_result++=local_code[local_code_chip_index[0]];
    	*_result++=local_code[local_code_chip_index[1]];
    	*_result++=local_code[local_code_chip_index[2]];
    	*_result++=local_code[local_code_chip_index[3]];

    	_4output_index = _mm_add_ps(_4output_index,_4constant_float);
    	//_scratch_buffer_float+=4;

    }

    for(number = quarterPoints * 4;number < num_output_samples; number++){
    	local_code_chip_index[0]=static_cast<int>(code_phase_step_chips*static_cast<float>(number) + rem_code_phase_chips+0.5f);
    	if (local_code_chip_index[0] < 0.0) local_code_chip_index[0] += code_length_chips-1;
    	if (local_code_chip_index[0]  > (code_length_chips-1)) local_code_chip_index[0] -= code_length_chips;
    	*_result++=local_code[local_code_chip_index[0]];
    	//*_scratch_buffer_float++=code_phase_step_chips*static_cast<float>(number)+rem_code_phase_chips;
    }

//    for(unsigned int n=0;n<num_output_samples;n++)
//    {
//
//		std::cout<<"s["<<n<<"]="<<scratch_buffer_float[n]<<","<<scratch_buffer[n]<<" ";
//    }
//    std::cout<<std::endl;




}
#endif /* LV_HAVE_SSE2 */

#endif /*INCLUDED_volk_gnsssdr_16ic_resampler_16ic_a_H*/
