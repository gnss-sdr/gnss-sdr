/*!
 * \file volk_gnsssdr_32fc_convert_16ic.h
 * \brief VOLK_GNSSSDR kernel: converts float32 complex values to 16 integer complex values taking care of overflow.
 * \authors <ul>
 *          <li> Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

/*!
 * \page volk_gnsssdr_32fc_convert_16ic
 *
 * \b Overview
 *
 * Converts a complex vector of 32-bits float each component into
 * a complex vector of 16-bits integer each component.
 * Values are saturated to the limit values of the output data type.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_32fc_convert_16ic(lv_16sc_t* outputVector, const lv_32fc_t* inputVector, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li inputVector:  The complex 32-bit float input data buffer.
 * \li num_points:   The number of data values to be converted.
 *
 * \b Outputs
 * \li outputVector: The complex 16-bit integer output data buffer.
 *
 */

#ifndef INCLUDED_volk_gnsssdr_32fc_convert_16ic_H
#define INCLUDED_volk_gnsssdr_32fc_convert_16ic_H

#include "volk_gnsssdr/volk_gnsssdr_complex.h"
#include <limits.h>
#include <math.h>

#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_32fc_convert_16ic_u_sse2(lv_16sc_t* outputVector, const lv_32fc_t* inputVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;

    float* inputVectorPtr = (float*)inputVector;
    int16_t* outputVectorPtr = (int16_t*)outputVector;
    float aux;
    unsigned int i;
    const float min_val = (float)SHRT_MIN;
    const float max_val = (float)SHRT_MAX;

    __m128 inputVal1, inputVal2;
    __m128i intInputVal1, intInputVal2;
    __m128 ret1, ret2;
    const __m128 vmin_val = _mm_set_ps1(min_val);
    const __m128 vmax_val = _mm_set_ps1(max_val);

    for (i = 0; i < sse_iters; i++)
        {
            inputVal1 = _mm_loadu_ps((float*)inputVectorPtr);
            inputVectorPtr += 4;
            inputVal2 = _mm_loadu_ps((float*)inputVectorPtr);
            inputVectorPtr += 4;
            __VOLK_GNSSSDR_PREFETCH(inputVectorPtr + 8);

            // Clip
            ret1 = _mm_max_ps(_mm_min_ps(inputVal1, vmax_val), vmin_val);
            ret2 = _mm_max_ps(_mm_min_ps(inputVal2, vmax_val), vmin_val);

            intInputVal1 = _mm_cvtps_epi32(ret1);
            intInputVal2 = _mm_cvtps_epi32(ret2);

            intInputVal1 = _mm_packs_epi32(intInputVal1, intInputVal2);

            _mm_storeu_si128((__m128i*)outputVectorPtr, intInputVal1);
            outputVectorPtr += 8;
        }

    for (i = sse_iters * 8; i < num_points * 2; i++)
        {
            aux = *inputVectorPtr++;
            if (aux > max_val)
                aux = max_val;
            else if (aux < min_val)
                aux = min_val;
            *outputVectorPtr++ = (int16_t)rintf(aux);
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_SSE
#include <xmmintrin.h>

static inline void volk_gnsssdr_32fc_convert_16ic_u_sse(lv_16sc_t* outputVector, const lv_32fc_t* inputVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;

    float* inputVectorPtr = (float*)inputVector;
    int16_t* outputVectorPtr = (int16_t*)outputVector;
    float aux;
    unsigned int i;

    const float min_val = (float)SHRT_MIN;
    const float max_val = (float)SHRT_MAX;

    __m128 inputVal1, inputVal2;
    __m128i intInputVal1, intInputVal2;  // is __m128i defined in xmmintrin.h?
    __m128 ret1, ret2;
    const __m128 vmin_val = _mm_set_ps1(min_val);
    const __m128 vmax_val = _mm_set_ps1(max_val);

    for (i = 0; i < sse_iters; i++)
        {
            inputVal1 = _mm_loadu_ps((float*)inputVectorPtr);
            inputVectorPtr += 4;
            inputVal2 = _mm_loadu_ps((float*)inputVectorPtr);
            inputVectorPtr += 4;
            __VOLK_GNSSSDR_PREFETCH(inputVectorPtr + 8);

            // Clip
            ret1 = _mm_max_ps(_mm_min_ps(inputVal1, vmax_val), vmin_val);
            ret2 = _mm_max_ps(_mm_min_ps(inputVal2, vmax_val), vmin_val);

            intInputVal1 = _mm_cvtps_epi32(ret1);
            intInputVal2 = _mm_cvtps_epi32(ret2);

            intInputVal1 = _mm_packs_epi32(intInputVal1, intInputVal2);

            _mm_storeu_si128((__m128i*)outputVectorPtr, intInputVal1);
            outputVectorPtr += 8;
        }

    for (i = sse_iters * 8; i < num_points * 2; i++)
        {
            aux = *inputVectorPtr++;
            if (aux > max_val)
                aux = max_val;
            else if (aux < min_val)
                aux = min_val;
            *outputVectorPtr++ = (int16_t)rintf(aux);
        }
}
#endif /* LV_HAVE_SSE */

#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_32fc_convert_16ic_u_avx2(lv_16sc_t* outputVector, const lv_32fc_t* inputVector, unsigned int num_points)
{
    const unsigned int avx2_iters = num_points / 8;

    float* inputVectorPtr = (float*)inputVector;
    int16_t* outputVectorPtr = (int16_t*)outputVector;
    float aux;
    unsigned int i;
    const float min_val = (float)SHRT_MIN;  /// todo Something off here, compiler does not perform right cast
    const float max_val = (float)SHRT_MAX;

    __m256 inputVal1, inputVal2;
    __m256i intInputVal1, intInputVal2;
    __m256 ret1, ret2;
    const __m256 vmin_val = _mm256_set1_ps(min_val);
    const __m256 vmax_val = _mm256_set1_ps(max_val);

    for (i = 0; i < avx2_iters; i++)
        {
            inputVal1 = _mm256_loadu_ps((float*)inputVectorPtr);
            inputVectorPtr += 8;
            inputVal2 = _mm256_loadu_ps((float*)inputVectorPtr);
            inputVectorPtr += 8;
            __VOLK_GNSSSDR_PREFETCH(inputVectorPtr + 16);

            // Clip
            ret1 = _mm256_max_ps(_mm256_min_ps(inputVal1, vmax_val), vmin_val);
            ret2 = _mm256_max_ps(_mm256_min_ps(inputVal2, vmax_val), vmin_val);

            intInputVal1 = _mm256_cvtps_epi32(ret1);
            intInputVal2 = _mm256_cvtps_epi32(ret2);

            intInputVal1 = _mm256_packs_epi32(intInputVal1, intInputVal2);
            intInputVal1 = _mm256_permute4x64_epi64(intInputVal1, 0b11011000);

            _mm256_storeu_si256((__m256i*)outputVectorPtr, intInputVal1);
            outputVectorPtr += 16;
        }

    for (i = avx2_iters * 16; i < num_points * 2; i++)
        {
            aux = *inputVectorPtr++;
            if (aux > max_val)
                aux = max_val;
            else if (aux < min_val)
                aux = min_val;
            *outputVectorPtr++ = (int16_t)rintf(aux);
        }
}
#endif /* LV_HAVE_AVX2 */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_32fc_convert_16ic_a_sse2(lv_16sc_t* outputVector, const lv_32fc_t* inputVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;

    float* inputVectorPtr = (float*)inputVector;
    int16_t* outputVectorPtr = (int16_t*)outputVector;
    float aux;
    unsigned int i;

    const float min_val = (float)SHRT_MIN;
    const float max_val = (float)SHRT_MAX;

    __m128 inputVal1, inputVal2;
    __m128i intInputVal1, intInputVal2;
    __m128 ret1, ret2;
    const __m128 vmin_val = _mm_set_ps1(min_val);
    const __m128 vmax_val = _mm_set_ps1(max_val);

    for (i = 0; i < sse_iters; i++)
        {
            inputVal1 = _mm_load_ps((float*)inputVectorPtr);
            inputVectorPtr += 4;
            inputVal2 = _mm_load_ps((float*)inputVectorPtr);
            inputVectorPtr += 4;
            __VOLK_GNSSSDR_PREFETCH(inputVectorPtr + 8);

            // Clip
            ret1 = _mm_max_ps(_mm_min_ps(inputVal1, vmax_val), vmin_val);
            ret2 = _mm_max_ps(_mm_min_ps(inputVal2, vmax_val), vmin_val);

            intInputVal1 = _mm_cvtps_epi32(ret1);
            intInputVal2 = _mm_cvtps_epi32(ret2);

            intInputVal1 = _mm_packs_epi32(intInputVal1, intInputVal2);

            _mm_store_si128((__m128i*)outputVectorPtr, intInputVal1);
            outputVectorPtr += 8;
        }

    for (i = sse_iters * 8; i < num_points * 2; i++)
        {
            aux = *inputVectorPtr++;
            if (aux > max_val)
                aux = max_val;
            else if (aux < min_val)
                aux = min_val;
            *outputVectorPtr++ = (int16_t)rintf(aux);
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_SSE
#include <xmmintrin.h>

static inline void volk_gnsssdr_32fc_convert_16ic_a_sse(lv_16sc_t* outputVector, const lv_32fc_t* inputVector, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    const float min_val = (float)SHRT_MIN;
    const float max_val = (float)SHRT_MAX;
    float aux;
    unsigned int i;
    float* inputVectorPtr = (float*)inputVector;
    int16_t* outputVectorPtr = (int16_t*)outputVector;

    __m128 inputVal1, inputVal2;
    __m128i intInputVal1, intInputVal2;
    __m128 ret1, ret2;
    const __m128 vmin_val = _mm_set_ps1(min_val);
    const __m128 vmax_val = _mm_set_ps1(max_val);

    for (i = 0; i < sse_iters; i++)
        {
            inputVal1 = _mm_load_ps((float*)inputVectorPtr);
            inputVectorPtr += 4;
            inputVal2 = _mm_load_ps((float*)inputVectorPtr);
            inputVectorPtr += 4;
            __VOLK_GNSSSDR_PREFETCH(inputVectorPtr + 8);

            // Clip
            ret1 = _mm_max_ps(_mm_min_ps(inputVal1, vmax_val), vmin_val);
            ret2 = _mm_max_ps(_mm_min_ps(inputVal2, vmax_val), vmin_val);

            intInputVal1 = _mm_cvtps_epi32(ret1);
            intInputVal2 = _mm_cvtps_epi32(ret2);

            intInputVal1 = _mm_packs_epi32(intInputVal1, intInputVal2);

            _mm_store_si128((__m128i*)outputVectorPtr, intInputVal1);
            outputVectorPtr += 8;
        }

    for (i = sse_iters * 8; i < num_points * 2; i++)
        {
            aux = *inputVectorPtr++;
            if (aux > max_val)
                aux = max_val;
            else if (aux < min_val)
                aux = min_val;
            *outputVectorPtr++ = (int16_t)rintf(aux);
        }
}
#endif /* LV_HAVE_SSE */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_32fc_convert_16ic_a_avx2(lv_16sc_t* outputVector, const lv_32fc_t* inputVector, unsigned int num_points)
{
    const unsigned int avx2_iters = num_points / 8;

    float* inputVectorPtr = (float*)inputVector;
    int16_t* outputVectorPtr = (int16_t*)outputVector;
    float aux;
    unsigned int i;
    const float min_val = (float)SHRT_MIN;  /// todo Something off here, compiler does not perform right cast
    const float max_val = (float)SHRT_MAX;

    __m256 inputVal1, inputVal2;
    __m256i intInputVal1, intInputVal2;
    __m256 ret1, ret2;
    const __m256 vmin_val = _mm256_set1_ps(min_val);
    const __m256 vmax_val = _mm256_set1_ps(max_val);

    for (i = 0; i < avx2_iters; i++)
        {
            inputVal1 = _mm256_load_ps((float*)inputVectorPtr);
            inputVectorPtr += 8;
            inputVal2 = _mm256_load_ps((float*)inputVectorPtr);
            inputVectorPtr += 8;
            __VOLK_GNSSSDR_PREFETCH(inputVectorPtr + 16);

            // Clip
            ret1 = _mm256_max_ps(_mm256_min_ps(inputVal1, vmax_val), vmin_val);
            ret2 = _mm256_max_ps(_mm256_min_ps(inputVal2, vmax_val), vmin_val);

            intInputVal1 = _mm256_cvtps_epi32(ret1);
            intInputVal2 = _mm256_cvtps_epi32(ret2);

            intInputVal1 = _mm256_packs_epi32(intInputVal1, intInputVal2);
            intInputVal1 = _mm256_permute4x64_epi64(intInputVal1, 0b11011000);

            _mm256_store_si256((__m256i*)outputVectorPtr, intInputVal1);
            outputVectorPtr += 16;
        }

    for (i = avx2_iters * 16; i < num_points * 2; i++)
        {
            aux = *inputVectorPtr++;
            if (aux > max_val)
                aux = max_val;
            else if (aux < min_val)
                aux = min_val;
            *outputVectorPtr++ = (int16_t)rintf(aux);
        }
}
#endif /* LV_HAVE_AVX2 */


#ifdef LV_HAVE_NEON
#include <arm_neon.h>

static inline void volk_gnsssdr_32fc_convert_16ic_neon(lv_16sc_t* outputVector, const lv_32fc_t* inputVector, unsigned int num_points)
{
    const unsigned int neon_iters = num_points / 4;

    float32_t* inputVectorPtr = (float32_t*)inputVector;
    int16_t* outputVectorPtr = (int16_t*)outputVector;

    const float min_val_f = (float)SHRT_MIN;
    const float max_val_f = (float)SHRT_MAX;
    float32_t aux;
    unsigned int i;
    const float32x4_t min_val = vmovq_n_f32(min_val_f);
    const float32x4_t max_val = vmovq_n_f32(max_val_f);
    float32x4_t half = vdupq_n_f32(0.5f);
    float32x4_t ret1, ret2, a, b, sign, PlusHalf, Round;

    int32x4_t toint_a, toint_b;
    int16x4_t intInputVal1, intInputVal2;
    int16x8_t res;

    for (i = 0; i < neon_iters; i++)
        {
            a = vld1q_f32((const float32_t*)(inputVectorPtr));
            inputVectorPtr += 4;
            b = vld1q_f32((const float32_t*)(inputVectorPtr));
            inputVectorPtr += 4;
            __VOLK_GNSSSDR_PREFETCH(inputVectorPtr + 8);

            ret1 = vmaxq_f32(vminq_f32(a, max_val), min_val);
            ret2 = vmaxq_f32(vminq_f32(b, max_val), min_val);

            /* in __aarch64__ we can do that with vcvtaq_s32_f32(ret1); vcvtaq_s32_f32(ret2); */
            sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(ret1), 31)));
            PlusHalf = vaddq_f32(ret1, half);
            Round = vsubq_f32(PlusHalf, sign);
            toint_a = vcvtq_s32_f32(Round);

            sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(ret2), 31)));
            PlusHalf = vaddq_f32(ret2, half);
            Round = vsubq_f32(PlusHalf, sign);
            toint_b = vcvtq_s32_f32(Round);

            intInputVal1 = vqmovn_s32(toint_a);
            intInputVal2 = vqmovn_s32(toint_b);

            res = vcombine_s16(intInputVal1, intInputVal2);
            vst1q_s16((int16_t*)outputVectorPtr, res);
            outputVectorPtr += 8;
        }

    for (i = neon_iters * 8; i < num_points * 2; i++)
        {
            aux = *inputVectorPtr++;
            if (aux > max_val_f)
                aux = max_val_f;
            else if (aux < min_val_f)
                aux = min_val_f;
            *outputVectorPtr++ = (int16_t)rintf(aux);
        }
}

#endif /* LV_HAVE_NEON */


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_32fc_convert_16ic_generic(lv_16sc_t* outputVector, const lv_32fc_t* inputVector, unsigned int num_points)
{
    float* inputVectorPtr = (float*)inputVector;
    int16_t* outputVectorPtr = (int16_t*)outputVector;
    const float min_val = (float)SHRT_MIN;
    const float max_val = (float)SHRT_MAX;
    float aux;
    unsigned int i;
    for (i = 0; i < num_points * 2; i++)
        {
            aux = *inputVectorPtr++;
            if (aux > max_val)
                aux = max_val;
            else if (aux < min_val)
                aux = min_val;
            *outputVectorPtr++ = (int16_t)rintf(aux);
        }
}
#endif /* LV_HAVE_GENERIC */

#ifdef LV_HAVE_RVV
#include <riscv_vector.h>

static inline void volk_gnsssdr_32fc_convert_16ic_rvv(lv_16sc_t* outputVector, const lv_32fc_t* inputVector, unsigned int num_points)
{
    // Will be converting by number, with each
    // complex number containing two component numbers
    size_t n = num_points * 2;

    // Initialize pointers to keep track as stripmine
    short* outPtr = (short*) outputVector;
    const float* inPtr = (const float*) inputVector;

    for (size_t vl; n > 0; n -= vl, outPtr += vl, inPtr += vl)
        {
            // Record how many elements will actually be processed
            vl = __riscv_vsetvl_e32m8(n);

            // Don't have to segment load/store since converting
            // both real and imaginary components
            // Load in[0..vl)
            vfloat32m8_t inVal = __riscv_vle32_v_f32m8(inPtr, vl);

            // Saturate in[i] to 16 bits
            inVal = __riscv_vfmin_vf_f32m8(inVal, (float) 32767, vl);
            inVal = __riscv_vfmax_vf_f32m8(inVal, (float) -32768, vl);

            // out[i] = (short) in[i]
            vint16m4_t outVal = __riscv_vfncvt_x_f_w_i16m4(inVal, vl);

            // Store out[0..vl)
            __riscv_vse16_v_i16m4(outPtr, outVal, vl);

            // In looping, decrement the number of
            // elements left and increment the pointers
            // by the number of elements processed,
            // accounting how the `vl` complex numbers
            // are each stored as two numbers of their
            // corresponding size.
        }
}
#endif /* LV_HAVE_RVV */

#endif /* INCLUDED_volk_gnsssdr_32fc_convert_16ic_H */
