/*!
 * \file volk_16ic_s32fc_x2_rotator_16ic.h
 * \brief Volk protokernel: rotates a 16 bits complex vector
 * \authors <ul>
 *          <li> Carles Fernandez-Prades, 2015  cfernandez at cttc.es
 *          </ul>
 *
 * Volk protokernel that rotates a 16-bit complex vector
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


#ifndef INCLUDED_volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_H

#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <math.h>

#define ROTATOR_RELOAD 512


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_generic(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    unsigned int i = 0;
    int j = 0;
    lv_16sc_t tmp16;
    lv_32fc_t tmp32;
    for(i = 0; i < (unsigned int)(num_points / ROTATOR_RELOAD); ++i)
        {
            for(j = 0; j < ROTATOR_RELOAD; ++j)
                {
                    tmp16 = *inVector++;
                    tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
                    *outVector++ = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
                    (*phase) *= phase_inc;
                }
        }
    for(i = 0; i < num_points % ROTATOR_RELOAD; ++i)
        {
            tmp16 = *inVector++;
            tmp32 = lv_cmake((float)lv_creal(tmp16), (float)lv_cimag(tmp16)) * (*phase);
            *outVector++ = lv_cmake((int16_t)rintf(lv_creal(tmp32)), (int16_t)rintf(lv_cimag(tmp32)));
            (*phase) *= phase_inc;
        }
}

#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_a_sse2(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    __m128i a,b,c, c_sr, mask_imag, mask_real, real, imag, imag1,imag2, b_sl, a_sl, result;

    mask_imag = _mm_set_epi8(255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0);
    mask_real = _mm_set_epi8(0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255);

    const lv_16sc_t* _in_a = inVector;
    __attribute__((aligned(32))) lv_32fc_t four_phase_rotations_32fc[4];
    // debug
    //__attribute__((aligned(16))) lv_16sc_t four_phase_rotations_16sc[4];

    // specify how many bits are used in the rotation (2^(N-1)) (it WILL increase the output signal range!)
    __attribute__((aligned(32))) float rotator_amplitude_float[4] = { 4.0f, 4.0f, 4.0f, 4.0f };
    __m128 _rotator_amplitude_reg = _mm_load_ps(rotator_amplitude_float);

    //const lv_16sc_t* _in_b = in_b;
    lv_16sc_t* _out = outVector;

    __m128 fc_reg1, fc_reg2;
    __m128i sc_reg1, sc_reg2; // is __m128i defined in xmmintrin.h?

    for(unsigned int number = 0; number < sse_iters; number++)
        {
            //std::complex<T> memory structure: real part -> reinterpret_cast<cv T*>(a)[2*i]
            //imaginery part -> reinterpret_cast<cv T*>(a)[2*i + 1]
            // a[127:0]=[a3.i,a3.r,a2.i,a2.r,a1.i,a1.r,a0.i,a0.r]
            a = _mm_load_si128((__m128i*)_in_a); //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
            //b = _mm_loadu_si128((__m128i*)_in_b);

            // compute next four 16ic complex exponential values for phase rotation

            // compute next four float complex rotations
            four_phase_rotations_32fc[0]=*phase;
            (*phase) *= phase_inc;
            four_phase_rotations_32fc[1]=*phase;
            (*phase) *= phase_inc;
            four_phase_rotations_32fc[2]=*phase;
            (*phase) *= phase_inc;
            four_phase_rotations_32fc[3]=*phase;
            (*phase) *= phase_inc;
            //convert the rotations to integers
            fc_reg1 = _mm_load_ps((float*)&four_phase_rotations_32fc[0]);

            // disable next line for 1 bit rotation (equivalent to a square wave NCO)
            fc_reg1 = _mm_mul_ps (fc_reg1, _rotator_amplitude_reg);

            fc_reg2 = _mm_load_ps((float*)&four_phase_rotations_32fc[2]);
            sc_reg1 = _mm_cvtps_epi32(fc_reg1);
            sc_reg2 = _mm_cvtps_epi32(fc_reg2);
            b = _mm_packs_epi32(sc_reg1, sc_reg2);

            // debug
            //_mm_store_si128((__m128i*)four_phase_rotations_16sc, b);
            //printf("phase fc: %f,%f phase sc: %i,%i \n",lv_creal(four_phase_rotations_32fc[0]),lv_cimag(four_phase_rotations_32fc[0]),lv_creal(four_phase_rotations_16sc[0]),lv_cimag(four_phase_rotations_16sc[0]));

            // multiply the input vector times the rotations
            c = _mm_mullo_epi16 (a, b); // a3.i*b3.i, a3.r*b3.r, ....

            c_sr = _mm_srli_si128 (c, 2); // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
            real = _mm_subs_epi16 (c, c_sr);
            real = _mm_and_si128 (real, mask_real); // a3.r*b3.r-a3.i*b3.i , 0,  a3.r*b3.r- a3.i*b3.i

            b_sl = _mm_slli_si128(b, 2); // b3.r, b2.i ....
            a_sl = _mm_slli_si128(a, 2); // a3.r, a2.i ....

            imag1 = _mm_mullo_epi16(a, b_sl); // a3.i*b3.r, ....
            imag2 = _mm_mullo_epi16(b, a_sl); // b3.i*a3.r, ....

            imag = _mm_adds_epi16(imag1, imag2);
            imag = _mm_and_si128 (imag, mask_imag); // a3.i*b3.r+b3.i*a3.r, 0, ...

            result = _mm_or_si128 (real, imag);

            // normalize the rotations
            // TODO

            // store results
            _mm_store_si128((__m128i*)_out, result);

            _in_a += 4;
            _out += 4;
        }

    for (unsigned int i = sse_iters * 4; i < num_points; ++i)
        {
            *_out++ = *_in_a++ * (*phase);
            (*phase) *= phase_inc;
        }
}
#endif /* LV_HAVE_SSE2 */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_u_sse2(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    const unsigned int sse_iters = num_points / 4;
    __m128i a,b,c, c_sr, mask_imag, mask_real, real, imag, imag1,imag2, b_sl, a_sl, result;

    mask_imag = _mm_set_epi8(255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0);
    mask_real = _mm_set_epi8(0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255, 255);

    const lv_16sc_t* _in_a = inVector;
    __attribute__((aligned(32))) lv_32fc_t four_phase_rotations_32fc[4];
    // debug
    //__attribute__((aligned(16))) lv_16sc_t four_phase_rotations_16sc[4];

    // specify how many bits are used in the rotation (2^(N-1)) (it WILL increase the output signal range!)
    __attribute__((aligned(32))) float rotator_amplitude_float[4] = { 4.0f, 4.0f, 4.0f, 4.0f };
    __m128 _rotator_amplitude_reg = _mm_load_ps(rotator_amplitude_float);

    //const lv_16sc_t* _in_b = in_b;
    lv_16sc_t* _out = outVector;

    __m128 fc_reg1, fc_reg2;
    __m128i sc_reg1, sc_reg2; // is __m128i defined in xmmintrin.h?

    for(unsigned int number = 0; number < sse_iters; number++)
        {
            //std::complex<T> memory structure: real part -> reinterpret_cast<cv T*>(a)[2*i]
            //imaginery part -> reinterpret_cast<cv T*>(a)[2*i + 1]
            // a[127:0]=[a3.i,a3.r,a2.i,a2.r,a1.i,a1.r,a0.i,a0.r]
            a = _mm_loadu_si128((__m128i*)_in_a); //load (2 byte imag, 2 byte real) x 4 into 128 bits reg
            //b = _mm_loadu_si128((__m128i*)_in_b);

            // compute next four 16ic complex exponential values for phase rotation

            // compute next four float complex rotations
            four_phase_rotations_32fc[0]=*phase;
            (*phase) *= phase_inc;
            four_phase_rotations_32fc[1]=*phase;
            (*phase) *= phase_inc;
            four_phase_rotations_32fc[2]=*phase;
            (*phase) *= phase_inc;
            four_phase_rotations_32fc[3]=*phase;
            (*phase) *= phase_inc;
            //convert the rotations to integers
            fc_reg1 = _mm_load_ps((float*)&four_phase_rotations_32fc[0]);

            // disable next line for 1 bit rotation (equivalent to a square wave NCO)
            fc_reg1 = _mm_mul_ps (fc_reg1, _rotator_amplitude_reg);

            fc_reg2 = _mm_load_ps((float*)&four_phase_rotations_32fc[2]);
            sc_reg1 = _mm_cvtps_epi32(fc_reg1);
            sc_reg2 = _mm_cvtps_epi32(fc_reg2);
            b = _mm_packs_epi32(sc_reg1, sc_reg2);

            // debug
            //_mm_store_si128((__m128i*)four_phase_rotations_16sc, b);
            //printf("phase fc: %f,%f phase sc: %i,%i \n",lv_creal(four_phase_rotations_32fc[0]),lv_cimag(four_phase_rotations_32fc[0]),lv_creal(four_phase_rotations_16sc[0]),lv_cimag(four_phase_rotations_16sc[0]));

            // multiply the input vector times the rotations
            c = _mm_mullo_epi16 (a, b); // a3.i*b3.i, a3.r*b3.r, ....

            c_sr = _mm_srli_si128 (c, 2); // Shift a right by imm8 bytes while shifting in zeros, and store the results in dst.
            real = _mm_subs_epi16 (c, c_sr);
            real = _mm_and_si128 (real, mask_real); // a3.r*b3.r-a3.i*b3.i , 0,  a3.r*b3.r- a3.i*b3.i

            b_sl = _mm_slli_si128(b, 2); // b3.r, b2.i ....
            a_sl = _mm_slli_si128(a, 2); // a3.r, a2.i ....

            imag1 = _mm_mullo_epi16(a, b_sl); // a3.i*b3.r, ....
            imag2 = _mm_mullo_epi16(b, a_sl); // b3.i*a3.r, ....

            imag = _mm_adds_epi16(imag1, imag2);
            imag = _mm_and_si128 (imag, mask_imag); // a3.i*b3.r+b3.i*a3.r, 0, ...

            result = _mm_or_si128 (real, imag);

            // normalize the rotations
            // TODO

            // store results
            _mm_storeu_si128((__m128i*)_out, result);

            _in_a += 4;
            _out += 4;
        }

    for (unsigned int i = sse_iters * 4; i < num_points; ++i)
        {
            *_out++ = *_in_a++ * (*phase);
            (*phase) *= phase_inc;
        }
}
#endif /* LV_HAVE_SSE2 */

#ifdef LV_HAVE_NEON
#include <arm_neon.h>

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_neon(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    unsigned int i = 0;
    const unsigned int neon_iters = num_points / 4;
    lv_16sc_t tmp16_;
    lv_32fc_t tmp32_;

    const lv_16sc_t* _in = inVector;
    lv_16sc_t* _out = outVector;

    lv_32fc_t ___phase4 = phase_inc * phase_inc * phase_inc * phase_inc;
    float32_t __phase4_real[4] = { lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4), lv_creal(___phase4) };
    float32_t __phase4_imag[4] = { lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4), lv_cimag(___phase4) };

    float32x4_t _phase4_real = vld1q_f32(__phase4_real);
    float32x4_t _phase4_imag = vld1q_f32(__phase4_imag);

    lv_32fc_t phase2 = (lv_32fc_t)(*phase) * phase_inc;
    lv_32fc_t phase3 = phase2 * phase_inc;
    lv_32fc_t phase4 = phase3 * phase_inc;

    float32_t __phase_real[4] = { lv_creal((*phase)), lv_creal(phase2), lv_creal(phase3), lv_creal(phase4) };
    float32_t __phase_imag[4] = { lv_cimag((*phase)), lv_cimag(phase2), lv_cimag(phase3), lv_cimag(phase4) };

    float32x4_t _phase_real = vld1q_f32(__phase_real);
    float32x4_t _phase_imag = vld1q_f32(__phase_imag);

    float32x4_t half = vdupq_n_f32(0.5f);
    int16x4x2_t tmp16;
    int32x4x2_t tmp32i;
    float32x4x2_t tmp32f, tmp_real, tmp_imag;
    float32x4_t sign, PlusHalf, Round;

    if (neon_iters > 0)
        {
            for(; i < neon_iters; ++i)
                {
                    /* load 4 complex numbers (int 16 bits each component) */
                    tmp16 = vld2_s16((int16_t*)_in); _in += 4;

                    /* promote them to int 32 bits */
                    tmp32i.val[0] = vmovl_s16(tmp16.val[0]);
                    tmp32i.val[1] = vmovl_s16(tmp16.val[1]);

                    /* promote them to float 32 bits */
                    tmp32f.val[0] = vcvtq_f32_s32(tmp32i.val[0]);
                    tmp32f.val[1] = vcvtq_f32_s32(tmp32i.val[1]);

                    /* complex multiplication of four complex samples (float 32 bits each component) */
                    tmp_real.val[0] = vmulq_f32(tmp32f.val[0], _phase_real);
                    tmp_real.val[1] = vmulq_f32(tmp32f.val[1], _phase_imag);
                    tmp_imag.val[0] = vmulq_f32(tmp32f.val[0], _phase_imag);
                    tmp_imag.val[1] = vmulq_f32(tmp32f.val[1], _phase_real);

                    tmp32f.val[0] = vsubq_f32(tmp_real.val[0], tmp_real.val[1]);
                    tmp32f.val[1] = vaddq_f32(tmp_imag.val[0], tmp_imag.val[1]);

                    /* downcast results to int32 */
                    /* in __aarch64__ we can do that with vcvtaq_s32_f32(ret1); vcvtaq_s32_f32(ret2); */
                    sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[0]), 31)));
                    PlusHalf = vaddq_f32(tmp32f.val[0], half);
                    Round = vsubq_f32(PlusHalf, sign);
                    tmp32i.val[0] = vcvtq_s32_f32(Round);

                    sign = vcvtq_f32_u32((vshrq_n_u32(vreinterpretq_u32_f32(tmp32f.val[1]), 31)));
                    PlusHalf = vaddq_f32(tmp32f.val[1], half);
                    Round = vsubq_f32(PlusHalf, sign);
                    tmp32i.val[1] = vcvtq_s32_f32(Round);

                    /* downcast results to int16 */
                    tmp16.val[0] = vqmovn_s32(tmp32i.val[0]);
                    tmp16.val[1] = vqmovn_s32(tmp32i.val[1]);

                    /* compute next four phases */
                    tmp_real.val[0] = vmulq_f32(_phase_real, _phase4_real);
                    tmp_real.val[1] = vmulq_f32(_phase_imag, _phase4_imag);
                    tmp_imag.val[0] = vmulq_f32(_phase_real, _phase4_imag);
                    tmp_imag.val[1] = vmulq_f32(_phase_imag, _phase4_real);

                    _phase_real = vsubq_f32(tmp_real.val[0], tmp_real.val[1]);
                    _phase_imag = vaddq_f32(tmp_imag.val[0], tmp_imag.val[1]);

                    /* store the four complex results */
                    vst2_s16((int16_t*)_out, tmp16);
                    _out += 4;
                }
            vst1q_f32((float32_t*)__phase_real, _phase_real);
            vst1q_f32((float32_t*)__phase_imag, _phase_imag);

            (*phase) = lv_cmake((float32_t)__phase_real[3], (float32_t)__phase_imag[3]) * phase_inc;
        }
    for(i = 0; i < neon_iters % 4; ++i)
        {
            tmp16_ = *_in++;
            tmp32_ = lv_cmake((float32_t)lv_creal(tmp16_), (float32_t)lv_cimag(tmp16_)) * (*phase);
            *_out++ = lv_cmake((int16_t)rintf(lv_creal(tmp32_)), (int16_t)rintf(lv_cimag(tmp32_)));
            (*phase) *= phase_inc;
        }
}

#endif /* LV_HAVE_NEON */

#endif /* INCLUDED_volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_H */
