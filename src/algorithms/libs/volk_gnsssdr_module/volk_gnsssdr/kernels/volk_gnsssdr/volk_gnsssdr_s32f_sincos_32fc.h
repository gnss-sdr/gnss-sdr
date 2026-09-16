/*!
 * \file volk_gnsssdr_s32f_sincos_32fc.h
 * \brief VOLK_GNSSSDR kernel: Computes the sine and cosine of a vector of floats.
 * \authors <ul>
 *          <li> Julien Pommier, 2007
 *          <li> Carles Fernandez-Prades, 2016-2026. cfernandez(at)cttc.es
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that computes the sine and cosine of a vector of floats.
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2007  Julien Pommier
 *
 * SPDX-License-Identifier: Zlib
 *
 */


/*!
 * \page volk_gnsssdr_s32f_sincos_32fc
 *
 * \b Overview
 *
 * VOLK_GNSSSDR kernel that computes the sine and cosine with a fixed
 * phase increment \p phase_inc per sample, providing the output in a complex vector (cosine, sine).
 * The phase is accumulated in 32-bit fixed point (2^32 units per turn), so it
 * wraps modulo 2*pi exactly and its resolution does not depend on the initial
 * phase, the phase increment or the number of points. The phase increment is
 * quantized to 2*pi/2^32 rad. The sine and cosine are not IEEE compliant: the
 * maximum absolute error is about 2^-24 per sample.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_s32f_sincos_32fc(lv_32fc_t* out, const float phase_inc, float* phase, unsigned int num_points)
 * \endcode
 *
 * \b Inputs
 * \li phase_inc:      Phase increment per sample, in radians.
 * \li phase:          Pointer to a float containing the initial phase, in radians.
 * \li num_points:     Number of components in \p in to be computed.
 *
 * \b Outputs
 * \li out:            Vector of the form lv_32fc_t out[n] = lv_cmake(cos(in[n]), sin(in[n]))
 * \li phase:          Pointer to a float containing the final phase, in radians, wrapped into [-pi, pi).
 *
 * Adapted from http://gruntthepeon.free.fr/ssemath/sse_mathfun.h, original code from Julien Pommier
 * Based on algorithms from the cephes library https://www.netlib.org/cephes/
 */


#ifndef INCLUDED_volk_gnsssdr_s32f_sincos_32fc_H
#define INCLUDED_volk_gnsssdr_s32f_sincos_32fc_H

#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <math.h>
#include <stdint.h>

/* Phase representation used by all the implementations: unsigned 32-bit
 * fixed point with 2^32 units per turn (2*pi rad). The accumulation of the
 * phase increment wraps modulo 2*pi with the integer arithmetic and does not
 * lose resolution regardless of the number of samples. A single-precision
 * accumulator does: with a large frequency offset (e.g., the carrier offsets
 * of GLONASS FDMA channels) the accumulated phase reached 1e5 rad over a
 * 20 ms block, its resolution dropped to 0.01-0.06 rad, and the phase
 * increment was effectively rounded, shifting the frequency of the generated
 * carrier by hundreds of Hz. The phase increment is quantized to 2*pi/2^32 rad
 * (1.5e-9 rad, i.e., 1.4 mHz at 6 Msps). */
#define VOLK_GNSSSDR_SINCOS_RAD_TO_FIXED (4294967296.0 / 6.283185307179586)
#define VOLK_GNSSSDR_SINCOS_INT32_TO_RAD (3.14159265358979323846f / 2147483648.0f)

static inline uint32_t volk_gnsssdr_sincos_rad_to_fixed(float rad)
{
    /* the 64-bit intermediate keeps the value modulo 2^32 exact for |rad| < 2^31 */
    return (uint32_t)llround((double)rad * VOLK_GNSSSDR_SINCOS_RAD_TO_FIXED);
}

static inline float volk_gnsssdr_sincos_fixed_to_rad(uint32_t fixed)
{
    /* reinterpreting the phase as a signed integer maps it into [-pi, pi) */
    return (float)((int32_t)fixed) * VOLK_GNSSSDR_SINCOS_INT32_TO_RAD;
}


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_s32f_sincos_32fc_a_sse2(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    lv_32fc_t *bPtr = out;

    const unsigned int sse_iters = num_points / 4;
    unsigned int number = 0;
    uint32_t _phase = volk_gnsssdr_sincos_rad_to_fixed(*phase);
    const uint32_t _phase_inc = volk_gnsssdr_sincos_rad_to_fixed(phase_inc);

    __m128 sine, cosine, aux, x;
    __m128 xmm1, xmm2, xmm3 = _mm_setzero_ps(), sign_bit_sin, y;
    __m128i emm0, emm2, emm4, four_phases_reg;

    /* declare some SSE constants */
    static const int _ps_inv_sign_mask[4] = {~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000};
    static const int _ps_sign_mask[4] = {(int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000};

    static const float _ps_cephes_FOPI[4] = {1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516};
    static const int _pi32_1[4] = {1, 1, 1, 1};
    static const int _pi32_inv1[4] = {~1, ~1, ~1, ~1};
    static const int _pi32_2[4] = {2, 2, 2, 2};
    static const int _pi32_4[4] = {4, 4, 4, 4};

    static const float _ps_minus_cephes_DP1[4] = {-0.78515625, -0.78515625, -0.78515625, -0.78515625};
    static const float _ps_minus_cephes_DP2[4] = {-2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4};
    static const float _ps_minus_cephes_DP3[4] = {-3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8};
    static const float _ps_coscof_p0[4] = {2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005};
    static const float _ps_coscof_p1[4] = {-1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003};
    static const float _ps_coscof_p2[4] = {4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002};
    static const float _ps_sincof_p0[4] = {-1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4};
    static const float _ps_sincof_p1[4] = {8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3};
    static const float _ps_sincof_p2[4] = {-1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1};
    static const float _ps_0p5[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    static const float _ps_1[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_int32_to_rad[4] = {VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD};

    /* fixed-point phases (2^32 units per turn): the integer accumulation wraps modulo 2*pi */
    __VOLK_ATTR_ALIGNED(16)
    uint32_t four_phases[4] = {_phase, _phase + _phase_inc, _phase + 2 * _phase_inc, _phase + 3 * _phase_inc};
    __VOLK_ATTR_ALIGNED(16)
    uint32_t four_phases_inc[4] = {4 * _phase_inc, 4 * _phase_inc, 4 * _phase_inc, 4 * _phase_inc};
    four_phases_reg = _mm_load_si128((__m128i *)four_phases);
    const __m128i four_phases_inc_reg = _mm_load_si128((__m128i *)four_phases_inc);

    for (; number < sse_iters; number++)
        {
            /* phase in [-pi, pi) */
            x = _mm_mul_ps(_mm_cvtepi32_ps(four_phases_reg), *(__m128 *)_ps_int32_to_rad);

            sign_bit_sin = x;
            /* take the absolute value */
            x = _mm_and_ps(x, *(__m128 *)_ps_inv_sign_mask);
            /* extract the sign bit (upper one) */
            sign_bit_sin = _mm_and_ps(sign_bit_sin, *(__m128 *)_ps_sign_mask);

            /* scale by 4/Pi */
            y = _mm_mul_ps(x, *(__m128 *)_ps_cephes_FOPI);

            /* store the integer part of y in emm2 */
            emm2 = _mm_cvttps_epi32(y);

            /* j=(j+1) & (~1) (see the cephes sources) */
            emm2 = _mm_add_epi32(emm2, *(__m128i *)_pi32_1);
            emm2 = _mm_and_si128(emm2, *(__m128i *)_pi32_inv1);
            y = _mm_cvtepi32_ps(emm2);

            emm4 = emm2;

            /* get the swap sign flag for the sine */
            emm0 = _mm_and_si128(emm2, *(__m128i *)_pi32_4);
            emm0 = _mm_slli_epi32(emm0, 29);
            __m128 swap_sign_bit_sin = _mm_castsi128_ps(emm0);

            /* get the polynom selection mask for the sine*/
            emm2 = _mm_and_si128(emm2, *(__m128i *)_pi32_2);
            emm2 = _mm_cmpeq_epi32(emm2, _mm_setzero_si128());
            __m128 poly_mask = _mm_castsi128_ps(emm2);

            /* The magic pass: "Extended precision modular arithmetic”
               x = ((x - y * DP1) - y * DP2) - y * DP3; */
            xmm1 = *(__m128 *)_ps_minus_cephes_DP1;
            xmm2 = *(__m128 *)_ps_minus_cephes_DP2;
            xmm3 = *(__m128 *)_ps_minus_cephes_DP3;
            xmm1 = _mm_mul_ps(y, xmm1);
            xmm2 = _mm_mul_ps(y, xmm2);
            xmm3 = _mm_mul_ps(y, xmm3);
            x = _mm_add_ps(x, xmm1);
            x = _mm_add_ps(x, xmm2);
            x = _mm_add_ps(x, xmm3);

            emm4 = _mm_sub_epi32(emm4, *(__m128i *)_pi32_2);
            emm4 = _mm_andnot_si128(emm4, *(__m128i *)_pi32_4);
            emm4 = _mm_slli_epi32(emm4, 29);
            __m128 sign_bit_cos = _mm_castsi128_ps(emm4);

            sign_bit_sin = _mm_xor_ps(sign_bit_sin, swap_sign_bit_sin);

            /* Evaluate the first polynom  (0 <= x <= Pi/4) */
            __m128 z = _mm_mul_ps(x, x);
            y = *(__m128 *)_ps_coscof_p0;

            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, *(__m128 *)_ps_coscof_p1);
            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, *(__m128 *)_ps_coscof_p2);
            y = _mm_mul_ps(y, z);
            y = _mm_mul_ps(y, z);
            __m128 tmp = _mm_mul_ps(z, *(__m128 *)_ps_0p5);
            y = _mm_sub_ps(y, tmp);
            y = _mm_add_ps(y, *(__m128 *)_ps_1);

            /* Evaluate the second polynom  (Pi/4 <= x <= 0) */
            __m128 y2 = *(__m128 *)_ps_sincof_p0;
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, *(__m128 *)_ps_sincof_p1);
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, *(__m128 *)_ps_sincof_p2);
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_mul_ps(y2, x);
            y2 = _mm_add_ps(y2, x);

            /* select the correct result from the two polynoms */
            xmm3 = poly_mask;
            __m128 ysin2 = _mm_and_ps(xmm3, y2);
            __m128 ysin1 = _mm_andnot_ps(xmm3, y);
            y2 = _mm_sub_ps(y2, ysin2);
            y = _mm_sub_ps(y, ysin1);

            xmm1 = _mm_add_ps(ysin1, ysin2);
            xmm2 = _mm_add_ps(y, y2);

            /* update the sign */
            sine = _mm_xor_ps(xmm1, sign_bit_sin);
            cosine = _mm_xor_ps(xmm2, sign_bit_cos);

            /* write the output */
            aux = _mm_unpacklo_ps(cosine, sine);
            _mm_store_ps((float *)bPtr, aux);
            bPtr += 2;
            aux = _mm_unpackhi_ps(cosine, sine);
            _mm_store_ps((float *)bPtr, aux);
            bPtr += 2;

            four_phases_reg = _mm_add_epi32(four_phases_reg, four_phases_inc_reg);
        }

    _phase = (uint32_t)_mm_cvtsi128_si32(four_phases_reg);
    for (number = sse_iters * 4; number < num_points; number++)
        {
            const float x_tail = volk_gnsssdr_sincos_fixed_to_rad(_phase);
            *bPtr++ = lv_cmake((float)cosf(x_tail), (float)sinf(x_tail));
            _phase += _phase_inc;
        }
    (*phase) = volk_gnsssdr_sincos_fixed_to_rad(_phase);
}

#endif /* LV_HAVE_SSE2  */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>

static inline void volk_gnsssdr_s32f_sincos_32fc_u_sse2(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    lv_32fc_t *bPtr = out;

    const unsigned int sse_iters = num_points / 4;
    unsigned int number = 0;

    uint32_t _phase = volk_gnsssdr_sincos_rad_to_fixed(*phase);
    const uint32_t _phase_inc = volk_gnsssdr_sincos_rad_to_fixed(phase_inc);

    __m128 sine, cosine, aux, x;
    __m128 xmm1, xmm2, xmm3 = _mm_setzero_ps(), sign_bit_sin, y;
    __m128i emm0, emm2, emm4, four_phases_reg;

    /* declare some SSE constants */
    __VOLK_ATTR_ALIGNED(16)
    static const int _ps_inv_sign_mask[4] = {~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000};
    __VOLK_ATTR_ALIGNED(16)
    static const int _ps_sign_mask[4] = {(int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000};

    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_cephes_FOPI[4] = {1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516};
    __VOLK_ATTR_ALIGNED(16)
    static const int _pi32_1[4] = {1, 1, 1, 1};
    __VOLK_ATTR_ALIGNED(16)
    static const int _pi32_inv1[4] = {~1, ~1, ~1, ~1};
    __VOLK_ATTR_ALIGNED(16)
    static const int _pi32_2[4] = {2, 2, 2, 2};
    __VOLK_ATTR_ALIGNED(16)
    static const int _pi32_4[4] = {4, 4, 4, 4};

    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_minus_cephes_DP1[4] = {-0.78515625, -0.78515625, -0.78515625, -0.78515625};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_minus_cephes_DP2[4] = {-2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_minus_cephes_DP3[4] = {-3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_coscof_p0[4] = {2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_coscof_p1[4] = {-1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_coscof_p2[4] = {4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_sincof_p0[4] = {-1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_sincof_p1[4] = {8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_sincof_p2[4] = {-1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_0p5[4] = {0.5f, 0.5f, 0.5f, 0.5f};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_1[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    __VOLK_ATTR_ALIGNED(16)
    static const float _ps_int32_to_rad[4] = {VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD};

    /* fixed-point phases (2^32 units per turn): the integer accumulation wraps modulo 2*pi */
    __VOLK_ATTR_ALIGNED(16)
    uint32_t four_phases[4] = {_phase, _phase + _phase_inc, _phase + 2 * _phase_inc, _phase + 3 * _phase_inc};
    __VOLK_ATTR_ALIGNED(16)
    uint32_t four_phases_inc[4] = {4 * _phase_inc, 4 * _phase_inc, 4 * _phase_inc, 4 * _phase_inc};
    four_phases_reg = _mm_load_si128((__m128i *)four_phases);
    const __m128i four_phases_inc_reg = _mm_load_si128((__m128i *)four_phases_inc);

    for (; number < sse_iters; number++)
        {
            /* phase in [-pi, pi) */
            x = _mm_mul_ps(_mm_cvtepi32_ps(four_phases_reg), *(__m128 *)_ps_int32_to_rad);

            sign_bit_sin = x;
            /* take the absolute value */
            x = _mm_and_ps(x, *(__m128 *)_ps_inv_sign_mask);
            /* extract the sign bit (upper one) */
            sign_bit_sin = _mm_and_ps(sign_bit_sin, *(__m128 *)_ps_sign_mask);

            /* scale by 4/Pi */
            y = _mm_mul_ps(x, *(__m128 *)_ps_cephes_FOPI);

            /* store the integer part of y in emm2 */
            emm2 = _mm_cvttps_epi32(y);

            /* j=(j+1) & (~1) (see the cephes sources) */
            emm2 = _mm_add_epi32(emm2, *(__m128i *)_pi32_1);
            emm2 = _mm_and_si128(emm2, *(__m128i *)_pi32_inv1);
            y = _mm_cvtepi32_ps(emm2);

            emm4 = emm2;

            /* get the swap sign flag for the sine */
            emm0 = _mm_and_si128(emm2, *(__m128i *)_pi32_4);
            emm0 = _mm_slli_epi32(emm0, 29);
            __m128 swap_sign_bit_sin = _mm_castsi128_ps(emm0);

            /* get the polynom selection mask for the sine*/
            emm2 = _mm_and_si128(emm2, *(__m128i *)_pi32_2);
            emm2 = _mm_cmpeq_epi32(emm2, _mm_setzero_si128());
            __m128 poly_mask = _mm_castsi128_ps(emm2);

            /* The magic pass: "Extended precision modular arithmetic”
               x = ((x - y * DP1) - y * DP2) - y * DP3; */
            xmm1 = *(__m128 *)_ps_minus_cephes_DP1;
            xmm2 = *(__m128 *)_ps_minus_cephes_DP2;
            xmm3 = *(__m128 *)_ps_minus_cephes_DP3;
            xmm1 = _mm_mul_ps(y, xmm1);
            xmm2 = _mm_mul_ps(y, xmm2);
            xmm3 = _mm_mul_ps(y, xmm3);
            x = _mm_add_ps(x, xmm1);
            x = _mm_add_ps(x, xmm2);
            x = _mm_add_ps(x, xmm3);

            emm4 = _mm_sub_epi32(emm4, *(__m128i *)_pi32_2);
            emm4 = _mm_andnot_si128(emm4, *(__m128i *)_pi32_4);
            emm4 = _mm_slli_epi32(emm4, 29);
            __m128 sign_bit_cos = _mm_castsi128_ps(emm4);

            sign_bit_sin = _mm_xor_ps(sign_bit_sin, swap_sign_bit_sin);

            /* Evaluate the first polynom  (0 <= x <= Pi/4) */
            __m128 z = _mm_mul_ps(x, x);
            y = *(__m128 *)_ps_coscof_p0;

            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, *(__m128 *)_ps_coscof_p1);
            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, *(__m128 *)_ps_coscof_p2);
            y = _mm_mul_ps(y, z);
            y = _mm_mul_ps(y, z);
            __m128 tmp = _mm_mul_ps(z, *(__m128 *)_ps_0p5);
            y = _mm_sub_ps(y, tmp);
            y = _mm_add_ps(y, *(__m128 *)_ps_1);

            /* Evaluate the second polynom  (Pi/4 <= x <= 0) */
            __m128 y2 = *(__m128 *)_ps_sincof_p0;
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, *(__m128 *)_ps_sincof_p1);
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, *(__m128 *)_ps_sincof_p2);
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_mul_ps(y2, x);
            y2 = _mm_add_ps(y2, x);

            /* select the correct result from the two polynoms */
            xmm3 = poly_mask;
            __m128 ysin2 = _mm_and_ps(xmm3, y2);
            __m128 ysin1 = _mm_andnot_ps(xmm3, y);
            y2 = _mm_sub_ps(y2, ysin2);
            y = _mm_sub_ps(y, ysin1);

            xmm1 = _mm_add_ps(ysin1, ysin2);
            xmm2 = _mm_add_ps(y, y2);

            /* update the sign */
            sine = _mm_xor_ps(xmm1, sign_bit_sin);
            cosine = _mm_xor_ps(xmm2, sign_bit_cos);

            /* write the output */
            aux = _mm_unpacklo_ps(cosine, sine);
            _mm_storeu_ps((float *)bPtr, aux);
            bPtr += 2;
            aux = _mm_unpackhi_ps(cosine, sine);
            _mm_storeu_ps((float *)bPtr, aux);
            bPtr += 2;

            four_phases_reg = _mm_add_epi32(four_phases_reg, four_phases_inc_reg);
        }

    _phase = (uint32_t)_mm_cvtsi128_si32(four_phases_reg);
    for (number = sse_iters * 4; number < num_points; number++)
        {
            const float x_tail = volk_gnsssdr_sincos_fixed_to_rad(_phase);
            *bPtr++ = lv_cmake((float)cosf(x_tail), (float)sinf(x_tail));
            _phase += _phase_inc;
        }
    (*phase) = volk_gnsssdr_sincos_fixed_to_rad(_phase);
}

#endif /* LV_HAVE_SSE2  */


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_s32f_sincos_32fc_generic(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    uint32_t _phase = volk_gnsssdr_sincos_rad_to_fixed(*phase);
    const uint32_t _phase_inc = volk_gnsssdr_sincos_rad_to_fixed(phase_inc);
    unsigned int i;
    for (i = 0; i < num_points; i++)
        {
            const float x = volk_gnsssdr_sincos_fixed_to_rad(_phase);
            *out++ = lv_cmake((float)cosf(x), (float)sinf(x));
            _phase += _phase_inc;
        }
    (*phase) = volk_gnsssdr_sincos_fixed_to_rad(_phase);
}

#endif /* LV_HAVE_GENERIC  */


#ifdef LV_HAVE_GENERIC
#include <volk_gnsssdr/volk_gnsssdr_sine_table.h>
#include <stdint.h>
static inline void volk_gnsssdr_s32f_sincos_32fc_generic_fxpt(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    float s, c;
    unsigned int i;
    int32_t x, sin_index, cos_index;
    const int32_t bitlength = 32;
    const int32_t Nbits = 10;
    const int32_t diffbits = bitlength - Nbits;
    uint32_t ux;
    uint32_t _phase = volk_gnsssdr_sincos_rad_to_fixed(*phase);
    const uint32_t _phase_inc = volk_gnsssdr_sincos_rad_to_fixed(phase_inc);
    for (i = 0; i < num_points; i++)
        {
            x = (int32_t)_phase;

            ux = x;
            sin_index = ux >> diffbits;
            s = sine_table_10bits[sin_index][0] * (ux >> 1) + sine_table_10bits[sin_index][1];

            ux = x + 0x40000000;
            cos_index = ux >> diffbits;
            c = sine_table_10bits[cos_index][0] * (ux >> 1) + sine_table_10bits[cos_index][1];

            *out++ = lv_cmake((float)c, (float)s);
            _phase += _phase_inc;
        }
    (*phase) = volk_gnsssdr_sincos_fixed_to_rad(_phase);
}

#endif /* LV_HAVE_GENERIC  */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_s32f_sincos_32fc_a_avx2(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    lv_32fc_t *bPtr = out;

    const unsigned int avx_iters = num_points / 8;
    unsigned int number = 0;

    uint32_t _phase = volk_gnsssdr_sincos_rad_to_fixed(*phase);
    const uint32_t _phase_inc = volk_gnsssdr_sincos_rad_to_fixed(phase_inc);

    __m256 sine, cosine, x;
    __m256 xmm1, xmm2, xmm3 = _mm256_setzero_ps(), sign_bit_sin, y;
    __m256i emm0, emm2, emm4, eight_phases_reg;
    __m128 aux, c1, s1;

    /* declare some AXX2 constants */
    __VOLK_ATTR_ALIGNED(32)
    static const int _ps_inv_sign_mask[8] = {~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000};
    __VOLK_ATTR_ALIGNED(32)
    static const int _ps_sign_mask[8] = {(int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000};

    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_cephes_FOPI[8] = {1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516};
    __VOLK_ATTR_ALIGNED(32)
    static const int _pi32_1[8] = {1, 1, 1, 1, 1, 1, 1, 1};
    __VOLK_ATTR_ALIGNED(32)
    static const int _pi32_inv1[8] = {~1, ~1, ~1, ~1, ~1, ~1, ~1, ~1};
    __VOLK_ATTR_ALIGNED(32)
    static const int _pi32_2[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    __VOLK_ATTR_ALIGNED(32)
    static const int _pi32_4[8] = {4, 4, 4, 4, 4, 4, 4, 4};

    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_minus_cephes_DP1[8] = {-0.78515625, -0.78515625, -0.78515625, -0.78515625, -0.78515625, -0.78515625, -0.78515625, -0.78515625};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_minus_cephes_DP2[8] = {-2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_minus_cephes_DP3[8] = {-3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_coscof_p0[8] = {2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_coscof_p1[8] = {-1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_coscof_p2[8] = {4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_sincof_p0[8] = {-1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_sincof_p1[8] = {8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_sincof_p2[8] = {-1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_0p5[8] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_1[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_int32_to_rad[8] = {VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD};

    /* fixed-point phases (2^32 units per turn): the integer accumulation wraps modulo 2*pi */
    __VOLK_ATTR_ALIGNED(32)
    uint32_t eight_phases[8] = {_phase, _phase + _phase_inc, _phase + 2 * _phase_inc, _phase + 3 * _phase_inc, _phase + 4 * _phase_inc, _phase + 5 * _phase_inc, _phase + 6 * _phase_inc, _phase + 7 * _phase_inc};
    __VOLK_ATTR_ALIGNED(32)
    uint32_t eight_phases_inc[8] = {8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc};
    eight_phases_reg = _mm256_load_si256((__m256i *)eight_phases);
    const __m256i eight_phases_inc_reg = _mm256_load_si256((__m256i *)eight_phases_inc);

    for (; number < avx_iters; number++)
        {
            /* phase in [-pi, pi) */
            x = _mm256_mul_ps(_mm256_cvtepi32_ps(eight_phases_reg), *(__m256 *)_ps_int32_to_rad);

            sign_bit_sin = x;
            /* take the absolute value */
            x = _mm256_and_ps(x, *(__m256 *)_ps_inv_sign_mask);
            /* extract the sign bit (upper one) */
            sign_bit_sin = _mm256_and_ps(sign_bit_sin, *(__m256 *)_ps_sign_mask);

            /* scale by 4/Pi */
            y = _mm256_mul_ps(x, *(__m256 *)_ps_cephes_FOPI);

            /* store the integer part of y in emm2 */
            emm2 = _mm256_cvttps_epi32(y);

            /* j=(j+1) & (~1) (see the cephes sources) */
            emm2 = _mm256_add_epi32(emm2, *(__m256i *)_pi32_1);
            emm2 = _mm256_and_si256(emm2, *(__m256i *)_pi32_inv1);
            y = _mm256_cvtepi32_ps(emm2);

            emm4 = emm2;

            /* get the swap sign flag for the sine */
            emm0 = _mm256_and_si256(emm2, *(__m256i *)_pi32_4);
            emm0 = _mm256_slli_epi32(emm0, 29);
            __m256 swap_sign_bit_sin = _mm256_castsi256_ps(emm0);

            /* get the polynom selection mask for the sine*/
            emm2 = _mm256_and_si256(emm2, *(__m256i *)_pi32_2);
            emm2 = _mm256_cmpeq_epi32(emm2, _mm256_setzero_si256());
            __m256 poly_mask = _mm256_castsi256_ps(emm2);

            /* The magic pass: "Extended precision modular arithmetic”
               x = ((x - y * DP1) - y * DP2) - y * DP3; */
            xmm1 = *(__m256 *)_ps_minus_cephes_DP1;
            xmm2 = *(__m256 *)_ps_minus_cephes_DP2;
            xmm3 = *(__m256 *)_ps_minus_cephes_DP3;
            xmm1 = _mm256_mul_ps(y, xmm1);
            xmm2 = _mm256_mul_ps(y, xmm2);
            xmm3 = _mm256_mul_ps(y, xmm3);
            x = _mm256_add_ps(x, xmm1);
            x = _mm256_add_ps(x, xmm2);
            x = _mm256_add_ps(x, xmm3);

            emm4 = _mm256_sub_epi32(emm4, *(__m256i *)_pi32_2);
            emm4 = _mm256_andnot_si256(emm4, *(__m256i *)_pi32_4);
            emm4 = _mm256_slli_epi32(emm4, 29);
            __m256 sign_bit_cos = _mm256_castsi256_ps(emm4);

            sign_bit_sin = _mm256_xor_ps(sign_bit_sin, swap_sign_bit_sin);

            /* Evaluate the first polynom  (0 <= x <= Pi/4) */
            __m256 z = _mm256_mul_ps(x, x);
            y = *(__m256 *)_ps_coscof_p0;

            y = _mm256_mul_ps(y, z);
            y = _mm256_add_ps(y, *(__m256 *)_ps_coscof_p1);
            y = _mm256_mul_ps(y, z);
            y = _mm256_add_ps(y, *(__m256 *)_ps_coscof_p2);
            y = _mm256_mul_ps(y, z);
            y = _mm256_mul_ps(y, z);
            __m256 tmp = _mm256_mul_ps(z, *(__m256 *)_ps_0p5);
            y = _mm256_sub_ps(y, tmp);
            y = _mm256_add_ps(y, *(__m256 *)_ps_1);

            /* Evaluate the second polynom  (Pi/4 <= x <= 0) */
            __m256 y2 = *(__m256 *)_ps_sincof_p0;
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_add_ps(y2, *(__m256 *)_ps_sincof_p1);
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_add_ps(y2, *(__m256 *)_ps_sincof_p2);
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_mul_ps(y2, x);
            y2 = _mm256_add_ps(y2, x);

            /* select the correct result from the two polynoms */
            xmm3 = poly_mask;
            __m256 ysin2 = _mm256_and_ps(xmm3, y2);
            __m256 ysin1 = _mm256_andnot_ps(xmm3, y);
            y2 = _mm256_sub_ps(y2, ysin2);
            y = _mm256_sub_ps(y, ysin1);

            xmm1 = _mm256_add_ps(ysin1, ysin2);
            xmm2 = _mm256_add_ps(y, y2);

            /* update the sign */
            sine = _mm256_xor_ps(xmm1, sign_bit_sin);
            cosine = _mm256_xor_ps(xmm2, sign_bit_cos);

            /* write the output */
            s1 = _mm256_extractf128_ps(sine, 0);
            c1 = _mm256_extractf128_ps(cosine, 0);
            aux = _mm_unpacklo_ps(c1, s1);
            _mm_store_ps((float *)bPtr, aux);
            bPtr += 2;
            aux = _mm_unpackhi_ps(c1, s1);
            _mm_store_ps((float *)bPtr, aux);
            bPtr += 2;
            s1 = _mm256_extractf128_ps(sine, 1);
            c1 = _mm256_extractf128_ps(cosine, 1);
            aux = _mm_unpacklo_ps(c1, s1);
            _mm_store_ps((float *)bPtr, aux);
            bPtr += 2;
            aux = _mm_unpackhi_ps(c1, s1);
            _mm_store_ps((float *)bPtr, aux);
            bPtr += 2;

            eight_phases_reg = _mm256_add_epi32(eight_phases_reg, eight_phases_inc_reg);
        }

    _phase = (uint32_t)_mm_cvtsi128_si32(_mm256_castsi256_si128(eight_phases_reg));
    for (number = avx_iters * 8; number < num_points; number++)
        {
            const float x_tail = volk_gnsssdr_sincos_fixed_to_rad(_phase);
            out[number] = lv_cmake((float)cosf(x_tail), (float)sinf(x_tail));
            _phase += _phase_inc;
        }
    (*phase) = volk_gnsssdr_sincos_fixed_to_rad(_phase);
}

#endif /* LV_HAVE_AVX2  */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>

static inline void volk_gnsssdr_s32f_sincos_32fc_u_avx2(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    lv_32fc_t *bPtr = out;

    const unsigned int avx_iters = num_points / 8;
    unsigned int number = 0;

    uint32_t _phase = volk_gnsssdr_sincos_rad_to_fixed(*phase);
    const uint32_t _phase_inc = volk_gnsssdr_sincos_rad_to_fixed(phase_inc);

    __m256 sine, cosine, x;
    __m256 xmm1, xmm2, xmm3 = _mm256_setzero_ps(), sign_bit_sin, y;
    __m256i emm0, emm2, emm4, eight_phases_reg;
    __m128 aux, c1, s1;

    /* declare some AXX2 constants */
    __VOLK_ATTR_ALIGNED(32)
    static const int _ps_inv_sign_mask[8] = {~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000, ~0x80000000};
    __VOLK_ATTR_ALIGNED(32)
    static const int _ps_sign_mask[8] = {(int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000, (int)0x80000000};

    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_cephes_FOPI[8] = {1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516, 1.27323954473516};
    __VOLK_ATTR_ALIGNED(32)
    static const int _pi32_1[8] = {1, 1, 1, 1, 1, 1, 1, 1};
    __VOLK_ATTR_ALIGNED(32)
    static const int _pi32_inv1[8] = {~1, ~1, ~1, ~1, ~1, ~1, ~1, ~1};
    __VOLK_ATTR_ALIGNED(32)
    static const int _pi32_2[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    __VOLK_ATTR_ALIGNED(32)
    static const int _pi32_4[8] = {4, 4, 4, 4, 4, 4, 4, 4};

    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_minus_cephes_DP1[8] = {-0.78515625, -0.78515625, -0.78515625, -0.78515625, -0.78515625, -0.78515625, -0.78515625, -0.78515625};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_minus_cephes_DP2[8] = {-2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4, -2.4187564849853515625e-4};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_minus_cephes_DP3[8] = {-3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8, -3.77489497744594108e-8};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_coscof_p0[8] = {2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005, 2.443315711809948E-005};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_coscof_p1[8] = {-1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003, -1.388731625493765E-003};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_coscof_p2[8] = {4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002, 4.166664568298827E-002};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_sincof_p0[8] = {-1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4, -1.9515295891E-4};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_sincof_p1[8] = {8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3, 8.3321608736E-3};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_sincof_p2[8] = {-1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1, -1.6666654611E-1};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_0p5[8] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_1[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    __VOLK_ATTR_ALIGNED(32)
    static const float _ps_int32_to_rad[8] = {VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD, VOLK_GNSSSDR_SINCOS_INT32_TO_RAD};

    /* fixed-point phases (2^32 units per turn): the integer accumulation wraps modulo 2*pi */
    __VOLK_ATTR_ALIGNED(32)
    uint32_t eight_phases[8] = {_phase, _phase + _phase_inc, _phase + 2 * _phase_inc, _phase + 3 * _phase_inc, _phase + 4 * _phase_inc, _phase + 5 * _phase_inc, _phase + 6 * _phase_inc, _phase + 7 * _phase_inc};
    __VOLK_ATTR_ALIGNED(32)
    uint32_t eight_phases_inc[8] = {8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc, 8 * _phase_inc};
    eight_phases_reg = _mm256_load_si256((__m256i *)eight_phases);
    const __m256i eight_phases_inc_reg = _mm256_load_si256((__m256i *)eight_phases_inc);

    for (; number < avx_iters; number++)
        {
            /* phase in [-pi, pi) */
            x = _mm256_mul_ps(_mm256_cvtepi32_ps(eight_phases_reg), *(__m256 *)_ps_int32_to_rad);

            sign_bit_sin = x;
            /* take the absolute value */
            x = _mm256_and_ps(x, *(__m256 *)_ps_inv_sign_mask);
            /* extract the sign bit (upper one) */
            sign_bit_sin = _mm256_and_ps(sign_bit_sin, *(__m256 *)_ps_sign_mask);

            /* scale by 4/Pi */
            y = _mm256_mul_ps(x, *(__m256 *)_ps_cephes_FOPI);

            /* store the integer part of y in emm2 */
            emm2 = _mm256_cvttps_epi32(y);

            /* j=(j+1) & (~1) (see the cephes sources) */
            emm2 = _mm256_add_epi32(emm2, *(__m256i *)_pi32_1);
            emm2 = _mm256_and_si256(emm2, *(__m256i *)_pi32_inv1);
            y = _mm256_cvtepi32_ps(emm2);

            emm4 = emm2;

            /* get the swap sign flag for the sine */
            emm0 = _mm256_and_si256(emm2, *(__m256i *)_pi32_4);
            emm0 = _mm256_slli_epi32(emm0, 29);
            __m256 swap_sign_bit_sin = _mm256_castsi256_ps(emm0);

            /* get the polynom selection mask for the sine*/
            emm2 = _mm256_and_si256(emm2, *(__m256i *)_pi32_2);
            emm2 = _mm256_cmpeq_epi32(emm2, _mm256_setzero_si256());
            __m256 poly_mask = _mm256_castsi256_ps(emm2);

            /* The magic pass: "Extended precision modular arithmetic”
               x = ((x - y * DP1) - y * DP2) - y * DP3; */
            xmm1 = *(__m256 *)_ps_minus_cephes_DP1;
            xmm2 = *(__m256 *)_ps_minus_cephes_DP2;
            xmm3 = *(__m256 *)_ps_minus_cephes_DP3;
            xmm1 = _mm256_mul_ps(y, xmm1);
            xmm2 = _mm256_mul_ps(y, xmm2);
            xmm3 = _mm256_mul_ps(y, xmm3);
            x = _mm256_add_ps(x, xmm1);
            x = _mm256_add_ps(x, xmm2);
            x = _mm256_add_ps(x, xmm3);

            emm4 = _mm256_sub_epi32(emm4, *(__m256i *)_pi32_2);
            emm4 = _mm256_andnot_si256(emm4, *(__m256i *)_pi32_4);
            emm4 = _mm256_slli_epi32(emm4, 29);
            __m256 sign_bit_cos = _mm256_castsi256_ps(emm4);

            sign_bit_sin = _mm256_xor_ps(sign_bit_sin, swap_sign_bit_sin);

            /* Evaluate the first polynom  (0 <= x <= Pi/4) */
            __m256 z = _mm256_mul_ps(x, x);
            y = *(__m256 *)_ps_coscof_p0;

            y = _mm256_mul_ps(y, z);
            y = _mm256_add_ps(y, *(__m256 *)_ps_coscof_p1);
            y = _mm256_mul_ps(y, z);
            y = _mm256_add_ps(y, *(__m256 *)_ps_coscof_p2);
            y = _mm256_mul_ps(y, z);
            y = _mm256_mul_ps(y, z);
            __m256 tmp = _mm256_mul_ps(z, *(__m256 *)_ps_0p5);
            y = _mm256_sub_ps(y, tmp);
            y = _mm256_add_ps(y, *(__m256 *)_ps_1);

            /* Evaluate the second polynom  (Pi/4 <= x <= 0) */
            __m256 y2 = *(__m256 *)_ps_sincof_p0;
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_add_ps(y2, *(__m256 *)_ps_sincof_p1);
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_add_ps(y2, *(__m256 *)_ps_sincof_p2);
            y2 = _mm256_mul_ps(y2, z);
            y2 = _mm256_mul_ps(y2, x);
            y2 = _mm256_add_ps(y2, x);

            /* select the correct result from the two polynoms */
            xmm3 = poly_mask;
            __m256 ysin2 = _mm256_and_ps(xmm3, y2);
            __m256 ysin1 = _mm256_andnot_ps(xmm3, y);
            y2 = _mm256_sub_ps(y2, ysin2);
            y = _mm256_sub_ps(y, ysin1);

            xmm1 = _mm256_add_ps(ysin1, ysin2);
            xmm2 = _mm256_add_ps(y, y2);

            /* update the sign */
            sine = _mm256_xor_ps(xmm1, sign_bit_sin);
            cosine = _mm256_xor_ps(xmm2, sign_bit_cos);

            /* write the output */
            s1 = _mm256_extractf128_ps(sine, 0);
            c1 = _mm256_extractf128_ps(cosine, 0);
            aux = _mm_unpacklo_ps(c1, s1);
            _mm_storeu_ps((float *)bPtr, aux);
            bPtr += 2;
            aux = _mm_unpackhi_ps(c1, s1);
            _mm_storeu_ps((float *)bPtr, aux);
            bPtr += 2;
            s1 = _mm256_extractf128_ps(sine, 1);
            c1 = _mm256_extractf128_ps(cosine, 1);
            aux = _mm_unpacklo_ps(c1, s1);
            _mm_storeu_ps((float *)bPtr, aux);
            bPtr += 2;
            aux = _mm_unpackhi_ps(c1, s1);
            _mm_storeu_ps((float *)bPtr, aux);
            bPtr += 2;

            eight_phases_reg = _mm256_add_epi32(eight_phases_reg, eight_phases_inc_reg);
        }

    _phase = (uint32_t)_mm_cvtsi128_si32(_mm256_castsi256_si128(eight_phases_reg));
    for (number = avx_iters * 8; number < num_points; number++)
        {
            const float x_tail = volk_gnsssdr_sincos_fixed_to_rad(_phase);
            out[number] = lv_cmake((float)cosf(x_tail), (float)sinf(x_tail));
            _phase += _phase_inc;
        }
    (*phase) = volk_gnsssdr_sincos_fixed_to_rad(_phase);
}

#endif /* LV_HAVE_AVX2  */


#ifdef LV_HAVE_NEON
#include <arm_neon.h>

static inline void volk_gnsssdr_s32f_sincos_32fc_neon(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    lv_32fc_t *bPtr = out;
    const unsigned int neon_iters = num_points / 4;
    uint32_t _phase = volk_gnsssdr_sincos_rad_to_fixed(*phase);
    const uint32_t _phase_inc = volk_gnsssdr_sincos_rad_to_fixed(phase_inc);

    /* fixed-point phases (2^32 units per turn): the integer accumulation wraps modulo 2*pi */
    __VOLK_ATTR_ALIGNED(16)
    uint32_t four_phases[4] = {_phase, _phase + _phase_inc, _phase + 2 * _phase_inc, _phase + 3 * _phase_inc};
    const uint32_t four_inc = 4 * _phase_inc;
    __VOLK_ATTR_ALIGNED(16)
    uint32_t four_phases_inc[4] = {four_inc, four_inc, four_inc, four_inc};

    uint32x4_t four_phases_reg = vld1q_u32(four_phases);
    const uint32x4_t four_phases_inc_reg = vld1q_u32(four_phases_inc);

    const float32_t c_minus_cephes_DP1 = -0.78515625;
    const float32_t c_minus_cephes_DP2 = -2.4187564849853515625e-4;
    const float32_t c_minus_cephes_DP3 = -3.77489497744594108e-8;
    const float32_t c_sincof_p0 = -1.9515295891E-4;
    const float32_t c_sincof_p1 = 8.3321608736E-3;
    const float32_t c_sincof_p2 = -1.6666654611E-1;
    const float32_t c_coscof_p0 = 2.443315711809948E-005;
    const float32_t c_coscof_p1 = -1.388731625493765E-003;
    const float32_t c_coscof_p2 = 4.166664568298827E-002;
    const float32_t c_cephes_FOPI = 1.27323954473516;
    const float32_t c_int32_to_rad = VOLK_GNSSSDR_SINCOS_INT32_TO_RAD;

    unsigned int number = 0;

    float32x4_t x, xmm1, xmm2, xmm3, y, y1, y2, ys, yc, z;
    float32x4x2_t result;

    uint32x4_t emm2, poly_mask, sign_mask_sin, sign_mask_cos;

    for (; number < neon_iters; number++)
        {
            /* phase in [-pi, pi) */
            x = vmulq_n_f32(vcvtq_f32_s32(vreinterpretq_s32_u32(four_phases_reg)), c_int32_to_rad);

            sign_mask_sin = vcltq_f32(x, vdupq_n_f32(0));
            x = vabsq_f32(x);

            /* scale by 4/Pi */
            y = vmulq_f32(x, vdupq_n_f32(c_cephes_FOPI));

            /* store the integer part of y in mm0 */
            emm2 = vcvtq_u32_f32(y);
            /* j=(j+1) & (~1) (see the cephes sources) */
            emm2 = vaddq_u32(emm2, vdupq_n_u32(1));
            emm2 = vandq_u32(emm2, vdupq_n_u32(~1));
            y = vcvtq_f32_u32(emm2);

            /* get the polynom selection mask
                    there is one polynom for 0 <= x <= Pi/4
                    and another one for Pi/4<x<=Pi/2

                    Both branches will be computed.
             */
            poly_mask = vtstq_u32(emm2, vdupq_n_u32(2));

            /* The magic pass: "Extended precision modular arithmetic"
                    x = ((x - y * DP1) - y * DP2) - y * DP3; */
            xmm1 = vmulq_n_f32(y, c_minus_cephes_DP1);
            xmm2 = vmulq_n_f32(y, c_minus_cephes_DP2);
            xmm3 = vmulq_n_f32(y, c_minus_cephes_DP3);
            x = vaddq_f32(x, xmm1);
            x = vaddq_f32(x, xmm2);
            x = vaddq_f32(x, xmm3);

            sign_mask_sin = veorq_u32(sign_mask_sin, vtstq_u32(emm2, vdupq_n_u32(4)));
            sign_mask_cos = vtstq_u32(vsubq_u32(emm2, vdupq_n_u32(2)), vdupq_n_u32(4));

            /* Evaluate the first polynom  (0 <= x <= Pi/4) in y1,
                    and the second polynom      (Pi/4 <= x <= 0) in y2 */
            z = vmulq_f32(x, x);

            y1 = vmulq_n_f32(z, c_coscof_p0);
            y2 = vmulq_n_f32(z, c_sincof_p0);
            y1 = vaddq_f32(y1, vdupq_n_f32(c_coscof_p1));
            y2 = vaddq_f32(y2, vdupq_n_f32(c_sincof_p1));
            y1 = vmulq_f32(y1, z);
            y2 = vmulq_f32(y2, z);
            y1 = vaddq_f32(y1, vdupq_n_f32(c_coscof_p2));
            y2 = vaddq_f32(y2, vdupq_n_f32(c_sincof_p2));
            y1 = vmulq_f32(y1, z);
            y2 = vmulq_f32(y2, z);
            y1 = vmulq_f32(y1, z);
            y2 = vmulq_f32(y2, x);
            y1 = vsubq_f32(y1, vmulq_f32(z, vdupq_n_f32(0.5f)));
            y2 = vaddq_f32(y2, x);
            y1 = vaddq_f32(y1, vdupq_n_f32(1));

            /* select the correct result from the two polynoms */
            ys = vbslq_f32(poly_mask, y1, y2);
            yc = vbslq_f32(poly_mask, y2, y1);
            result.val[1] = vbslq_f32(sign_mask_sin, vnegq_f32(ys), ys);
            result.val[0] = vbslq_f32(sign_mask_cos, yc, vnegq_f32(yc));

            vst2q_f32((float32_t *)bPtr, result);
            bPtr += 4;

            four_phases_reg = vaddq_u32(four_phases_reg, four_phases_inc_reg);
        }

    _phase = vgetq_lane_u32(four_phases_reg, 0);
    for (number = neon_iters * 4; number < num_points; number++)
        {
            const float x_tail = volk_gnsssdr_sincos_fixed_to_rad(_phase);
            *bPtr++ = lv_cmake((float)cosf(x_tail), (float)sinf(x_tail));
            _phase += _phase_inc;
        }
    (*phase) = volk_gnsssdr_sincos_fixed_to_rad(_phase);
}

#endif /* LV_HAVE_NEON  */


#ifdef LV_HAVE_RVV
#include <riscv_vector.h>

// Reverse-engineered from NEON implementation
static inline void volk_gnsssdr_s32f_sincos_32fc_rvv(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    // Copied from other implementations, specifically NEON
    const float c_minus_cephes_DP1 = -0.78515625;
    const float c_minus_cephes_DP2 = -2.4187564849853515625e-4;
    const float c_minus_cephes_DP3 = -3.77489497744594108e-8;
    const float c_sincof_p0 = -1.9515295891E-4;
    const float c_sincof_p1 = 8.3321608736E-3;
    const float c_sincof_p2 = -1.6666654611E-1;
    const float c_coscof_p0 = 2.443315711809948E-005;
    const float c_coscof_p1 = -1.388731625493765E-003;
    const float c_coscof_p2 = 4.166664568298827E-002;
    const float c_cephes_FOPI = 1.27323954473516;
    const float c_int32_to_rad = VOLK_GNSSSDR_SINCOS_INT32_TO_RAD;

    size_t n = num_points;

    // Fixed-point phase (2^32 units per turn): the integer accumulation
    // wraps modulo 2*pi and does not lose resolution
    uint32_t phase_fixed = volk_gnsssdr_sincos_rad_to_fixed(*phase);
    const uint32_t phase_inc_fixed = volk_gnsssdr_sincos_rad_to_fixed(phase_inc);

    // Initialize pointers to keep track as stripmine
    float *outPtr = (float *)out;

    for (size_t vl; n > 0; n -= vl, outPtr += vl * 2)
        {
            // Record how many elements will actually be processed
            vl = __riscv_vsetvl_e32m4(n);

            // phaseFixed[i] = phaseFixed + i * phaseIncFixed (mod 2^32)
            vuint32m4_t iterVal = __riscv_vid_v_u32m4(vl);
            vuint32m4_t phaseFixedVal = __riscv_vmacc_vx_u32m4(__riscv_vmv_v_x_u32m4(phase_fixed, vl), phase_inc_fixed, iterVal, vl);

            // phase[i] = (float)(int32)phaseFixed[i] * (pi / 2^31), in [-pi, pi)
            vfloat32m4_t phaseVal = __riscv_vfmul_vf_f32m4(__riscv_vfcvt_f_x_v_f32m4(__riscv_vreinterpret_v_u32m4_i32m4(phaseFixedVal), vl), c_int32_to_rad, vl);

            // Save initial signs
            // signMask[i] = phase[i] < 0
            vbool8_t signMask = __riscv_vmflt_vf_f32m4_b8(phaseVal, (float)0, vl);

            // x[i] = |phase[i]|
            vfloat32m4_t xVal = __riscv_vfabs_v_f32m4(phaseVal, vl);

            // y[i] = (4/PI) * x[i]
            vfloat32m4_t yVal = __riscv_vfmul_vf_f32m4(xVal, c_cephes_FOPI, vl);

            // Quantize (reduce) y into discrete chunks to approximate into,
            // and use the integer version to do some neat bit masking in order
            // to encode sin/cos signs
            // reduced[i] = ( ((unsigned int) y[i] + 1) / 2 ) * 2 = ((unsigned int) y[i] + 1) & ~1
            vuint32m4_t reducedVal = __riscv_vfcvt_xu_f_v_u32m4(yVal, vl);
            reducedVal = __riscv_vadd_vx_u32m4(reducedVal, 1, vl);
            reducedVal = __riscv_vand_vx_u32m4(reducedVal, ~1, vl);

            // Save which polynomial should be used
            // polyMask[i] = reduced[i] & 2 != 0
            vuint32m4_t polyTempVal = __riscv_vand_vx_u32m4(reducedVal, 2, vl);
            vbool8_t polyMask = __riscv_vmsne_vx_u32m4_b8(polyTempVal, 0, vl);

            // Save sign value for sin, cos, encoded within LSB 3:1
            // sinSignMask[i] = signMask[i] ^ ( reduced[i] & 4 != 0 )
            vuint32m4_t sinSignTempVal = __riscv_vand_vx_u32m4(reducedVal, 4, vl);
            vbool8_t sinSignMask = __riscv_vmsne_vx_u32m4_b8(sinSignTempVal, 0, vl);
            sinSignMask = __riscv_vmxor_mm_b8(signMask, sinSignMask, vl);

            // Encoded the opposite to sinSignMask, i.e. 0 is positive for cosSignMask
            // cosSignMask[i] = ( reduced[i] - 2 ) & 4 != 0
            vuint32m4_t cosSignTempVal = __riscv_vsub_vx_u32m4(reducedVal, 2, vl);
            cosSignTempVal = __riscv_vand_vx_u32m4(cosSignTempVal, 4, vl);
            vbool8_t cosSignMask = __riscv_vmsne_vx_u32m4_b8(cosSignTempVal, 0, vl);

            // reducedY[i] = (float) reduced[i]
            vfloat32m4_t reducedYVal = __riscv_vfcvt_f_xu_v_f32m4(reducedVal, vl);

            // The magic pass: "Extended precision modular arithmetic"
            // x[i] = ((in[i] + reducedY[i] * -DP1) + reducedY[i] * -DP2) + reducedY[i] * -DP3;
            vfloat32m4_t xmm1Val = __riscv_vfmul_vf_f32m4(reducedYVal, c_minus_cephes_DP1, vl);
            xVal = __riscv_vfadd_vv_f32m4(xVal, xmm1Val, vl);
            vfloat32m4_t xmm2Val = __riscv_vfmul_vf_f32m4(reducedYVal, c_minus_cephes_DP2, vl);
            xVal = __riscv_vfadd_vv_f32m4(xVal, xmm2Val, vl);
            vfloat32m4_t xmm3Val = __riscv_vfmul_vf_f32m4(reducedYVal, c_minus_cephes_DP3, vl);
            xVal = __riscv_vfadd_vv_f32m4(xVal, xmm3Val, vl);

            // Calculate both polynomials; one for 0 <= x <= PI / 4,
            //  other for PI / 4 <= x <= PI / 2
            vfloat32m4_t xSqVal = __riscv_vfmul_vv_f32m4(xVal, xVal, vl);

            vfloat32m4_t y1Val = __riscv_vfmul_vf_f32m4(xSqVal, c_coscof_p0, vl);
            y1Val = __riscv_vfadd_vf_f32m4(y1Val, c_coscof_p1, vl);
            y1Val = __riscv_vfmul_vv_f32m4(y1Val, xSqVal, vl);
            y1Val = __riscv_vfadd_vf_f32m4(y1Val, c_coscof_p2, vl);
            y1Val = __riscv_vfmul_vv_f32m4(y1Val, xSqVal, vl);
            y1Val = __riscv_vfmul_vv_f32m4(y1Val, xSqVal, vl);
            y1Val = __riscv_vfsub_vv_f32m4(y1Val, __riscv_vfmul_vf_f32m4(xSqVal, 0.5f, vl), vl);
            y1Val = __riscv_vfadd_vf_f32m4(y1Val, 1, vl);

            vfloat32m4_t y2Val = __riscv_vfmul_vf_f32m4(xSqVal, c_sincof_p0, vl);
            y2Val = __riscv_vfadd_vf_f32m4(y2Val, c_sincof_p1, vl);
            y2Val = __riscv_vfmul_vv_f32m4(y2Val, xSqVal, vl);
            y2Val = __riscv_vfadd_vf_f32m4(y2Val, c_sincof_p2, vl);
            y2Val = __riscv_vfmul_vv_f32m4(y2Val, xSqVal, vl);
            y2Val = __riscv_vfmul_vv_f32m4(y2Val, xVal, vl);
            y2Val = __riscv_vfadd_vv_f32m4(y2Val, xVal, vl);

            // Output results
            // sin[i] = polyMask ? y1[i] : y2[i]
            // cos[i] = polyMask ? y2[i] : y1[i]
            vfloat32m4_t sinVal = __riscv_vmerge_vvm_f32m4(y2Val, y1Val, polyMask, vl);
            vfloat32m4_t cosVal = __riscv_vmerge_vvm_f32m4(y1Val, y2Val, polyMask, vl);

            // outImag[i] = sinSignMask ? -sin[i] : sin[i]
            // outReal[i] = cosSignMask ? cos[i] : -cos[i]
            vfloat32m4_t outImagVal = __riscv_vmerge_vvm_f32m4(
                sinVal, __riscv_vfneg_v_f32m4(sinVal, vl), sinSignMask, vl);
            vfloat32m4_t outRealVal = __riscv_vmerge_vvm_f32m4(
                __riscv_vfneg_v_f32m4(cosVal, vl), cosVal, cosSignMask, vl);

            // Store out[0..vl)
            vfloat32m4x2_t outVal = __riscv_vcreate_v_f32m4x2(outRealVal, outImagVal);
            __riscv_vsseg2e32_v_f32m4x2(outPtr, outVal, vl);

            // Carry the phase to the next chunk (mod 2^32)
            phase_fixed += (uint32_t)vl * phase_inc_fixed;

            // In looping, decrement the number of
            // elements left and increment the pointers
            // by the number of elements processed,
            // taking into account how the output `vl`
            // complex numbers are stored as 2 `float`s
        }
    (*phase) = volk_gnsssdr_sincos_fixed_to_rad(phase_fixed);
}

#endif /* LV_HAVE_RVV */
#endif /* INCLUDED_volk_gnsssdr_s32f_sincos_32fc_H */
