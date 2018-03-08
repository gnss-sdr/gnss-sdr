/*!
 * \file volk_gnsssdr_s32f_sincos_32fc.h
 * \brief VOLK_GNSSSDR kernel: Computes the sine and cosine of a vector of floats.
 * \authors <ul>
 *          <li> Julien Pommier, 2007
 *          <li> Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that computes the sine and cosine of a vector of floats.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2007  Julien Pommier
 *
 *  This software is provided 'as-is', without any express or implied
 *  warranty.  In no event will the authors be held liable for any damages
 *  arising from the use of this software.
 *
 *  Permission is granted to anyone to use this software for any purpose,
 *  including commercial applications, and to alter it and redistribute it
 *  freely, subject to the following restrictions:
 *
 *  1. The origin of this software must not be misrepresented; you must not
 *     claim that you wrote the original software. If you use this software
 *     in a product, an acknowledgment in the product documentation would be
 *     appreciated but is not required.
 *  2. Altered source versions must be plainly marked as such, and must not be
 *     misrepresented as being the original software.
 *  3. This notice may not be removed or altered from any source distribution.
 *
 *  (this is the zlib license)
 */


/*!
 * \page volk_gnsssdr_s32f_sincos_32fc
 *
 * \b Overview
 *
 * VOLK_GNSSSDR kernel that computes the sine and cosine with a fixed
 * phase increment \p phase_inc per sample, providing the output in a complex vector (cosine, sine).
 * WARNING: it is not IEEE compliant, but the max absolute error on sines is 2^-24 on the range [-8192, 8192].
 *          To be safe, keep initial phase + phase_inc * num_points within that range.
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
 * \li phase:          Pointer to a float containing the final phase, in radians.
 *
 */


#ifndef INCLUDED_volk_gnsssdr_s32f_sincos_32fc_H
#define INCLUDED_volk_gnsssdr_s32f_sincos_32fc_H

#include <math.h>
#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/* Adapted from http://gruntthepeon.free.fr/ssemath/sse_mathfun.h, original code from Julien Pommier  */
/* Based on algorithms from the cephes library http://www.netlib.org/cephes/   */
static inline void volk_gnsssdr_s32f_sincos_32fc_a_sse2(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    lv_32fc_t *bPtr = out;

    const unsigned int sse_iters = num_points / 4;
    unsigned int number = 0;
    float _phase = (*phase);

    __m128 sine, cosine, aux, x, four_phases_reg;
    __m128 xmm1, xmm2, xmm3 = _mm_setzero_ps(), sign_bit_sin, y;
    __m128i emm0, emm2, emm4;

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

    float four_phases[4] = {_phase, _phase + phase_inc, _phase + 2 * phase_inc, _phase + 3 * phase_inc};
    float four_phases_inc[4] = {4 * phase_inc, 4 * phase_inc, 4 * phase_inc, 4 * phase_inc};
    four_phases_reg = _mm_load_ps(four_phases);
    const __m128 four_phases_inc_reg = _mm_load_ps(four_phases_inc);

    for (; number < sse_iters; number++)
        {
            x = four_phases_reg;

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

            four_phases_reg = _mm_add_ps(four_phases_reg, four_phases_inc_reg);
        }

    _phase = _phase + phase_inc * (sse_iters * 4);
    for (number = sse_iters * 4; number < num_points; number++)
        {
            *bPtr++ = lv_cmake((float)cosf((_phase)), (float)sinf((_phase)));
            _phase += phase_inc;
        }
    (*phase) = _phase;
}

#endif /* LV_HAVE_SSE2  */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/* Adapted from http://gruntthepeon.free.fr/ssemath/sse_mathfun.h, original code from Julien Pommier  */
/* Based on algorithms from the cephes library http://www.netlib.org/cephes/   */
static inline void volk_gnsssdr_s32f_sincos_32fc_u_sse2(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    lv_32fc_t *bPtr = out;

    const unsigned int sse_iters = num_points / 4;
    unsigned int number = 0;

    float _phase = (*phase);

    __m128 sine, cosine, aux, x, four_phases_reg;
    __m128 xmm1, xmm2, xmm3 = _mm_setzero_ps(), sign_bit_sin, y;
    __m128i emm0, emm2, emm4;

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
    float four_phases[4] = {_phase, _phase + phase_inc, _phase + 2 * phase_inc, _phase + 3 * phase_inc};
    __VOLK_ATTR_ALIGNED(16)
    float four_phases_inc[4] = {4 * phase_inc, 4 * phase_inc, 4 * phase_inc, 4 * phase_inc};
    four_phases_reg = _mm_load_ps(four_phases);
    const __m128 four_phases_inc_reg = _mm_load_ps(four_phases_inc);

    for (; number < sse_iters; number++)
        {
            x = four_phases_reg;

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

            four_phases_reg = _mm_add_ps(four_phases_reg, four_phases_inc_reg);
        }

    _phase = _phase + phase_inc * (sse_iters * 4);
    for (number = sse_iters * 4; number < num_points; number++)
        {
            *bPtr++ = lv_cmake((float)cosf(_phase), (float)sinf(_phase));
            _phase += phase_inc;
        }
    (*phase) = _phase;
}

#endif /* LV_HAVE_SSE2  */


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_s32f_sincos_32fc_generic(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    float _phase = (*phase);
    unsigned int i;
    for (i = 0; i < num_points; i++)
        {
            *out++ = lv_cmake((float)cosf(_phase), (float)sinf(_phase));
            _phase += phase_inc;
        }
    (*phase) = _phase;
}

#endif /* LV_HAVE_GENERIC  */


#ifdef LV_HAVE_GENERIC
#include <volk_gnsssdr/volk_gnsssdr_sine_table.h>
#include <stdint.h>
static inline void volk_gnsssdr_s32f_sincos_32fc_generic_fxpt(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    float _in, s, c;
    unsigned int i;
    int32_t x, sin_index, cos_index, d;
    const float PI = 3.14159265358979323846;
    const float TWO_TO_THE_31_DIV_PI = 2147483648.0 / PI;
    const float TWO_PI = PI * 2;
    const int32_t bitlength = 32;
    const int32_t Nbits = 10;
    const int32_t diffbits = bitlength - Nbits;
    uint32_t ux;
    float _phase = (*phase);
    for (i = 0; i < num_points; i++)
        {
            _in = _phase;
            d = (int32_t)floor(_in / TWO_PI + 0.5);
            _in -= d * TWO_PI;
            x = (int32_t)((float)_in * TWO_TO_THE_31_DIV_PI);

            ux = x;
            sin_index = ux >> diffbits;
            s = sine_table_10bits[sin_index][0] * (ux >> 1) + sine_table_10bits[sin_index][1];

            ux = x + 0x40000000;
            cos_index = ux >> diffbits;
            c = sine_table_10bits[cos_index][0] * (ux >> 1) + sine_table_10bits[cos_index][1];

            *out++ = lv_cmake((float)c, (float)s);
            _phase += phase_inc;
        }
    (*phase) = _phase;
}

#endif /* LV_HAVE_GENERIC  */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>
/* Based on algorithms from the cephes library http://www.netlib.org/cephes/
 * Adapted to AVX2 by Carles Fernandez, based on original SSE2 code by Julien Pommier*/
static inline void volk_gnsssdr_s32f_sincos_32fc_a_avx2(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    lv_32fc_t *bPtr = out;

    const unsigned int avx_iters = num_points / 8;
    unsigned int number = 0;

    float _phase = (*phase);

    __m256 sine, cosine, x, eight_phases_reg;
    __m256 xmm1, xmm2, xmm3 = _mm256_setzero_ps(), sign_bit_sin, y;
    __m256i emm0, emm2, emm4;
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
    float eight_phases[8] = {_phase, _phase + phase_inc, _phase + 2 * phase_inc, _phase + 3 * phase_inc, _phase + 4 * phase_inc, _phase + 5 * phase_inc, _phase + 6 * phase_inc, _phase + 7 * phase_inc};
    __VOLK_ATTR_ALIGNED(32)
    float eight_phases_inc[8] = {8 * phase_inc, 8 * phase_inc, 8 * phase_inc, 8 * phase_inc, 8 * phase_inc, 8 * phase_inc, 8 * phase_inc, 8 * phase_inc};
    eight_phases_reg = _mm256_load_ps(eight_phases);
    const __m256 eight_phases_inc_reg = _mm256_load_ps(eight_phases_inc);

    for (; number < avx_iters; number++)
        {
            x = eight_phases_reg;

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

            eight_phases_reg = _mm256_add_ps(eight_phases_reg, eight_phases_inc_reg);
        }
    _mm256_zeroupper();
    _phase = _phase + phase_inc * (avx_iters * 8);
    for (number = avx_iters * 8; number < num_points; number++)
        {
            out[number] = lv_cmake((float)cosf(_phase), (float)sinf(_phase));
            _phase += phase_inc;
        }
    (*phase) = _phase;
}

#endif /* LV_HAVE_AVX2  */


#ifdef LV_HAVE_AVX2
#include <immintrin.h>
/* Based on algorithms from the cephes library http://www.netlib.org/cephes/
 * Adapted to AVX2 by Carles Fernandez, based on original SSE2 code by Julien Pommier*/
static inline void volk_gnsssdr_s32f_sincos_32fc_u_avx2(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    lv_32fc_t *bPtr = out;

    const unsigned int avx_iters = num_points / 8;
    unsigned int number = 0;

    float _phase = (*phase);

    __m256 sine, cosine, x, eight_phases_reg;
    __m256 xmm1, xmm2, xmm3 = _mm256_setzero_ps(), sign_bit_sin, y;
    __m256i emm0, emm2, emm4;
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
    float eight_phases[8] = {_phase, _phase + phase_inc, _phase + 2 * phase_inc, _phase + 3 * phase_inc, _phase + 4 * phase_inc, _phase + 5 * phase_inc, _phase + 6 * phase_inc, _phase + 7 * phase_inc};
    __VOLK_ATTR_ALIGNED(32)
    float eight_phases_inc[8] = {8 * phase_inc, 8 * phase_inc, 8 * phase_inc, 8 * phase_inc, 8 * phase_inc, 8 * phase_inc, 8 * phase_inc, 8 * phase_inc};
    eight_phases_reg = _mm256_load_ps(eight_phases);
    const __m256 eight_phases_inc_reg = _mm256_load_ps(eight_phases_inc);

    for (; number < avx_iters; number++)
        {
            x = eight_phases_reg;

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

            eight_phases_reg = _mm256_add_ps(eight_phases_reg, eight_phases_inc_reg);
        }
    _mm256_zeroupper();
    _phase = _phase + phase_inc * (avx_iters * 8);
    for (number = avx_iters * 8; number < num_points; number++)
        {
            out[number] = lv_cmake((float)cosf(_phase), (float)sinf(_phase));
            _phase += phase_inc;
        }
    (*phase) = _phase;
}

#endif /* LV_HAVE_AVX2  */


#ifdef LV_HAVE_NEON
#include <arm_neon.h>
/* Adapted from http://gruntthepeon.free.fr/ssemath/neon_mathfun.h, original code from Julien Pommier  */
/* Based on algorithms from the cephes library http://www.netlib.org/cephes/   */
static inline void volk_gnsssdr_s32f_sincos_32fc_neon(lv_32fc_t *out, const float phase_inc, float *phase, unsigned int num_points)
{
    lv_32fc_t *bPtr = out;
    const unsigned int neon_iters = num_points / 4;
    float _phase = (*phase);

    __VOLK_ATTR_ALIGNED(16)
    float32_t four_phases[4] = {_phase, _phase + phase_inc, _phase + 2 * phase_inc, _phase + 3 * phase_inc};
    float four_inc = 4 * phase_inc;
    __VOLK_ATTR_ALIGNED(16)
    float32_t four_phases_inc[4] = {four_inc, four_inc, four_inc, four_inc};

    float32x4_t four_phases_reg = vld1q_f32(four_phases);
    float32x4_t four_phases_inc_reg = vld1q_f32(four_phases_inc);

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

    unsigned int number = 0;

    float32x4_t x, xmm1, xmm2, xmm3, y, y1, y2, ys, yc, z;
    float32x4x2_t result;

    uint32x4_t emm2, poly_mask, sign_mask_sin, sign_mask_cos;

    for (; number < neon_iters; number++)
        {
            x = four_phases_reg;

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

            four_phases_reg = vaddq_f32(four_phases_reg, four_phases_inc_reg);
        }

    _phase = _phase + phase_inc * (neon_iters * 4);
    for (number = neon_iters * 4; number < num_points; number++)
        {
            *bPtr++ = lv_cmake((float)cosf(_phase), (float)sinf(_phase));
            _phase += phase_inc;
        }
    (*phase) = _phase;
}

#endif /* LV_HAVE_NEON  */

#endif /* INCLUDED_volk_gnsssdr_s32f_sincos_32fc_H */
