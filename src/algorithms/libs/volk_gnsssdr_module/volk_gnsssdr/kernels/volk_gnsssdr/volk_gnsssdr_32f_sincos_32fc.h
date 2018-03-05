/*!
 * \file volk_gnsssdr_32f_sincos_32fc.h
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
 * \page volk_gnsssdr_32f_sincos_32fc
 *
 * \b Overview
 *
 * VOLK_GNSSSDR kernel that computes the sine and cosine of a vector
 * of floats, providing the output in a complex vector (cosine, sine).
 * WARNING: it is not IEEE compliant, but the max absolute error on sines is 2^-24 on the range [-8192, 8192].
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_32f_sincos_32fc(lv_32fc_t* out, const float* in, unsigned int num_points)
 * \endcode
 *
 * \b Inputs
 * \li in:             Vector of floats, in radians.
 * \li num_points:     Number of components in \p in to be computed.
 *
 * \b Outputs
 * \li out:            Vector of the form lv_32fc_t out[n] = lv_cmake(cos(in[n]), sin(in[n]))
 *
 */

#ifndef INCLUDED_volk_gnsssdr_32f_sincos_32fc_H
#define INCLUDED_volk_gnsssdr_32f_sincos_32fc_H

#include <math.h>
#include <volk_gnsssdr/volk_gnsssdr_common.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>

#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
/* Adapted from the original VOLK for comparison purposes.
 *  In turn based on algorithms from:
 *   Naoki Shibata, "Efficient Evaluation Methods of Elementary Functions Suitable for SIMD Computation,"
 *   Computer Science Research and Development, May 2010, Volume 25, Issue 1, pp 25-32. DOI 10.1007/s00450-010-0108-2  */
static inline void volk_gnsssdr_32f_sincos_32fc_u_sse4_1(lv_32fc_t* out, const float* in, unsigned int num_points)
{
    lv_32fc_t* bPtr = out;
    const float* aPtr = in;

    unsigned int number = 0;
    unsigned int quarterPoints = num_points / 4;
    unsigned int i = 0;

    __m128 aVal, s, m4pi, pio4A, pio4B, cp1, cp2, cp3, cp4, cp5, ffours, ftwos, fones, fzeroes;
    __m128 sine, cosine, condition1, condition2, condition3, cplxValue;
    __m128i q, r, ones, twos, fours;

    m4pi = _mm_set1_ps(1.273239545);
    pio4A = _mm_set1_ps(0.78515625);
    pio4B = _mm_set1_ps(0.241876e-3);
    ffours = _mm_set1_ps(4.0);
    ftwos = _mm_set1_ps(2.0);
    fones = _mm_set1_ps(1.0);
    fzeroes = _mm_setzero_ps();
    ones = _mm_set1_epi32(1);
    twos = _mm_set1_epi32(2);
    fours = _mm_set1_epi32(4);

    cp1 = _mm_set1_ps(1.0);
    cp2 = _mm_set1_ps(0.83333333e-1);
    cp3 = _mm_set1_ps(0.2777778e-2);
    cp4 = _mm_set1_ps(0.49603e-4);
    cp5 = _mm_set1_ps(0.551e-6);

    for (; number < quarterPoints; number++)
        {
            aVal = _mm_loadu_ps(aPtr);
            __VOLK_GNSSSDR_PREFETCH(aPtr + 8);
            s = _mm_sub_ps(aVal, _mm_and_ps(_mm_mul_ps(aVal, ftwos), _mm_cmplt_ps(aVal, fzeroes)));
            q = _mm_cvtps_epi32(_mm_floor_ps(_mm_mul_ps(s, m4pi)));
            r = _mm_add_epi32(q, _mm_and_si128(q, ones));

            s = _mm_sub_ps(s, _mm_mul_ps(_mm_cvtepi32_ps(r), pio4A));
            s = _mm_sub_ps(s, _mm_mul_ps(_mm_cvtepi32_ps(r), pio4B));

            s = _mm_div_ps(s, _mm_set1_ps(8.0));  // The constant is 2^N, for 3 times argument reduction
            s = _mm_mul_ps(s, s);
            // Evaluate Taylor series
            s = _mm_mul_ps(_mm_add_ps(_mm_mul_ps(_mm_sub_ps(_mm_mul_ps(_mm_add_ps(_mm_mul_ps(_mm_sub_ps(_mm_mul_ps(s, cp5), cp4), s), cp3), s), cp2), s), cp1), s);

            for (i = 0; i < 3; i++)
                {
                    s = _mm_mul_ps(s, _mm_sub_ps(ffours, s));
                }
            s = _mm_div_ps(s, ftwos);

            sine = _mm_sqrt_ps(_mm_mul_ps(_mm_sub_ps(ftwos, s), s));
            cosine = _mm_sub_ps(fones, s);

            condition1 = _mm_cmpneq_ps(_mm_cvtepi32_ps(_mm_and_si128(_mm_add_epi32(q, ones), twos)), fzeroes);
            condition2 = _mm_cmpneq_ps(_mm_cmpneq_ps(_mm_cvtepi32_ps(_mm_and_si128(q, fours)), fzeroes), _mm_cmplt_ps(aVal, fzeroes));
            condition3 = _mm_cmpneq_ps(_mm_cvtepi32_ps(_mm_and_si128(_mm_add_epi32(q, twos), fours)), fzeroes);

            cplxValue = sine;
            sine = _mm_add_ps(sine, _mm_and_ps(_mm_sub_ps(cosine, sine), condition1));
            sine = _mm_sub_ps(sine, _mm_and_ps(_mm_mul_ps(sine, _mm_set1_ps(2.0f)), condition2));

            cosine = _mm_add_ps(cosine, _mm_and_ps(_mm_sub_ps(cplxValue, cosine), condition1));
            cosine = _mm_sub_ps(cosine, _mm_and_ps(_mm_mul_ps(cosine, _mm_set1_ps(2.0f)), condition3));

            cplxValue = _mm_unpacklo_ps(cosine, sine);
            _mm_storeu_ps((float*)bPtr, cplxValue);
            bPtr += 2;

            cplxValue = _mm_unpackhi_ps(cosine, sine);
            _mm_storeu_ps((float*)bPtr, cplxValue);
            bPtr += 2;

            aPtr += 4;
        }

    number = quarterPoints * 4;
    for (; number < num_points; number++)
        {
            float _in = *aPtr++;
            *bPtr++ = lv_cmake(cosf(_in), sinf(_in));
        }
}

#endif /* LV_HAVE_SSE4_1 for unaligned */


#ifdef LV_HAVE_SSE4_1
#include <smmintrin.h>
/* Adapted from the original VOLK for comparison purposes.
 *  In turn based on algorithms from:
 *   Naoki Shibata, "Efficient Evaluation Methods of Elementary Functions Suitable for SIMD Computation,"
 *   Computer Science Research and Development, May 2010, Volume 25, Issue 1, pp 25-32. DOI 10.1007/s00450-010-0108-2  */
static inline void volk_gnsssdr_32f_sincos_32fc_a_sse4_1(lv_32fc_t* out, const float* in, unsigned int num_points)
{
    lv_32fc_t* bPtr = out;
    const float* aPtr = in;

    unsigned int number = 0;
    unsigned int quarterPoints = num_points / 4;
    unsigned int i = 0;

    __m128 aVal, s, m4pi, pio4A, pio4B, cp1, cp2, cp3, cp4, cp5, ffours, ftwos, fones, fzeroes;
    __m128 sine, cosine, condition1, condition2, condition3, cplxValue;
    __m128i q, r, ones, twos, fours;

    m4pi = _mm_set1_ps(1.273239545);
    pio4A = _mm_set1_ps(0.78515625);
    pio4B = _mm_set1_ps(0.241876e-3);
    ffours = _mm_set1_ps(4.0);
    ftwos = _mm_set1_ps(2.0);
    fones = _mm_set1_ps(1.0);
    fzeroes = _mm_setzero_ps();
    ones = _mm_set1_epi32(1);
    twos = _mm_set1_epi32(2);
    fours = _mm_set1_epi32(4);

    cp1 = _mm_set1_ps(1.0);
    cp2 = _mm_set1_ps(0.83333333e-1);
    cp3 = _mm_set1_ps(0.2777778e-2);
    cp4 = _mm_set1_ps(0.49603e-4);
    cp5 = _mm_set1_ps(0.551e-6);

    for (; number < quarterPoints; number++)
        {
            aVal = _mm_load_ps(aPtr);
            __VOLK_GNSSSDR_PREFETCH(aPtr + 8);
            s = _mm_sub_ps(aVal, _mm_and_ps(_mm_mul_ps(aVal, ftwos), _mm_cmplt_ps(aVal, fzeroes)));
            q = _mm_cvtps_epi32(_mm_floor_ps(_mm_mul_ps(s, m4pi)));
            r = _mm_add_epi32(q, _mm_and_si128(q, ones));

            s = _mm_sub_ps(s, _mm_mul_ps(_mm_cvtepi32_ps(r), pio4A));
            s = _mm_sub_ps(s, _mm_mul_ps(_mm_cvtepi32_ps(r), pio4B));

            s = _mm_div_ps(s, _mm_set1_ps(8.0));  // The constant is 2^N, for 3 times argument reduction
            s = _mm_mul_ps(s, s);
            // Evaluate Taylor series
            s = _mm_mul_ps(_mm_add_ps(_mm_mul_ps(_mm_sub_ps(_mm_mul_ps(_mm_add_ps(_mm_mul_ps(_mm_sub_ps(_mm_mul_ps(s, cp5), cp4), s), cp3), s), cp2), s), cp1), s);

            for (i = 0; i < 3; i++)
                {
                    s = _mm_mul_ps(s, _mm_sub_ps(ffours, s));
                }
            s = _mm_div_ps(s, ftwos);

            sine = _mm_sqrt_ps(_mm_mul_ps(_mm_sub_ps(ftwos, s), s));
            cosine = _mm_sub_ps(fones, s);

            condition1 = _mm_cmpneq_ps(_mm_cvtepi32_ps(_mm_and_si128(_mm_add_epi32(q, ones), twos)), fzeroes);
            condition2 = _mm_cmpneq_ps(_mm_cmpneq_ps(_mm_cvtepi32_ps(_mm_and_si128(q, fours)), fzeroes), _mm_cmplt_ps(aVal, fzeroes));
            condition3 = _mm_cmpneq_ps(_mm_cvtepi32_ps(_mm_and_si128(_mm_add_epi32(q, twos), fours)), fzeroes);

            cplxValue = sine;
            sine = _mm_add_ps(sine, _mm_and_ps(_mm_sub_ps(cosine, sine), condition1));
            sine = _mm_sub_ps(sine, _mm_and_ps(_mm_mul_ps(sine, _mm_set1_ps(2.0f)), condition2));

            cosine = _mm_add_ps(cosine, _mm_and_ps(_mm_sub_ps(cplxValue, cosine), condition1));
            cosine = _mm_sub_ps(cosine, _mm_and_ps(_mm_mul_ps(cosine, _mm_set1_ps(2.0f)), condition3));

            cplxValue = _mm_unpacklo_ps(cosine, sine);
            _mm_store_ps((float*)bPtr, cplxValue);
            bPtr += 2;

            cplxValue = _mm_unpackhi_ps(cosine, sine);
            _mm_store_ps((float*)bPtr, cplxValue);
            bPtr += 2;

            aPtr += 4;
        }

    number = quarterPoints * 4;
    for (; number < num_points; number++)
        {
            float _in = *aPtr++;
            *bPtr++ = lv_cmake(cosf(_in), sinf(_in));
        }
}

#endif /* LV_HAVE_SSE4_1 for aligned */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/* Adapted from http://gruntthepeon.free.fr/ssemath/sse_mathfun.h, original code from Julien Pommier  */
/* Based on algorithms from the cephes library http://www.netlib.org/cephes/   */
static inline void volk_gnsssdr_32f_sincos_32fc_a_sse2(lv_32fc_t* out, const float* in, unsigned int num_points)
{
    lv_32fc_t* bPtr = out;
    const float* aPtr = in;

    const unsigned int sse_iters = num_points / 4;
    unsigned int number = 0;
    float _in;

    __m128 sine, cosine, aux, x;
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

    for (; number < sse_iters; number++)
        {
            x = _mm_load_ps(aPtr);
            __VOLK_GNSSSDR_PREFETCH(aPtr + 8);

            sign_bit_sin = x;
            /* take the absolute value */
            x = _mm_and_ps(x, *(__m128*)_ps_inv_sign_mask);
            /* extract the sign bit (upper one) */
            sign_bit_sin = _mm_and_ps(sign_bit_sin, *(__m128*)_ps_sign_mask);

            /* scale by 4/Pi */
            y = _mm_mul_ps(x, *(__m128*)_ps_cephes_FOPI);

            /* store the integer part of y in emm2 */
            emm2 = _mm_cvttps_epi32(y);

            /* j=(j+1) & (~1) (see the cephes sources) */
            emm2 = _mm_add_epi32(emm2, *(__m128i*)_pi32_1);
            emm2 = _mm_and_si128(emm2, *(__m128i*)_pi32_inv1);
            y = _mm_cvtepi32_ps(emm2);

            emm4 = emm2;

            /* get the swap sign flag for the sine */
            emm0 = _mm_and_si128(emm2, *(__m128i*)_pi32_4);
            emm0 = _mm_slli_epi32(emm0, 29);
            __m128 swap_sign_bit_sin = _mm_castsi128_ps(emm0);

            /* get the polynom selection mask for the sine*/
            emm2 = _mm_and_si128(emm2, *(__m128i*)_pi32_2);
            emm2 = _mm_cmpeq_epi32(emm2, _mm_setzero_si128());
            __m128 poly_mask = _mm_castsi128_ps(emm2);

            /* The magic pass: "Extended precision modular arithmetic”
               x = ((x - y * DP1) - y * DP2) - y * DP3; */
            xmm1 = *(__m128*)_ps_minus_cephes_DP1;
            xmm2 = *(__m128*)_ps_minus_cephes_DP2;
            xmm3 = *(__m128*)_ps_minus_cephes_DP3;
            xmm1 = _mm_mul_ps(y, xmm1);
            xmm2 = _mm_mul_ps(y, xmm2);
            xmm3 = _mm_mul_ps(y, xmm3);
            x = _mm_add_ps(x, xmm1);
            x = _mm_add_ps(x, xmm2);
            x = _mm_add_ps(x, xmm3);

            emm4 = _mm_sub_epi32(emm4, *(__m128i*)_pi32_2);
            emm4 = _mm_andnot_si128(emm4, *(__m128i*)_pi32_4);
            emm4 = _mm_slli_epi32(emm4, 29);
            __m128 sign_bit_cos = _mm_castsi128_ps(emm4);

            sign_bit_sin = _mm_xor_ps(sign_bit_sin, swap_sign_bit_sin);

            /* Evaluate the first polynom  (0 <= x <= Pi/4) */
            __m128 z = _mm_mul_ps(x, x);
            y = *(__m128*)_ps_coscof_p0;

            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, *(__m128*)_ps_coscof_p1);
            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, *(__m128*)_ps_coscof_p2);
            y = _mm_mul_ps(y, z);
            y = _mm_mul_ps(y, z);
            __m128 tmp = _mm_mul_ps(z, *(__m128*)_ps_0p5);
            y = _mm_sub_ps(y, tmp);
            y = _mm_add_ps(y, *(__m128*)_ps_1);

            /* Evaluate the second polynom  (Pi/4 <= x <= 0) */

            __m128 y2 = *(__m128*)_ps_sincof_p0;
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, *(__m128*)_ps_sincof_p1);
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, *(__m128*)_ps_sincof_p2);
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
            _mm_store_ps((float*)bPtr, aux);
            bPtr += 2;
            aux = _mm_unpackhi_ps(cosine, sine);
            _mm_store_ps((float*)bPtr, aux);
            bPtr += 2;

            aPtr += 4;
        }

    for (number = sse_iters * 4; number < num_points; number++)
        {
            _in = *aPtr++;
            *bPtr++ = lv_cmake((float)cosf(_in), (float)sinf(_in));
        }
}
#endif /* LV_HAVE_SSE2  */


#ifdef LV_HAVE_SSE2
#include <emmintrin.h>
/* Adapted from http://gruntthepeon.free.fr/ssemath/sse_mathfun.h, original code from Julien Pommier  */
/* Based on algorithms from the cephes library http://www.netlib.org/cephes/   */
static inline void volk_gnsssdr_32f_sincos_32fc_u_sse2(lv_32fc_t* out, const float* in, unsigned int num_points)
{
    lv_32fc_t* bPtr = out;
    const float* aPtr = in;

    const unsigned int sse_iters = num_points / 4;
    unsigned int number = 0;
    float _in;

    __m128 sine, cosine, aux, x;
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

    for (; number < sse_iters; number++)
        {
            x = _mm_loadu_ps(aPtr);
            __VOLK_GNSSSDR_PREFETCH(aPtr + 8);

            sign_bit_sin = x;
            /* take the absolute value */
            x = _mm_and_ps(x, *(__m128*)_ps_inv_sign_mask);
            /* extract the sign bit (upper one) */
            sign_bit_sin = _mm_and_ps(sign_bit_sin, *(__m128*)_ps_sign_mask);

            /* scale by 4/Pi */
            y = _mm_mul_ps(x, *(__m128*)_ps_cephes_FOPI);

            /* store the integer part of y in emm2 */
            emm2 = _mm_cvttps_epi32(y);

            /* j=(j+1) & (~1) (see the cephes sources) */
            emm2 = _mm_add_epi32(emm2, *(__m128i*)_pi32_1);
            emm2 = _mm_and_si128(emm2, *(__m128i*)_pi32_inv1);
            y = _mm_cvtepi32_ps(emm2);

            emm4 = emm2;

            /* get the swap sign flag for the sine */
            emm0 = _mm_and_si128(emm2, *(__m128i*)_pi32_4);
            emm0 = _mm_slli_epi32(emm0, 29);
            __m128 swap_sign_bit_sin = _mm_castsi128_ps(emm0);

            /* get the polynom selection mask for the sine*/
            emm2 = _mm_and_si128(emm2, *(__m128i*)_pi32_2);
            emm2 = _mm_cmpeq_epi32(emm2, _mm_setzero_si128());
            __m128 poly_mask = _mm_castsi128_ps(emm2);

            /* The magic pass: "Extended precision modular arithmetic”
               x = ((x - y * DP1) - y * DP2) - y * DP3; */
            xmm1 = *(__m128*)_ps_minus_cephes_DP1;
            xmm2 = *(__m128*)_ps_minus_cephes_DP2;
            xmm3 = *(__m128*)_ps_minus_cephes_DP3;
            xmm1 = _mm_mul_ps(y, xmm1);
            xmm2 = _mm_mul_ps(y, xmm2);
            xmm3 = _mm_mul_ps(y, xmm3);
            x = _mm_add_ps(x, xmm1);
            x = _mm_add_ps(x, xmm2);
            x = _mm_add_ps(x, xmm3);

            emm4 = _mm_sub_epi32(emm4, *(__m128i*)_pi32_2);
            emm4 = _mm_andnot_si128(emm4, *(__m128i*)_pi32_4);
            emm4 = _mm_slli_epi32(emm4, 29);
            __m128 sign_bit_cos = _mm_castsi128_ps(emm4);

            sign_bit_sin = _mm_xor_ps(sign_bit_sin, swap_sign_bit_sin);

            /* Evaluate the first polynom  (0 <= x <= Pi/4) */
            __m128 z = _mm_mul_ps(x, x);
            y = *(__m128*)_ps_coscof_p0;

            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, *(__m128*)_ps_coscof_p1);
            y = _mm_mul_ps(y, z);
            y = _mm_add_ps(y, *(__m128*)_ps_coscof_p2);
            y = _mm_mul_ps(y, z);
            y = _mm_mul_ps(y, z);
            __m128 tmp = _mm_mul_ps(z, *(__m128*)_ps_0p5);
            y = _mm_sub_ps(y, tmp);
            y = _mm_add_ps(y, *(__m128*)_ps_1);

            /* Evaluate the second polynom  (Pi/4 <= x <= 0) */

            __m128 y2 = *(__m128*)_ps_sincof_p0;
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, *(__m128*)_ps_sincof_p1);
            y2 = _mm_mul_ps(y2, z);
            y2 = _mm_add_ps(y2, *(__m128*)_ps_sincof_p2);
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
            _mm_storeu_ps((float*)bPtr, aux);
            bPtr += 2;
            aux = _mm_unpackhi_ps(cosine, sine);
            _mm_storeu_ps((float*)bPtr, aux);
            bPtr += 2;

            aPtr += 4;
        }

    for (number = sse_iters * 4; number < num_points; number++)
        {
            _in = *aPtr++;
            *bPtr++ = lv_cmake((float)cosf(_in), (float)sinf(_in));
        }
}
#endif /* LV_HAVE_SSE2  */


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_32f_sincos_32fc_generic(lv_32fc_t* out, const float* in, unsigned int num_points)
{
    float _in;
    unsigned int i;
    for (i = 0; i < num_points; i++)
        {
            _in = *in++;
            *out++ = lv_cmake((float)cosf(_in), (float)sinf(_in));
        }
}

#endif /* LV_HAVE_GENERIC  */


#ifdef LV_HAVE_GENERIC
#include <volk_gnsssdr/volk_gnsssdr_sine_table.h>
#include <stdint.h>
static inline void volk_gnsssdr_32f_sincos_32fc_generic_fxpt(lv_32fc_t* out, const float* in, unsigned int num_points)
{
    float _in, s, c;
    int32_t x, sin_index, cos_index, d;
    const float PI = 3.14159265358979323846;
    const float TWO_TO_THE_31_DIV_PI = 2147483648.0 / PI;
    const float TWO_PI = PI * 2;
    const int32_t bitlength = 32;
    const int32_t Nbits = 10;
    const int32_t diffbits = bitlength - Nbits;
    uint32_t ux;
    unsigned int i;
    for (i = 0; i < num_points; i++)
        {
            _in = *in++;
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
        }
}

#endif /* LV_HAVE_GENERIC  */


#ifdef LV_HAVE_NEON
#include <arm_neon.h>
/* Adapted from http://gruntthepeon.free.fr/ssemath/neon_mathfun.h, original code from Julien Pommier  */
/* Based on algorithms from the cephes library http://www.netlib.org/cephes/   */
static inline void volk_gnsssdr_32f_sincos_32fc_neon(lv_32fc_t* out, const float* in, unsigned int num_points)
{
    lv_32fc_t* bPtr = out;
    const float* aPtr = in;
    const unsigned int neon_iters = num_points / 4;

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
    float _in;

    float32x4_t x, xmm1, xmm2, xmm3, y, y1, y2, ys, yc, z;
    float32x4x2_t result;

    uint32x4_t emm2, poly_mask, sign_mask_sin, sign_mask_cos;

    for (; number < neon_iters; number++)
        {
            x = vld1q_f32(aPtr);
            __VOLK_GNSSSDR_PREFETCH(aPtr + 8);

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

            vst2q_f32((float32_t*)bPtr, result);
            bPtr += 4;
            aPtr += 4;
        }

    for (number = neon_iters * 4; number < num_points; number++)
        {
            _in = *aPtr++;
            *bPtr++ = lv_cmake((float)cosf(_in), (float)sinf(_in));
        }
}

#endif /* LV_HAVE_NEON  */


#endif /* INCLUDED_volk_gnsssdr_32f_sincos_32fc_H  */
