/*!
 * \file volk_gnsssdr_sse3_intrinsics.h
 * \author Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 * \brief Holds SSE3 intrinsics of intrinsics.
 * They should be used in VOLK kernels to avoid copy-paste.
 *
 * Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
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
 */


#ifndef INCLUDED_VOLK_VOLK_SSE3_INTRINSICS_H_
#define INCLUDED_VOLK_VOLK_SSE3_INTRINSICS_H_
#include <pmmintrin.h>

static inline __m128
_mm_complexmul_ps(__m128 x, __m128 y)
{
    __m128 yl, yh, tmp1, tmp2;
    yl = _mm_moveldup_ps(y);           // Load yl with cr,cr,dr,dr
    yh = _mm_movehdup_ps(y);           // Load yh with ci,ci,di,di
    tmp1 = _mm_mul_ps(x, yl);          // tmp1 = ar*cr,ai*cr,br*dr,bi*dr
    x = _mm_shuffle_ps(x, x, 0xB1);    // Re-arrange x to be ai,ar,bi,br
    tmp2 = _mm_mul_ps(x, yh);          // tmp2 = ai*ci,ar*ci,bi*di,br*di
    return _mm_addsub_ps(tmp1, tmp2);  // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
}

static inline __m128
_mm_complexconjugatemul_ps(__m128 x, __m128 y)
{
    const __m128 conjugator = _mm_setr_ps(0, -0.f, 0, -0.f);
    y = _mm_xor_ps(y, conjugator);  // conjugate y
    return _mm_complexmul_ps(x, y);
}

static inline __m128
_mm_magnitudesquared_ps_sse3(__m128 cplxValue1, __m128 cplxValue2)
{
    cplxValue1 = _mm_mul_ps(cplxValue1, cplxValue1);  // Square the values
    cplxValue2 = _mm_mul_ps(cplxValue2, cplxValue2);  // Square the Values
    return _mm_hadd_ps(cplxValue1, cplxValue2);       // Add the I2 and Q2 values
}

static inline __m128
_mm_magnitude_ps_sse3(__m128 cplxValue1, __m128 cplxValue2)
{
    return _mm_sqrt_ps(_mm_magnitudesquared_ps_sse3(cplxValue1, cplxValue2));
}

#endif /* INCLUDE_VOLK_VOLK_SSE3_INTRINSICS_H_ */
