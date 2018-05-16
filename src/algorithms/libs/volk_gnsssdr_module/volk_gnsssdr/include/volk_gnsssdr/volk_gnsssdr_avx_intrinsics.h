/*!
 * \file volk_gnsssdr_avx_intrinsics.h
 * \author Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 * \brief This file is intended to hold AVX intrinsics of intrinsics.
 * They should be used in VOLK kernels to avoid copy-paste.
 *
 * Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 */


#ifndef INCLUDED_VOLK_VOLK_AVX_INTRINSICS_H_
#define INCLUDED_VOLK_VOLK_AVX_INTRINSICS_H_
#include <immintrin.h>

static inline __m256
_mm256_complexmul_ps(__m256 x, __m256 y)
{
    __m256 yl, yh, tmp1, tmp2;
    yl = _mm256_moveldup_ps(y);           // Load yl with cr,cr,dr,dr ...
    yh = _mm256_movehdup_ps(y);           // Load yh with ci,ci,di,di ...
    tmp1 = _mm256_mul_ps(x, yl);          // tmp1 = ar*cr,ai*cr,br*dr,bi*dr ...
    x = _mm256_shuffle_ps(x, x, 0xB1);    // Re-arrange x to be ai,ar,bi,br ...
    tmp2 = _mm256_mul_ps(x, yh);          // tmp2 = ai*ci,ar*ci,bi*di,br*di
    return _mm256_addsub_ps(tmp1, tmp2);  // ar*cr-ai*ci, ai*cr+ar*ci, br*dr-bi*di, bi*dr+br*di
}

static inline __m256
_mm256_conjugate_ps(__m256 x)
{
    const __m256 conjugator = _mm256_setr_ps(0, -0.f, 0, -0.f, 0, -0.f, 0, -0.f);
    return _mm256_xor_ps(x, conjugator);  // conjugate y
}

static inline __m256
_mm256_complexconjugatemul_ps(__m256 x, __m256 y)
{
    y = _mm256_conjugate_ps(y);
    return _mm256_complexmul_ps(x, y);
}

static inline __m256
_mm256_magnitudesquared_ps(__m256 cplxValue1, __m256 cplxValue2)
{
    __m256 complex1, complex2;
    cplxValue1 = _mm256_mul_ps(cplxValue1, cplxValue1);  // Square the values
    cplxValue2 = _mm256_mul_ps(cplxValue2, cplxValue2);  // Square the Values
    complex1 = _mm256_permute2f128_ps(cplxValue1, cplxValue2, 0x20);
    complex2 = _mm256_permute2f128_ps(cplxValue1, cplxValue2, 0x31);
    return _mm256_hadd_ps(complex1, complex2);  // Add the I2 and Q2 values
}

static inline __m256 _mm256_complexnormalise_ps(__m256 z)
{
    __m256 tmp1 = _mm256_mul_ps(z, z);
    __m256 tmp2 = _mm256_hadd_ps(tmp1, tmp1);
    tmp1 = _mm256_shuffle_ps(tmp2, tmp2, 0xD8);
    tmp2 = _mm256_sqrt_ps(tmp1);
    return _mm256_div_ps(z, tmp2);
}

static inline __m256
_mm256_magnitude_ps(__m256 cplxValue1, __m256 cplxValue2)
{
    return _mm256_sqrt_ps(_mm256_magnitudesquared_ps(cplxValue1, cplxValue2));
}

#endif /* INCLUDE_VOLK_VOLK_AVX_INTRINSICS_H_ */
