/*!
 * \file volk_gnsssdr_sse_intrinsics.h
 * \author Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 * \brief Holds SSE intrinsics of intrinsics.
 * They should be used in VOLK kernels to avoid copy-paste
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

#ifndef INCLUDED_VOLK_VOLK_SSE_INTRINSICS_H_
#define INCLUDED_VOLK_VOLK_SSE_INTRINSICS_H_
#include <xmmintrin.h>

static inline __m128
_mm_magnitudesquared_ps(__m128 cplxValue1, __m128 cplxValue2)
{
    __m128 iValue, qValue;
    // Arrange in i1i2i3i4 format
    iValue = _mm_shuffle_ps(cplxValue1, cplxValue2, _MM_SHUFFLE(2, 0, 2, 0));
    // Arrange in q1q2q3q4 format
    qValue = _mm_shuffle_ps(cplxValue1, cplxValue2, _MM_SHUFFLE(3, 1, 3, 1));
    iValue = _mm_mul_ps(iValue, iValue);  // Square the I values
    qValue = _mm_mul_ps(qValue, qValue);  // Square the Q Values
    return _mm_add_ps(iValue, qValue);    // Add the I2 and Q2 values
}

static inline __m128
_mm_magnitude_ps(__m128 cplxValue1, __m128 cplxValue2)
{
    return _mm_sqrt_ps(_mm_magnitudesquared_ps(cplxValue1, cplxValue2));
}

#endif /* INCLUDED_VOLK_VOLK_SSE_INTRINSICS_H_ */
