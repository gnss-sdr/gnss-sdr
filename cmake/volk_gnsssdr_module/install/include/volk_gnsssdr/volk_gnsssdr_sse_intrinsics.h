/*!
 * \file volk_gnsssdr_sse_intrinsics.h
 * \author Andres Cecilia, 2014. a.cecilia.luque(at)gmail.com
 * \brief Holds SSE intrinsics of intrinsics.
 * They should be used in VOLK kernels to avoid copy-paste
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_VOLK_VOLK_SSE_INTRINSICS_H
#define INCLUDED_VOLK_VOLK_SSE_INTRINSICS_H
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
