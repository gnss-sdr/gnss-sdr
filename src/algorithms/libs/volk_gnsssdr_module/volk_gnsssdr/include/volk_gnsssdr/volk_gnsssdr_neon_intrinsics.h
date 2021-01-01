/*!
 * \file volk_gnsssdr_neon_intrinsics.h
 * \author Carles Fernandez, 2016. carles.fernandez(at)gmail.com
 * \brief Holds NEON intrinsics of intrinsics.
 * They can be used in VOLK_GNSSSDR kernels to avoid copy-paste
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef INCLUDED_VOLK_GNSSSDR_NEON_INTRINSICS_H
#define INCLUDED_VOLK_GNSSSDR_NEON_INTRINSICS_H

#include <arm_neon.h>

static inline float32x4_t vdivq_f32(float32x4_t num, float32x4_t den)
{
    const float32x4_t q_inv0 = vrecpeq_f32(den);
    const float32x4_t q_step0 = vrecpsq_f32(q_inv0, den);

    const float32x4_t q_inv1 = vmulq_f32(q_step0, q_inv0);
    return vmulq_f32(num, q_inv1);
}


static inline float32x4_t vsqrtq_f32(float32x4_t q_x)
{
    const float32x4_t q_step_0 = vrsqrteq_f32(q_x);
    // step
    const float32x4_t q_step_parm0 = vmulq_f32(q_x, q_step_0);
    const float32x4_t q_step_result0 = vrsqrtsq_f32(q_step_parm0, q_step_0);
    // step
    const float32x4_t q_step_1 = vmulq_f32(q_step_0, q_step_result0);
    const float32x4_t q_step_parm1 = vmulq_f32(q_x, q_step_1);
    const float32x4_t q_step_result1 = vrsqrtsq_f32(q_step_parm1, q_step_1);
    // take the res
    const float32x4_t q_step_2 = vmulq_f32(q_step_1, q_step_result1);
    // mul by x to get sqrt, not rsqrt
    return vmulq_f32(q_x, q_step_2);
}


/* Magnitude squared for float32x4x2_t */
static inline float32x4_t _vmagnitudesquaredq_f32(float32x4x2_t cmplxValue)
{
    float32x4_t iValue, qValue, result;
    iValue = vmulq_f32(cmplxValue.val[0], cmplxValue.val[0]);  // Square the values
    qValue = vmulq_f32(cmplxValue.val[1], cmplxValue.val[1]);  // Square the values
    result = vaddq_f32(iValue, qValue);                        // Add the I2 and Q2 values
    return result;
}


/* Inverse square root for float32x4_t */
static inline float32x4_t _vinvsqrtq_f32(float32x4_t x)
{
    float32x4_t sqrt_reciprocal = vrsqrteq_f32(x);
    sqrt_reciprocal = vmulq_f32(vrsqrtsq_f32(vmulq_f32(x, sqrt_reciprocal), sqrt_reciprocal), sqrt_reciprocal);
    sqrt_reciprocal = vmulq_f32(vrsqrtsq_f32(vmulq_f32(x, sqrt_reciprocal), sqrt_reciprocal), sqrt_reciprocal);

    return sqrt_reciprocal;
}


/* Complex multiplication for float32x4x2_t */
static inline float32x4x2_t _vmultiply_complexq_f32(float32x4x2_t a_val, float32x4x2_t b_val)
{
    float32x4x2_t tmp_real;
    float32x4x2_t tmp_imag;
    float32x4x2_t c_val;

    // multiply the real*real and imag*imag to get real result
    // a0r*b0r|a1r*b1r|a2r*b2r|a3r*b3r
    tmp_real.val[0] = vmulq_f32(a_val.val[0], b_val.val[0]);
    // a0i*b0i|a1i*b1i|a2i*b2i|a3i*b3i
    tmp_real.val[1] = vmulq_f32(a_val.val[1], b_val.val[1]);
    // Multiply cross terms to get the imaginary result
    // a0r*b0i|a1r*b1i|a2r*b2i|a3r*b3i
    tmp_imag.val[0] = vmulq_f32(a_val.val[0], b_val.val[1]);
    // a0i*b0r|a1i*b1r|a2i*b2r|a3i*b3r
    tmp_imag.val[1] = vmulq_f32(a_val.val[1], b_val.val[0]);
    // combine the products
    c_val.val[0] = vsubq_f32(tmp_real.val[0], tmp_real.val[1]);
    c_val.val[1] = vaddq_f32(tmp_imag.val[0], tmp_imag.val[1]);
    return c_val;
}

#endif /* INCLUDED_VOLK_GNSSSDR_NEON_INTRINSICS_H_ */
