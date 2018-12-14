/*!
 * \file volk_gnsssdr_neon_intrinsics.h
 * \author Carles Fernandez, 2016. carles.fernandez(at)gmail.com
 * \brief Holds NEON intrinsics of intrinsics.
 * They can be used in VOLK_GNSSSDR kernels to avoid copy-paste
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

#ifndef INCLUDED_VOLK_GNSSSDR_NEON_INTRINSICS_H_
#define INCLUDED_VOLK_GNSSSDR_NEON_INTRINSICS_H_

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

#endif /* INCLUDED_VOLK_GNSSSDR_NEON_INTRINSICS_H_ */
