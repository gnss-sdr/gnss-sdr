/*!
 * \file volk_gnsssdr_s32f_sincospuppet_32fc.h
 * \brief VOLK_GNSSSDR puppet for the sincos kernel.
 * \authors <ul>
 *          <li> Carles Fernandez Prades 2016 cfernandez at cttc dot cat
 *          </ul>
 *
 * VOLK_GNSSSDR puppet for integrating the sincos kernel into the test system
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef INCLUDED_volk_gnsssdr_s32f_sincospuppet_32fc_H
#define INCLUDED_volk_gnsssdr_s32f_sincospuppet_32fc_H


#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include "volk_gnsssdr/volk_gnsssdr_s32f_sincos_32fc.h"
#include <math.h>


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_s32f_sincospuppet_32fc_generic(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    float phase[1];
    phase[0] = 3;
    volk_gnsssdr_s32f_sincos_32fc_generic(out, phase_inc, phase, num_points);
}

#endif /* LV_HAVE_GENERIC  */


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_s32f_sincospuppet_32fc_generic_fxpt(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    float phase[1];
    phase[0] = 3;
    volk_gnsssdr_s32f_sincos_32fc_generic_fxpt(out, phase_inc, phase, num_points);
}

#endif /* LV_HAVE_GENERIC  */


#ifdef LV_HAVE_SSE2
static inline void volk_gnsssdr_s32f_sincospuppet_32fc_a_sse2(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    float phase[1];
    phase[0] = 3;
    volk_gnsssdr_s32f_sincos_32fc_a_sse2(out, phase_inc, phase, num_points);
}
#endif /* LV_HAVE_SSE2  */


#ifdef LV_HAVE_SSE2
static inline void volk_gnsssdr_s32f_sincospuppet_32fc_u_sse2(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    float phase[1];
    phase[0] = 3;
    volk_gnsssdr_s32f_sincos_32fc_u_sse2(out, phase_inc, phase, num_points);
}
#endif /* LV_HAVE_SSE2  */


#ifdef LV_HAVE_AVX2
static inline void volk_gnsssdr_s32f_sincospuppet_32fc_a_avx2(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    float phase[1];
    phase[0] = 3;
    volk_gnsssdr_s32f_sincos_32fc_a_avx2(out, phase_inc, phase, num_points);
}
#endif /* LV_HAVE_AVX2  */


#ifdef LV_HAVE_AVX2
static inline void volk_gnsssdr_s32f_sincospuppet_32fc_u_avx2(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    float phase[1];
    phase[0] = 3;
    volk_gnsssdr_s32f_sincos_32fc_u_avx2(out, phase_inc, phase, num_points);
}
#endif /* LV_HAVE_AVX2  */


#ifdef LV_HAVE_NEONV7
static inline void volk_gnsssdr_s32f_sincospuppet_32fc_neon(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    float phase[1];
    phase[0] = 3;
    volk_gnsssdr_s32f_sincos_32fc_neon(out, phase_inc, phase, num_points);
}
#endif /* LV_HAVE_NEONV7  */

#endif /* INCLUDED_volk_gnsssdr_s32f_sincospuppet_32fc_H */
