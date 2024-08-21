/*!
 * \file volk_gnsssdr_s32f_sincospuppet_32fc.h
 * \brief VOLK_GNSSSDR puppet for the sincos kernel.
 * \authors <ul>
 *          <li> Carles Fernandez Prades 2016 cfernandez at cttc dot cat
 *          </ul>
 *
 * VOLK_GNSSSDR puppet for integrating the sincos kernel into the test system
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef INCLUDED_volk_gnsssdr_s32f_sincospuppet_32fc_H
#define INCLUDED_volk_gnsssdr_s32f_sincospuppet_32fc_H


#include "volk_gnsssdr/volk_gnsssdr_s32f_sincos_32fc.h"
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
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


#ifdef LV_HAVE_NEON
static inline void volk_gnsssdr_s32f_sincospuppet_32fc_neon(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    float phase[1];
    phase[0] = 3;
    volk_gnsssdr_s32f_sincos_32fc_neon(out, phase_inc, phase, num_points);
}
#endif /* LV_HAVE_NEON  */

#endif /* INCLUDED_volk_gnsssdr_s32f_sincospuppet_32fc_H */
