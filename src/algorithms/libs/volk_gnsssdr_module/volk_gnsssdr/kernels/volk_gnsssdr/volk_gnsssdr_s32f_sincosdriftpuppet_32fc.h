/*!
 * \file volk_gnsssdr_s32f_sincosdriftpuppet_32fc.h
 * \brief VOLK_GNSSSDR puppet for the sincos kernel with a large phase increment.
 * \authors <ul>
 *          <li> Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *          </ul>
 *
 * VOLK_GNSSSDR puppet for testing the volk_gnsssdr_s32f_sincos_32fc kernel
 * with a phase increment of several radians per sample, as happens with the
 * FDMA carrier offsets of GLONASS signals in the acquisition Doppler grid.
 *
 * The generic implementation of this puppet is a double-precision reference
 * that does not call the kernel, so the QA system checks every implementation
 * of the kernel against it. Without a modulo-2pi reduction of the phase
 * accumulator, the single-precision accumulated phase reaches 1e4-1e5 rad
 * over a test vector, its resolution drops to 1e-3-1e-2 rad, and the phase
 * increment is effectively rounded, which shifts the frequency of the
 * generated carrier by hundreds of Hz and makes this test fail.
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef INCLUDED_volk_gnsssdr_s32f_sincosdriftpuppet_32fc_H
#define INCLUDED_volk_gnsssdr_s32f_sincosdriftpuppet_32fc_H


#include "volk_gnsssdr/volk_gnsssdr_s32f_sincos_32fc.h"
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <math.h>

/* The scalar provided by the QA system is ignored. The phase increment is
 * fixed to the carrier offset of the GLONASS L1 frequency channel k = -7
 * (-3.9375 MHz, plus 1 kHz of Doppler) sampled at 6 Msps, with the sign that
 * the acquisition uses, and the initial phase is fixed, so that the test is
 * deterministic and exercises the failure it guards against. */
#define VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_INC (-4.1243877f)
#define VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_0 (3.0f)


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_s32f_sincosdriftpuppet_32fc_generic(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    /* Double-precision reference: out[n] = exp(j * (phase_0 + n * phase_inc)),
     * with the increment taken exactly as the single-precision value that the
     * kernel receives. */
    (void)phase_inc;
    const double inc = (double)VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_INC;
    const double two_pi = 6.283185307179586;
    double phase = (double)VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_0;
    unsigned int i;
    for (i = 0; i < num_points; i++)
        {
            out[i] = lv_cmake((float)cos(phase), (float)sin(phase));
            phase += inc;
            phase -= two_pi * floor(phase / two_pi + 0.5);
        }
}
#endif /* LV_HAVE_GENERIC  */


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_s32f_sincosdriftpuppet_32fc_generic_float(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    (void)phase_inc;
    float phase[1];
    phase[0] = VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_0;
    volk_gnsssdr_s32f_sincos_32fc_generic(out, VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_INC, phase, num_points);
}
#endif /* LV_HAVE_GENERIC  */


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_s32f_sincosdriftpuppet_32fc_generic_fxpt(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    (void)phase_inc;
    float phase[1];
    phase[0] = VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_0;
    volk_gnsssdr_s32f_sincos_32fc_generic_fxpt(out, VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_INC, phase, num_points);
}
#endif /* LV_HAVE_GENERIC  */


#ifdef LV_HAVE_SSE2
static inline void volk_gnsssdr_s32f_sincosdriftpuppet_32fc_a_sse2(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    (void)phase_inc;
    float phase[1];
    phase[0] = VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_0;
    volk_gnsssdr_s32f_sincos_32fc_a_sse2(out, VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_INC, phase, num_points);
}
#endif /* LV_HAVE_SSE2  */


#ifdef LV_HAVE_SSE2
static inline void volk_gnsssdr_s32f_sincosdriftpuppet_32fc_u_sse2(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    (void)phase_inc;
    float phase[1];
    phase[0] = VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_0;
    volk_gnsssdr_s32f_sincos_32fc_u_sse2(out, VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_INC, phase, num_points);
}
#endif /* LV_HAVE_SSE2  */


#ifdef LV_HAVE_AVX2
static inline void volk_gnsssdr_s32f_sincosdriftpuppet_32fc_a_avx2(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    (void)phase_inc;
    float phase[1];
    phase[0] = VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_0;
    volk_gnsssdr_s32f_sincos_32fc_a_avx2(out, VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_INC, phase, num_points);
}
#endif /* LV_HAVE_AVX2  */


#ifdef LV_HAVE_AVX2
static inline void volk_gnsssdr_s32f_sincosdriftpuppet_32fc_u_avx2(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    (void)phase_inc;
    float phase[1];
    phase[0] = VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_0;
    volk_gnsssdr_s32f_sincos_32fc_u_avx2(out, VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_INC, phase, num_points);
}
#endif /* LV_HAVE_AVX2  */


#ifdef LV_HAVE_NEON
static inline void volk_gnsssdr_s32f_sincosdriftpuppet_32fc_neon(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    (void)phase_inc;
    float phase[1];
    phase[0] = VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_0;
    volk_gnsssdr_s32f_sincos_32fc_neon(out, VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_INC, phase, num_points);
}
#endif /* LV_HAVE_NEON  */


#ifdef LV_HAVE_RVV
static inline void volk_gnsssdr_s32f_sincosdriftpuppet_32fc_rvv(lv_32fc_t* out, const float phase_inc, unsigned int num_points)
{
    (void)phase_inc;
    float phase[1];
    phase[0] = VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_0;
    volk_gnsssdr_s32f_sincos_32fc_rvv(out, VOLK_GNSSSDR_SINCOSDRIFTPUPPET_PHASE_INC, phase, num_points);
}
#endif /* LV_HAVE_RVV  */

#endif /* INCLUDED_volk_gnsssdr_s32f_sincosdriftpuppet_32fc_H */
