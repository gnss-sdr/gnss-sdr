/*!
 * \file volk_gnsssdr_16ic_resamplerfastpuppet_16ic.h
 * \brief VOLK_GNSSSDR puppet for the 16-bit complex vector resampler kernel.
 * \authors <ul>
 *          <li> Carles Fernandez Prades 2016 cfernandez at cttc dot cat
 *          </ul>
 *
 * VOLK_GNSSSDR puppet for integrating the resampler into the test system
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

#ifndef INCLUDED_volk_gnsssdr_16ic_resamplerfastpuppet_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_resamplerfastpuppet_16ic_H

#include "volk_gnsssdr/volk_gnsssdr_16ic_resampler_fast_16ic.h"


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_16ic_resamplerfastpuppet_16ic_generic(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    float rem_code_phase_chips = -0.123;
    float code_phase_step_chips = 0.1;
    int code_length_chips = 1023;
    volk_gnsssdr_16ic_resampler_fast_16ic_generic(result, local_code, rem_code_phase_chips, code_phase_step_chips, code_length_chips, num_points);
}

#endif /* LV_HAVE_GENERIC */

#ifdef LV_HAVE_SSE2
static inline void volk_gnsssdr_16ic_resamplerfastpuppet_16ic_a_sse2(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    float rem_code_phase_chips = -0.123;
    float code_phase_step_chips = 0.1;
    int code_length_chips = 1023;
    volk_gnsssdr_16ic_resampler_fast_16ic_a_sse2(result, local_code, rem_code_phase_chips, code_phase_step_chips, code_length_chips, num_points);
}

#endif /* LV_HAVE_SSE2 */

#ifdef LV_HAVE_SSE2

static inline void volk_gnsssdr_16ic_resamplerfastpuppet_16ic_u_sse2(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    float rem_code_phase_chips = -0.123;
    float code_phase_step_chips = 0.1;
    int code_length_chips = 1023;
    volk_gnsssdr_16ic_resampler_fast_16ic_u_sse2(result, local_code, rem_code_phase_chips, code_phase_step_chips, code_length_chips, num_points);
}

#endif /* LV_HAVE_SSE2 */

#ifdef LV_HAVE_NEONV7

static inline void volk_gnsssdr_16ic_resamplerfastpuppet_16ic_neon(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    float rem_code_phase_chips = -0.123;
    float code_phase_step_chips = 0.1;
    int code_length_chips = 1023;
    volk_gnsssdr_16ic_resampler_fast_16ic_neon(result, local_code, rem_code_phase_chips, code_phase_step_chips, code_length_chips, num_points);
}

#endif /* LV_HAVE_NEONV7 */

#endif  // INCLUDED_volk_gnsssdr_16ic_resamplerfastpuppet_16ic_H
