/*!
 * \file volk_gnsssdr_16ic_rotatorpuppet_16ic.h
 * \brief VOLK_GNSSSDR puppet for the 16-bit complex rotator kernel.
 * \authors <ul>
 *          <li> Carles Fernandez Prades 2016 cfernandez at cttc dot cat
 *          </ul>
 *
 * VOLK_GNSSSDR puppet for integrating the rotator into the test system
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef INCLUDED_volk_gnsssdr_16ic_rotatorpuppet_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_rotatorpuppet_16ic_H


#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include "volk_gnsssdr/volk_gnsssdr_16ic_s32fc_x2_rotator_16ic.h"
#include <math.h>


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_16ic_rotatorpuppet_16ic_generic(lv_16sc_t* outVector, const lv_16sc_t* inVector, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.123;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), -sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad));
    volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_generic(outVector, inVector, phase_inc[0], phase, num_points);
}

#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_16ic_rotatorpuppet_16ic_generic_reload(lv_16sc_t* outVector, const lv_16sc_t* inVector, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.123;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), -sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad));
    volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_generic_reload(outVector, inVector, phase_inc[0], phase, num_points);
}

#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSE3
static inline void volk_gnsssdr_16ic_rotatorpuppet_16ic_a_sse3(lv_16sc_t* outVector, const lv_16sc_t* inVector, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.123;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), -sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad));
    volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_a_sse3(outVector, inVector, phase_inc[0], phase, num_points);
}

#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_SSE3
static inline void volk_gnsssdr_16ic_rotatorpuppet_16ic_a_sse3_reload(lv_16sc_t* outVector, const lv_16sc_t* inVector, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.123;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), -sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad));
    volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_a_sse3_reload(outVector, inVector, phase_inc[0], phase, num_points);
}

#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_SSE3
static inline void volk_gnsssdr_16ic_rotatorpuppet_16ic_u_sse3(lv_16sc_t* outVector, const lv_16sc_t* inVector, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.123;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), -sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad));
    volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_u_sse3(outVector, inVector, phase_inc[0], phase, num_points);
}

#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_SSE3
static inline void volk_gnsssdr_16ic_rotatorpuppet_16ic_u_sse3_reload(lv_16sc_t* outVector, const lv_16sc_t* inVector, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.123;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), -sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad));
    volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_u_sse3_reload(outVector, inVector, phase_inc[0], phase, num_points);
}

#endif /* LV_HAVE_SSE3 */


#ifdef LV_HAVE_NEONV7
static inline void volk_gnsssdr_16ic_rotatorpuppet_16ic_neon(lv_16sc_t* outVector, const lv_16sc_t* inVector, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.123;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), -sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad));
    volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_neon(outVector, inVector, phase_inc[0], phase, num_points);
}

#endif /* LV_HAVE_NEONV7 */


#ifdef LV_HAVE_NEONV7
static inline void volk_gnsssdr_16ic_rotatorpuppet_16ic_neon_reload(lv_16sc_t* outVector, const lv_16sc_t* inVector, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.123;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), -sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), -sin(phase_step_rad));
    volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_neon_reload(outVector, inVector, phase_inc[0], phase, num_points);
}

#endif /* LV_HAVE_NEONV7 */


#endif /* INCLUDED_volk_gnsssdr_16ic_rotatorpuppet_16ic_H */
