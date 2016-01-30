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

#ifdef LV_HAVE_NEON

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

#endif /* LV_HAVE_NEON */


#endif /* INCLUDED_volk_gnsssdr_16ic_rotatorpuppet_16ic_H */
