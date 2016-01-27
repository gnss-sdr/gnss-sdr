#ifndef INCLUDED_volk_gnsssdr_16ic_rotatorpuppet_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_rotatorpuppet_16ic_H


#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include "volk_gnsssdr/volk_gnsssdr_16ic_s32fc_x2_rotator_16ic.h"


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_rotatorpuppet_16ic_generic(lv_16sc_t* outVector, const lv_16sc_t* inVector, unsigned int num_points)
{
    lv_32fc_t phase[1] = {lv_cmake(.3, 0.95393)};
    const lv_32fc_t phase_inc = lv_cmake(.1, 0.01);
    volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_generic(outVector, inVector, phase_inc, phase, num_points);
}

#endif /* LV_HAVE_GENERIC */

#ifdef LV_HAVE_NEON

static inline void volk_gnsssdr_16ic_rotatorpuppet_16ic_neon(lv_16sc_t* outVector, const lv_16sc_t* inVector, unsigned int num_points)
{
    lv_32fc_t phase[1] = {lv_cmake(.3, 0.95393)};
    const lv_32fc_t phase_inc = lv_cmake(.1, 0.01);
    volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_neon(outVector, inVector, phase_inc, phase, num_points);
}

#endif /* LV_HAVE_NEON */


#endif /* INCLUDED_volk_gnsssdr_16ic_rotatorpuppet_16ic_H */
