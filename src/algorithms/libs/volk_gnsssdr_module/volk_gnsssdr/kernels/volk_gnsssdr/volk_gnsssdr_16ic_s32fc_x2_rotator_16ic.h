/*!
 * \file volk_16ic_s32fc_x2_rotator_16ic.h
 * \brief Volk protokernel: rotates a 16 bits complex vector
 * \authors <ul>
 *          <li> Carles Fernandez-Prades, 2015  cfernandez at cttc.es
 *          </ul>
 *
 * Volk protokernel that rotates a 16-bit complex vector
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


#ifndef INCLUDED_volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_H

#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <math.h>
#define ROTATOR_RELOAD 512


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_generic(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    unsigned int i = 0;
    int j = 0;
    for(i = 0; i < (unsigned int)(num_points / ROTATOR_RELOAD); ++i)
        {
            for(j = 0; j < ROTATOR_RELOAD; ++j)
                {
                    *outVector++ = *inVector++ * (*phase);
                    (*phase) *= phase_inc;
                }
#ifdef __cplusplus
            (*phase) /= std::abs((*phase));
#else
            //(*phase) /= cabsf((*phase));
            (*phase) /= hypotf(lv_creal(*phase), lv_cimag(*phase));
#endif
        }
    for(i = 0; i < num_points % ROTATOR_RELOAD; ++i)
        {
            *outVector++ = *inVector++ * (*phase);
            (*phase) *= phase_inc;
        }
}

#endif /* LV_HAVE_GENERIC */

#ifdef LV_HAVE_NEON
#include <arm.neon.h>
static inline void volk_gnsssdr_16ic_s32fc_x2_rotator_16ic_neon(lv_16sc_t* outVector, const lv_16sc_t* inVector, const lv_32fc_t phase_inc, lv_32fc_t* phase, unsigned int num_points)
{
    unsigned int i = 0;
    int j = 0;
    for(i = 0; i < (unsigned int)(num_points / ROTATOR_RELOAD); ++i)
        {
            for(j = 0; j < ROTATOR_RELOAD; ++j)
                {
                    *outVector++ = *inVector++ * (*phase);
                    (*phase) *= phase_inc;
                }
#ifdef __cplusplus
            (*phase) /= std::abs((*phase));
#else
            //(*phase) /= cabsf((*phase));
            (*phase) /= hypotf(lv_creal(*phase), lv_cimag(*phase));
#endif
        }
    for(i = 0; i < num_points % ROTATOR_RELOAD; ++i)
        {
            *outVector++ = *inVector++ * (*phase);
            (*phase) *= phase_inc;
        }
}

#endif /* LV_HAVE_NEON */


#endif

