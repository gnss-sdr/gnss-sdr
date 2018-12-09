/*!
 * \file volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc.h
 * \brief Volk puppet for the multiple 16-bit complex dot product kernel.
 * \authors <ul>
 *          <li> Carles Fernandez Prades 2016 cfernandez at cttc dot cat
 *          </ul>
 *
 * Volk puppet for integrating the resampler into volk's test system
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

#ifndef INCLUDED_volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc_H
#define INCLUDED_volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc_H

#include "volk_gnsssdr/volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn.h"
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <string.h>

#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc_generic(lv_32fc_t* result, const lv_32fc_t* local_code, const lv_32fc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.25;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    int n;
    int num_a_vectors = 3;
    lv_32fc_t** in_a = (lv_32fc_t**)volk_gnsssdr_malloc(sizeof(lv_32fc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_32fc_t*)volk_gnsssdr_malloc(sizeof(lv_32fc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_32fc_t*)in_a[n], (lv_32fc_t*)in, sizeof(lv_32fc_t) * num_points);
        }
    volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_generic_reload(result, local_code, phase_inc[0], phase, (const lv_32fc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // Generic


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc_generic_reload(lv_32fc_t* result, const lv_32fc_t* local_code, const lv_32fc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.25;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    int n;
    int num_a_vectors = 3;
    lv_32fc_t** in_a = (lv_32fc_t**)volk_gnsssdr_malloc(sizeof(lv_32fc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_32fc_t*)volk_gnsssdr_malloc(sizeof(lv_32fc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_32fc_t*)in_a[n], (lv_32fc_t*)in, sizeof(lv_32fc_t) * num_points);
        }
    volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_generic_reload(result, local_code, phase_inc[0], phase, (const lv_32fc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // Generic


#ifdef LV_HAVE_SSE3
static inline void volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc_u_sse3(lv_32fc_t* result, const lv_32fc_t* local_code, const lv_32fc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.25;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    int n;
    int num_a_vectors = 3;
    lv_32fc_t** in_a = (lv_32fc_t**)volk_gnsssdr_malloc(sizeof(lv_32fc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_32fc_t*)volk_gnsssdr_malloc(sizeof(lv_32fc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_32fc_t*)in_a[n], (lv_32fc_t*)in, sizeof(lv_32fc_t) * num_points);
        }
    volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_u_sse3(result, local_code, phase_inc[0], phase, (const lv_32fc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // SSE3


#ifdef LV_HAVE_SSE3
static inline void volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc_a_sse3(lv_32fc_t* result, const lv_32fc_t* local_code, const lv_32fc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.25;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    int n;
    int num_a_vectors = 3;
    lv_32fc_t** in_a = (lv_32fc_t**)volk_gnsssdr_malloc(sizeof(lv_32fc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_32fc_t*)volk_gnsssdr_malloc(sizeof(lv_32fc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_32fc_t*)in_a[n], (lv_32fc_t*)in, sizeof(lv_32fc_t) * num_points);
        }
    volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_a_sse3(result, local_code, phase_inc[0], phase, (const lv_32fc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // SSE3


#ifdef LV_HAVE_AVX
static inline void volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc_u_avx(lv_32fc_t* result, const lv_32fc_t* local_code, const lv_32fc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.25;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    int n;
    int num_a_vectors = 3;
    lv_32fc_t** in_a = (lv_32fc_t**)volk_gnsssdr_malloc(sizeof(lv_32fc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_32fc_t*)volk_gnsssdr_malloc(sizeof(lv_32fc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_32fc_t*)in_a[n], (lv_32fc_t*)in, sizeof(lv_32fc_t) * num_points);
        }
    volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_u_avx(result, local_code, phase_inc[0], phase, (const lv_32fc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // AVX


#ifdef LV_HAVE_AVX
static inline void volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc_a_avx(lv_32fc_t* result, const lv_32fc_t* local_code, const lv_32fc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.25;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    int n;
    int num_a_vectors = 3;
    lv_32fc_t** in_a = (lv_32fc_t**)volk_gnsssdr_malloc(sizeof(lv_32fc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_32fc_t*)volk_gnsssdr_malloc(sizeof(lv_32fc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_32fc_t*)in_a[n], (lv_32fc_t*)in, sizeof(lv_32fc_t) * num_points);
        }
    volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_a_avx(result, local_code, phase_inc[0], phase, (const lv_32fc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // AVX


#ifdef LV_HAVE_NEONV7
static inline void volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc_neon(lv_32fc_t* result, const lv_32fc_t* local_code, const lv_32fc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.25;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    int n;
    int num_a_vectors = 3;
    lv_32fc_t** in_a = (lv_32fc_t**)volk_gnsssdr_malloc(sizeof(lv_32fc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_32fc_t*)volk_gnsssdr_malloc(sizeof(lv_32fc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_32fc_t*)in_a[n], (lv_32fc_t*)in, sizeof(lv_32fc_t) * num_points);
        }
    volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn_neon(result, local_code, phase_inc[0], phase, (const lv_32fc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // AVX


#endif  // INCLUDED_volk_gnsssdr_32fc_x2_rotator_dotprodxnpuppet_32fc_H
