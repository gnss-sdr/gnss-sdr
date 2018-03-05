/*!
 * \file volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic.h
 * \brief Volk puppet for the multiple 16-bit complex dot product kernel.
 * \authors <ul>
 *          <li> Carles Fernandez Prades 2016 cfernandez at cttc dot cat
 *          </ul>
 *
 * Volk puppet for integrating the resampler into volk's test system
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

#ifndef INCLUDED_volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_H

#include "volk_gnsssdr/volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn.h"
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <string.h>

#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_generic(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    unsigned int n;
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }
    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_generic(result, local_code, phase_inc[0], phase, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // Generic


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_generic_reload(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    unsigned int n;
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }
    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_generic_reload(result, local_code, phase_inc[0], phase, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // Generic


#ifdef LV_HAVE_SSE3
static inline void volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_a_sse3(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    unsigned int n;
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_a_sse3(result, local_code, phase_inc[0], phase, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // SSE3


#ifdef LV_HAVE_SSE3
static inline void volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_a_sse3_reload(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    unsigned int n;
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_a_sse3_reload(result, local_code, phase_inc[0], phase, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // SSE3


#ifdef LV_HAVE_SSE3
static inline void volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_u_sse3(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    unsigned int n;
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_u_sse3(result, local_code, phase_inc[0], phase, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // SSE3


#ifdef LV_HAVE_AVX2
static inline void volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_a_avx2(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    unsigned int n;
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_a_avx2(result, local_code, phase_inc[0], phase, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // AVX2


#ifdef LV_HAVE_AVX2
static inline void volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_a_avx2_reload(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    unsigned int n;
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_a_avx2_reload(result, local_code, phase_inc[0], phase, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // AVX2


#ifdef LV_HAVE_AVX2
static inline void volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_u_avx2(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    unsigned int n;
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_a_avx2(result, local_code, phase_inc[0], phase, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // AVX2


#ifdef LV_HAVE_AVX2
static inline void volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_u_avx2_reload(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    unsigned int n;
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_a_avx2_reload(result, local_code, phase_inc[0], phase, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // AVX2


#ifdef LV_HAVE_NEON
static inline void volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_neon(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    unsigned int n;
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_neon(result, local_code, phase_inc[0], phase, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // NEON


#ifdef LV_HAVE_NEON
static inline void volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_neon_vma(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    // phases must be normalized. Phase rotator expects a complex exponential input!
    float rem_carrier_phase_in_rad = 0.345;
    float phase_step_rad = 0.1;
    lv_32fc_t phase[1];
    phase[0] = lv_cmake(cos(rem_carrier_phase_in_rad), sin(rem_carrier_phase_in_rad));
    lv_32fc_t phase_inc[1];
    phase_inc[0] = lv_cmake(cos(phase_step_rad), sin(phase_step_rad));
    unsigned int n;
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn_neon_vma(result, local_code, phase_inc[0], phase, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // NEON

#endif  // INCLUDED_volk_gnsssdr_16ic_x2_rotator_dotprodxnpuppet_16ic_H
