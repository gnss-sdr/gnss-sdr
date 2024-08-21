/*!
 * \file volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic.h
 * \brief VOLK_GNSSSDR puppet for the multiple 16-bit complex dot product kernel.
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

#ifndef INCLUDED_volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_H

#include "volk_gnsssdr/volk_gnsssdr_16ic_x2_dot_prod_16ic_xn.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <string.h>

#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_generic(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    int n;
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_generic(result, local_code, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif /* Generic */


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_generic_sat(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    int n;
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_generic_sat(result, local_code, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif /* Generic */


#ifdef LV_HAVE_SSE2
static inline void volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_a_sse2(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    int n;
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_a_sse2(result, local_code, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif /*  SSE2   */


#if LV_HAVE_SSE2

static inline void volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_u_sse2(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    int n;
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_u_sse2(result, local_code, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif /* LV_HAVE_SSE2  */


#if LV_HAVE_AVX2

static inline void volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_a_avx2(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    int n;
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_a_avx2(result, local_code, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif /* LV_HAVE_AVX2  */


#if LV_HAVE_AVX2

static inline void volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_u_avx2(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    int n;
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_u_avx2(result, local_code, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif /* LV_HAVE_AVX2  */


#ifdef LV_HAVE_NEON

static inline void volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_neon(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    int n;
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_neon(result, local_code, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // NEON


#ifdef LV_HAVE_NEON

static inline void volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_neon_vma(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    int n;
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_neon_vma(result, local_code, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // NEON

#ifdef LV_HAVE_NEON

static inline void volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_neon_optvma(lv_16sc_t* result, const lv_16sc_t* local_code, const lv_16sc_t* in, unsigned int num_points)
{
    int num_a_vectors = 3;
    lv_16sc_t** in_a = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_a_vectors, volk_gnsssdr_get_alignment());
    int n;
    for (n = 0; n < num_a_vectors; n++)
        {
            in_a[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
            memcpy((lv_16sc_t*)in_a[n], (lv_16sc_t*)in, sizeof(lv_16sc_t) * num_points);
        }

    volk_gnsssdr_16ic_x2_dot_prod_16ic_xn_neon_optvma(result, local_code, (const lv_16sc_t**)in_a, num_a_vectors, num_points);

    for (n = 0; n < num_a_vectors; n++)
        {
            volk_gnsssdr_free(in_a[n]);
        }
    volk_gnsssdr_free(in_a);
}

#endif  // NEON

#endif  // INCLUDED_volk_gnsssdr_16ic_x2_dotprodxnpuppet_16ic_H
