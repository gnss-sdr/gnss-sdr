/*!
 * \file volk_gnsssdr_16ic_resamplerxnpuppet_16ic.h
 * \brief VOLK_GNSSSDR puppet for the multiple 16-bit complex vector resampler kernel.
 * \authors <ul>
 *          <li> Carles Fernandez Prades 2016 cfernandez at cttc dot cat
 *          </ul>
 *
 * VOLK_GNSSSDR puppet for integrating the multiple resampler into the test system
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

#ifndef INCLUDED_volk_gnsssdr_16ic_resamplerxnpuppet_16ic_H
#define INCLUDED_volk_gnsssdr_16ic_resamplerxnpuppet_16ic_H

#include "volk_gnsssdr/volk_gnsssdr_16ic_xn_resampler_16ic_xn.h"
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <string.h>

#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_16ic_resamplerxnpuppet_16ic_generic(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    int code_length_chips = 2046;
    float code_phase_step_chips = ((float)(code_length_chips) + 0.1) / ((float)num_points);
    int num_out_vectors = 3;
    unsigned int n;
    float rem_code_phase_chips = -0.234;
    float shifts_chips[3] = {-0.1, 0.0, 0.1};
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }

    volk_gnsssdr_16ic_xn_resampler_16ic_xn_generic(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, shifts_chips, code_length_chips, num_out_vectors, num_points);

    memcpy((lv_16sc_t*)result, (lv_16sc_t*)result_aux[0], sizeof(lv_16sc_t) * num_points);

    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif /* LV_HAVE_GENERIC */


#ifdef LV_HAVE_SSE3
static inline void volk_gnsssdr_16ic_resamplerxnpuppet_16ic_a_sse3(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    int code_length_chips = 2046;
    float code_phase_step_chips = ((float)(code_length_chips) + 0.1) / ((float)num_points);
    int num_out_vectors = 3;
    float rem_code_phase_chips = -0.234;
    unsigned int n;
    float shifts_chips[3] = {-0.1, 0.0, 0.1};
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }

    volk_gnsssdr_16ic_xn_resampler_16ic_xn_a_sse3(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, shifts_chips, code_length_chips, num_out_vectors, num_points);

    memcpy((lv_16sc_t*)result, (lv_16sc_t*)result_aux[0], sizeof(lv_16sc_t) * num_points);

    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif

#ifdef LV_HAVE_SSE3
static inline void volk_gnsssdr_16ic_resamplerxnpuppet_16ic_u_sse3(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    int code_length_chips = 2046;
    float code_phase_step_chips = ((float)(code_length_chips) + 0.1) / ((float)num_points);
    int num_out_vectors = 3;
    float rem_code_phase_chips = -0.234;
    unsigned int n;
    float shifts_chips[3] = {-0.1, 0.0, 0.1};
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }

    volk_gnsssdr_16ic_xn_resampler_16ic_xn_u_sse3(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, shifts_chips, code_length_chips, num_out_vectors, num_points);

    memcpy((lv_16sc_t*)result, (lv_16sc_t*)result_aux[0], sizeof(lv_16sc_t) * num_points);

    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif


#ifdef LV_HAVE_SSE4_1
static inline void volk_gnsssdr_16ic_resamplerxnpuppet_16ic_u_sse4_1(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    int code_length_chips = 2046;
    float code_phase_step_chips = ((float)(code_length_chips) + 0.1) / ((float)num_points);
    int num_out_vectors = 3;
    float rem_code_phase_chips = -0.234;
    unsigned int n;
    float shifts_chips[3] = {-0.1, 0.0, 0.1};
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }

    volk_gnsssdr_16ic_xn_resampler_16ic_xn_u_sse4_1(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, shifts_chips, code_length_chips, num_out_vectors, num_points);

    memcpy((lv_16sc_t*)result, (lv_16sc_t*)result_aux[0], sizeof(lv_16sc_t) * num_points);

    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif


#ifdef LV_HAVE_SSE4_1
static inline void volk_gnsssdr_16ic_resamplerxnpuppet_16ic_a_sse4_1(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    int code_length_chips = 2046;
    float code_phase_step_chips = ((float)(code_length_chips) + 0.1) / ((float)num_points);
    int num_out_vectors = 3;
    float rem_code_phase_chips = -0.234;
    unsigned int n;
    float shifts_chips[3] = {-0.1, 0.0, 0.1};
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }

    volk_gnsssdr_16ic_xn_resampler_16ic_xn_a_sse4_1(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, shifts_chips, code_length_chips, num_out_vectors, num_points);

    memcpy((lv_16sc_t*)result, (lv_16sc_t*)result_aux[0], sizeof(lv_16sc_t) * num_points);

    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif


#ifdef LV_HAVE_AVX
static inline void volk_gnsssdr_16ic_resamplerxnpuppet_16ic_u_avx(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    int code_length_chips = 2046;
    float code_phase_step_chips = ((float)(code_length_chips) + 0.1) / ((float)num_points);
    int num_out_vectors = 3;
    float rem_code_phase_chips = -0.234;
    unsigned int n;
    float shifts_chips[3] = {-0.1, 0.0, 0.1};
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }

    volk_gnsssdr_16ic_xn_resampler_16ic_xn_u_avx(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, shifts_chips, code_length_chips, num_out_vectors, num_points);

    memcpy((lv_16sc_t*)result, (lv_16sc_t*)result_aux[0], sizeof(lv_16sc_t) * num_points);

    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif


#ifdef LV_HAVE_AVX
static inline void volk_gnsssdr_16ic_resamplerxnpuppet_16ic_a_avx(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    int code_length_chips = 2046;
    float code_phase_step_chips = ((float)(code_length_chips) + 0.1) / ((float)num_points);
    int num_out_vectors = 3;
    float rem_code_phase_chips = -0.234;
    unsigned int n;
    float shifts_chips[3] = {-0.1, 0.0, 0.1};
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }

    volk_gnsssdr_16ic_xn_resampler_16ic_xn_a_avx(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, shifts_chips, code_length_chips, num_out_vectors, num_points);

    memcpy((lv_16sc_t*)result, (lv_16sc_t*)result_aux[0], sizeof(lv_16sc_t) * num_points);

    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif


#ifdef LV_HAVE_NEON
static inline void volk_gnsssdr_16ic_resamplerxnpuppet_16ic_neon(lv_16sc_t* result, const lv_16sc_t* local_code, unsigned int num_points)
{
    int code_length_chips = 2046;
    float code_phase_step_chips = ((float)(code_length_chips) + 0.1) / ((float)num_points);
    int num_out_vectors = 3;
    float rem_code_phase_chips = -0.234;
    unsigned int n;
    float shifts_chips[3] = {-0.1, 0.0, 0.1};
    lv_16sc_t** result_aux = (lv_16sc_t**)volk_gnsssdr_malloc(sizeof(lv_16sc_t*) * num_out_vectors, volk_gnsssdr_get_alignment());

    for (n = 0; n < num_out_vectors; n++)
        {
            result_aux[n] = (lv_16sc_t*)volk_gnsssdr_malloc(sizeof(lv_16sc_t) * num_points, volk_gnsssdr_get_alignment());
        }

    volk_gnsssdr_16ic_xn_resampler_16ic_xn_neon(result_aux, local_code, rem_code_phase_chips, code_phase_step_chips, shifts_chips, code_length_chips, num_out_vectors, num_points);

    memcpy((lv_16sc_t*)result, (lv_16sc_t*)result_aux[0], sizeof(lv_16sc_t) * num_points);

    for (n = 0; n < num_out_vectors; n++)
        {
            volk_gnsssdr_free(result_aux[n]);
        }
    volk_gnsssdr_free(result_aux);
}

#endif

#endif  // INCLUDED_volk_gnsssdr_16ic_resamplerpuppet_16ic_H
