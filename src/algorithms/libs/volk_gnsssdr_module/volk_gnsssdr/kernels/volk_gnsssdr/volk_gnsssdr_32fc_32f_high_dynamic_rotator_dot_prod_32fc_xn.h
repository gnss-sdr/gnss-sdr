/*!
 * \file volk_gnsssdr_32fc_32f_high_dynamic_rotator_dot_prod_32fc_xn.h
 * \brief VOLK_GNSSSDR kernel: multiplies N complex (32-bit float per component) vectors
 * by a common vector, phase rotated with Doppler rate and accumulates the results in N float complex outputs.
 * \authors <ul>
 *          <li> Carles Fernandez, 2019 cfernandez@cttc.es
 *          <li> Javier Arribas, 2019 javiarribas@cttc.es
 *          </ul>
 *
 * VOLK_GNSSSDR kernel that multiplies N 32 bits complex vectors by a common vector, which is
 * phase-rotated by phase offset and phase increment, and accumulates the results
 * in N 32 bits float complex outputs.
 * It is optimized to perform the N tap correlation process in GNSS receivers.
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

/*!
 * \page volk_gnsssdr_32fc_32f_high_dynamic_rotator_dot_prod_32fc_xn
 *
 * \b Overview
 *
 * Rotates and multiplies the reference complex vector with an arbitrary number of other real vectors,
 * accumulates the results and stores them in the output vector.
 * The rotation is done at a variable rate per sample, from an initial \p phase offset.
 * This function can be used for Doppler wipe-off and multiple correlator in the presence of Doppler rate.
 *
 * <b>Dispatcher Prototype</b>
 * \code
 * void volk_gnsssdr_32fc_32f_high_dynamic_rotator_dot_prod_32fc_xn(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, const lv_32fc_t phase_inc_rate, lv_32fc_t* phase, const float** in_a, int num_a_vectors, unsigned int num_points);
 * \endcode
 *
 * \b Inputs
 * \li in_common:      Pointer to one of the vectors to be rotated, multiplied and accumulated (reference vector).
 * \li phase_inc:      Phase increment = lv_cmake(cos(phase_step_rad), sin(phase_step_rad))
 * \li phase_inc_rate: Phase increment rate = lv_cmake(cos(phase_step_rate_rad), sin(phase_step_rate_rad))
 * \li phase:          Initial phase = lv_cmake(cos(initial_phase_rad), sin(initial_phase_rad))
 * \li in_a:           Pointer to an array of pointers to multiple vectors to be multiplied and accumulated.
 * \li num_a_vectors:  Number of vectors to be multiplied by the reference vector and accumulated.
 * \li num_points:     Number of complex values to be multiplied together, accumulated and stored into \p result.
 *
 * \b Outputs
 * \li phase:         Final phase.
 * \li result:        Vector of \p num_a_vectors components with the multiple vectors of \p in_a rotated, multiplied by \p in_common and accumulated.
 *
 */

#ifndef INCLUDED_volk_gnsssdr_32fc_32f_high_dynamic_rotator_dot_prod_32fc_xn_H
#define INCLUDED_volk_gnsssdr_32fc_32f_high_dynamic_rotator_dot_prod_32fc_xn_H


#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_complex.h>
#include <volk_gnsssdr/volk_gnsssdr_malloc.h>
#include <math.h>


#ifdef LV_HAVE_GENERIC

static inline void volk_gnsssdr_32fc_32f_high_dynamic_rotator_dot_prod_32fc_xn_generic(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, const lv_32fc_t phase_inc_rate, lv_32fc_t* phase, const float** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_32fc_t tmp32_1;
    lv_32fc_t phase_doppler_rate = lv_cmake(1.0f, 0.0f);
    lv_32fc_t phase_doppler = (*phase);
    int n_vec;
    unsigned int n;
    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            result[n_vec] = lv_cmake(0.0f, 0.0f);
        }
    for (n = 0; n < num_points; n++)
        {
            // Regenerate phase
            if (n % 256 == 0)
                {
#ifdef __cplusplus
                    (*phase) /= std::abs((*phase));
#else
                    (*phase) /= hypotf(lv_creal(*phase), lv_cimag(*phase));
#endif
                }
            tmp32_1 = *in_common++ * (*phase);
            phase_doppler *= phase_inc;
            phase_doppler_rate = cpowf(phase_inc_rate, lv_cmake((float)(n * n), 0.0f));
            phase_doppler_rate /= hypotf(lv_creal(phase_doppler_rate), lv_cimag(phase_doppler_rate));
            (*phase) = phase_doppler * phase_doppler_rate;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    result[n_vec] += (tmp32_1 * in_a[n_vec][n]);
                }
        }
}
#endif


#ifdef LV_HAVE_GENERIC
static inline void volk_gnsssdr_32fc_32f_high_dynamic_rotator_dot_prod_32fc_xn_generic_arg(lv_32fc_t* result, const lv_32fc_t* in_common, const lv_32fc_t phase_inc, const lv_32fc_t phase_inc_rate, lv_32fc_t* phase, const float** in_a, int num_a_vectors, unsigned int num_points)
{
    lv_32fc_t tmp32_1;
    lv_32fc_t phase_doppler_rate = lv_cmake(1.0f, 0.0f);
    lv_32fc_t phase_doppler = (*phase);
    int n_vec;
    const float arga = cargf(phase_inc_rate);
    unsigned int n;
    for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
        {
            result[n_vec] = lv_cmake(0.0f, 0.0f);
        }
    for (n = 0; n < num_points; n++)
        {
            // Regenerate phase
            if (n % 256 == 0)
                {
#ifdef __cplusplus
                    (*phase) /= std::abs((*phase));
#else
                    (*phase) /= hypotf(lv_creal(*phase), lv_cimag(*phase));
#endif
                }
            tmp32_1 = *in_common++ * (*phase);
            phase_doppler *= phase_inc;
            const float theta = (float)(n * n) * arga;
            phase_doppler_rate = lv_cmake(cosf(theta), sinf(theta));
            (*phase) = phase_doppler * phase_doppler_rate;

            for (n_vec = 0; n_vec < num_a_vectors; n_vec++)
                {
                    result[n_vec] += (tmp32_1 * in_a[n_vec][n]);
                }
        }
}
#endif

#endif /* INCLUDED_volk_gnsssdr_32fc_32f_high_dynamic_rotator_dot_prod_32fc_xn_H */
