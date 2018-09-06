/*!
 * \file cpu_multicorrelator_real_codes.cc
 * \brief Highly optimized CPU vector multiTAP correlator class with real-valued local codes
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          <li> Cillian O'Driscoll, 2017. cillian.odriscoll(at)gmail.com
 *          </ul>
 *
 * Class that implements a high optimized vector multiTAP correlator class for CPUs
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

#include "cpu_multicorrelator_real_codes.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cmath>

cpu_multicorrelator_real_codes::cpu_multicorrelator_real_codes()
{
    d_sig_in = nullptr;
    d_local_code_in = nullptr;
    d_shifts_chips = nullptr;
    d_corr_out = nullptr;
    d_local_codes_resampled = nullptr;
    d_code_length_chips = 0;
    d_n_correlators = 0;
    d_use_high_dynamics_resampler = true;
}


cpu_multicorrelator_real_codes::~cpu_multicorrelator_real_codes()
{
    if (d_local_codes_resampled != nullptr)
        {
            cpu_multicorrelator_real_codes::free();
        }
}


bool cpu_multicorrelator_real_codes::init(
    int max_signal_length_samples,
    int n_correlators)
{
    // ALLOCATE MEMORY FOR INTERNAL vectors
    size_t size = max_signal_length_samples * sizeof(float);

    d_local_codes_resampled = static_cast<float**>(volk_gnsssdr_malloc(n_correlators * sizeof(float*), volk_gnsssdr_get_alignment()));
    for (int n = 0; n < n_correlators; n++)
        {
            d_local_codes_resampled[n] = static_cast<float*>(volk_gnsssdr_malloc(size, volk_gnsssdr_get_alignment()));
        }
    d_n_correlators = n_correlators;
    return true;
}


bool cpu_multicorrelator_real_codes::set_local_code_and_taps(
    int code_length_chips,
    const float* local_code_in,
    float* shifts_chips)
{
    d_local_code_in = local_code_in;
    d_shifts_chips = shifts_chips;
    d_code_length_chips = code_length_chips;

    return true;
}


bool cpu_multicorrelator_real_codes::set_input_output_vectors(std::complex<float>* corr_out, const std::complex<float>* sig_in)
{
    // Save CPU pointers
    d_sig_in = sig_in;
    d_corr_out = corr_out;
    return true;
}


void cpu_multicorrelator_real_codes::update_local_code(int correlator_length_samples, float rem_code_phase_chips, float code_phase_step_chips, float code_phase_rate_step_chips)
{
    if (d_use_high_dynamics_resampler)
        {
            volk_gnsssdr_32f_xn_high_dynamics_resampler_32f_xn(d_local_codes_resampled,
                d_local_code_in,
                rem_code_phase_chips,
                code_phase_step_chips,
                code_phase_rate_step_chips,
                d_shifts_chips,
                d_code_length_chips,
                d_n_correlators,
                correlator_length_samples);
        }
    else
        {
            volk_gnsssdr_32f_xn_resampler_32f_xn(d_local_codes_resampled,
                d_local_code_in,
                rem_code_phase_chips,
                code_phase_step_chips,
                d_shifts_chips,
                d_code_length_chips,
                d_n_correlators,
                correlator_length_samples);
        }
}

// Overload Carrier_wipeoff_multicorrelator_resampler to ensure back compatibility
bool cpu_multicorrelator_real_codes::Carrier_wipeoff_multicorrelator_resampler(
    float rem_carrier_phase_in_rad,
    float phase_step_rad,
    float phase_rate_step_rad,
    float rem_code_phase_chips,
    float code_phase_step_chips,
    float code_phase_rate_step_chips,
    int signal_length_samples)
{
    update_local_code(signal_length_samples, rem_code_phase_chips, code_phase_step_chips, code_phase_rate_step_chips);
    // Regenerate phase at each call in order to avoid numerical issues
    lv_32fc_t phase_offset_as_complex[1];
    phase_offset_as_complex[0] = lv_cmake(std::cos(rem_carrier_phase_in_rad), -std::sin(rem_carrier_phase_in_rad));
    // call VOLK_GNSSSDR kernel
    if (d_use_high_dynamics_resampler)
        {
            volk_gnsssdr_32fc_32f_high_dynamic_rotator_dot_prod_32fc_xn(d_corr_out, d_sig_in, std::exp(lv_32fc_t(0.0, -phase_step_rad)), std::exp(lv_32fc_t(0.0, -phase_rate_step_rad)), phase_offset_as_complex, const_cast<const float**>(d_local_codes_resampled), d_n_correlators, signal_length_samples);
        }
    else
        {
            volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn(d_corr_out, d_sig_in, std::exp(lv_32fc_t(0.0, -phase_step_rad)), phase_offset_as_complex, const_cast<const float**>(d_local_codes_resampled), d_n_correlators, signal_length_samples);
        }
    return true;
}
// Overload Carrier_wipeoff_multicorrelator_resampler to ensure back compatibility
bool cpu_multicorrelator_real_codes::Carrier_wipeoff_multicorrelator_resampler(
    float rem_carrier_phase_in_rad,
    float phase_step_rad,
    float rem_code_phase_chips,
    float code_phase_step_chips,
    float code_phase_rate_step_chips,
    int signal_length_samples)
{
    update_local_code(signal_length_samples, rem_code_phase_chips, code_phase_step_chips, code_phase_rate_step_chips);
    // Regenerate phase at each call in order to avoid numerical issues
    lv_32fc_t phase_offset_as_complex[1];
    phase_offset_as_complex[0] = lv_cmake(std::cos(rem_carrier_phase_in_rad), -std::sin(rem_carrier_phase_in_rad));
    // call VOLK_GNSSSDR kernel
    volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn(d_corr_out, d_sig_in, std::exp(lv_32fc_t(0.0, -phase_step_rad)), phase_offset_as_complex, const_cast<const float**>(d_local_codes_resampled), d_n_correlators, signal_length_samples);
    return true;
}


bool cpu_multicorrelator_real_codes::free()
{
    // Free memory
    if (d_local_codes_resampled != nullptr)
        {
            for (int n = 0; n < d_n_correlators; n++)
                {
                    volk_gnsssdr_free(d_local_codes_resampled[n]);
                }
            volk_gnsssdr_free(d_local_codes_resampled);
            d_local_codes_resampled = nullptr;
        }
    return true;
}


void cpu_multicorrelator_real_codes::set_high_dynamics_resampler(
    bool use_high_dynamics_resampler)
{
    d_use_high_dynamics_resampler = use_high_dynamics_resampler;
}
