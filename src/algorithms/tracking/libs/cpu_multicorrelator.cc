/*!
 * \file cpu_multicorrelator.cc
 * \brief Highly optimized CPU vector multiTAP correlator class
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * Class that implements a highly optimized vector multiTAP correlator class for CPUs
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

#include "cpu_multicorrelator.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cmath>


Cpu_Multicorrelator::~Cpu_Multicorrelator()
{
    if (d_local_codes_resampled != nullptr)
        {
            Cpu_Multicorrelator::free();
        }
}


bool Cpu_Multicorrelator::init(
    int max_signal_length_samples,
    int n_correlators)
{
    // ALLOCATE MEMORY FOR INTERNAL vectors
    size_t size = max_signal_length_samples * sizeof(std::complex<float>);

    d_local_codes_resampled = static_cast<std::complex<float>**>(volk_gnsssdr_malloc(n_correlators * sizeof(std::complex<float>*), volk_gnsssdr_get_alignment()));
    for (int n = 0; n < n_correlators; n++)
        {
            d_local_codes_resampled[n] = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(size, volk_gnsssdr_get_alignment()));
        }
    d_n_correlators = n_correlators;
    return true;
}


bool Cpu_Multicorrelator::set_local_code_and_taps(
    int code_length_chips,
    const std::complex<float>* local_code_in,
    float* shifts_chips)
{
    d_local_code_in = local_code_in;
    d_shifts_chips = shifts_chips;
    d_code_length_chips = code_length_chips;
    return true;
}


bool Cpu_Multicorrelator::set_input_output_vectors(std::complex<float>* corr_out, const std::complex<float>* sig_in)
{
    // Save CPU pointers
    d_sig_in = sig_in;
    d_corr_out = corr_out;
    return true;
}


void Cpu_Multicorrelator::update_local_code(int correlator_length_samples, float rem_code_phase_chips, float code_phase_step_chips)
{
    volk_gnsssdr_32fc_xn_resampler_32fc_xn(d_local_codes_resampled,
        d_local_code_in,
        rem_code_phase_chips,
        code_phase_step_chips,
        d_shifts_chips,
        d_code_length_chips,
        d_n_correlators,
        correlator_length_samples);
}


bool Cpu_Multicorrelator::Carrier_wipeoff_multicorrelator_resampler(
    float rem_carrier_phase_in_rad,
    float phase_step_rad,
    float rem_code_phase_chips,
    float code_phase_step_chips,
    int signal_length_samples)
{
    update_local_code(signal_length_samples, rem_code_phase_chips, code_phase_step_chips);
    // Regenerate phase at each call in order to avoid numerical issues
    lv_32fc_t phase_offset_as_complex[1];
    phase_offset_as_complex[0] = lv_cmake(std::cos(rem_carrier_phase_in_rad), -std::sin(rem_carrier_phase_in_rad));
    // call VOLK_GNSSSDR kernel
    volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn(d_corr_out, d_sig_in, std::exp(lv_32fc_t(0, -phase_step_rad)), phase_offset_as_complex, const_cast<const lv_32fc_t**>(d_local_codes_resampled), d_n_correlators, signal_length_samples);
    return true;
}


bool Cpu_Multicorrelator::free()
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
