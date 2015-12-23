/*!
 * \file cpu_multicorrelator.cc
 * \brief High optimized CPU vector multiTAP correlator class
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * Class that implements a high optimized vector multiTAP correlator class for CPUs
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

#include "cpu_multicorrelator.h"
#include <cmath>
#include <iostream>
#include <gnuradio/fxpt.h>  // fixed point sine and cosine
#include <volk/volk.h>


bool cpu_multicorrelator::init(
		int max_signal_length_samples,
		int n_correlators
		)
{
    // ALLOCATE MEMORY FOR INTERNAL vectors
    size_t size = max_signal_length_samples * sizeof(std::complex<float>);

    // NCO signal
    d_nco_in = static_cast<std::complex<float>*>(volk_malloc(size, volk_get_alignment()));

    // Doppler-free signal
    d_sig_doppler_wiped = static_cast<std::complex<float>*>(volk_malloc(size, volk_get_alignment()));

    d_local_codes_resampled = new std::complex<float>*[n_correlators];
    for (int n = 0; n < n_correlators; n++)
        {
            d_local_codes_resampled[n] = static_cast<std::complex<float>*>(volk_malloc(size, volk_get_alignment()));
        }
    d_n_correlators = n_correlators;
    return true;
}



bool cpu_multicorrelator::set_local_code_and_taps(
		int code_length_chips,
		const std::complex<float>* local_code_in,
		float *shifts_chips
		)
{
    d_local_code_in = local_code_in;
    d_shifts_chips = shifts_chips;
    d_code_length_chips = code_length_chips;
    return true;
}


bool cpu_multicorrelator::set_input_output_vectors(std::complex<float>* corr_out, const std::complex<float>* sig_in)
{
    // Save CPU pointers
    d_sig_in = sig_in;
    d_corr_out = corr_out;
    return true;
}



void cpu_multicorrelator::update_local_code(int correlator_length_samples,float rem_code_phase_chips, float code_phase_step_chips)
{
    float local_code_chip_index;
    for (int current_correlator_tap = 0; current_correlator_tap < d_n_correlators; current_correlator_tap++)
        {
            for (int n = 0; n < correlator_length_samples; n++)
                {
                    // resample code for current tap
                    local_code_chip_index = std::fmod(code_phase_step_chips*static_cast<float>(n)+ d_shifts_chips[current_correlator_tap] - rem_code_phase_chips, d_code_length_chips);
                    //Take into account that in multitap correlators, the shifts can be negative!
                    if (local_code_chip_index < 0.0) local_code_chip_index += d_code_length_chips;
                    d_local_codes_resampled[current_correlator_tap][n] = d_local_code_in[static_cast<int>(round(local_code_chip_index))];
                }
        }
}


void cpu_multicorrelator::update_local_carrier(int correlator_length_samples, float rem_carr_phase_rad, float phase_step_rad)
{
    float sin_f, cos_f;
    int phase_step_rad_i = gr::fxpt::float_to_fixed(phase_step_rad);
    int phase_rad_i = gr::fxpt::float_to_fixed(rem_carr_phase_rad);

    for(int i = 0; i < correlator_length_samples; i++)
        {
            gr::fxpt::sincos(phase_rad_i, &sin_f, &cos_f);
            d_nco_in[i] = std::complex<float>(cos_f, -sin_f);
            phase_rad_i += phase_step_rad_i;
        }
}

bool cpu_multicorrelator::Carrier_wipeoff_multicorrelator_resampler(
        float rem_carrier_phase_in_rad,
        float phase_step_rad,
        float rem_code_phase_chips,
        float code_phase_step_chips,
        int signal_length_samples)
{
    //update_local_carrier(signal_length_samples, rem_carrier_phase_in_rad, phase_step_rad); //replaced by VOLK phase rotator
    //volk_32fc_x2_multiply_32fc(d_sig_doppler_wiped, d_sig_in, d_nco_in, signal_length_samples); //replaced by VOLK phase rotator

    lv_32fc_t phase_offset_as_complex[1]= lv_cmake(std::cos(rem_carrier_phase_in_rad),-std::sin(rem_carrier_phase_in_rad));
    volk_32fc_s32fc_x2_rotator_32fc(d_sig_doppler_wiped, d_sig_in, std::exp(lv_32fc_t(0, -phase_step_rad)),phase_offset_as_complex, signal_length_samples);
    update_local_code(signal_length_samples,rem_code_phase_chips, code_phase_step_chips);
    for (int current_correlator_tap = 0; current_correlator_tap < d_n_correlators; current_correlator_tap++)
        {
            volk_32fc_x2_dot_prod_32fc(&d_corr_out[current_correlator_tap], d_sig_doppler_wiped, d_local_codes_resampled[current_correlator_tap], signal_length_samples);
        }
    return true;
}


cpu_multicorrelator::cpu_multicorrelator()
{
    d_sig_in = NULL;
    d_nco_in = NULL;
    d_sig_doppler_wiped = NULL;
    d_local_code_in = NULL;
    d_shifts_chips = NULL;
    d_corr_out = NULL;
    d_local_codes_resampled = NULL;
    d_code_length_chips = 0;
    d_n_correlators = 0;
}

bool cpu_multicorrelator::free()
{
    // Free memory
    if (d_sig_doppler_wiped != NULL) volk_free(d_sig_doppler_wiped);
    if (d_nco_in != NULL) volk_free(d_nco_in);
    for (int n = 0; n < d_n_correlators; n++)
        {
            volk_free(d_local_codes_resampled[n]);
        }
    delete d_local_codes_resampled;
    return true;
}

