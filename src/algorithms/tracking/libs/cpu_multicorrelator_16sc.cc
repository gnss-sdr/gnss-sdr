/*!
 * \file cpu_multicorrelator_16sc.cc
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

#include "cpu_multicorrelator_16sc.h"
#include <cmath>



bool cpu_multicorrelator_16sc::init(
        int max_signal_length_samples,
        int n_correlators)
{
    // ALLOCATE MEMORY FOR INTERNAL vectors
    size_t size = max_signal_length_samples * sizeof(lv_16sc_t);

    // NCO signal (not needed if the rotator+dot_product kernel is used)
    //d_nco_in = static_cast<lv_16sc_t*>(volk_gnsssdr_malloc(size, volk_gnsssdr_get_alignment()));
    // Doppler-free signal (not needed if the rotator+dot_product kernel is used)
    //d_sig_doppler_wiped = static_cast<lv_16sc_t*>(volk_gnsssdr_malloc(size, volk_gnsssdr_get_alignment()));

    d_n_correlators = n_correlators;
	d_tmp_code_phases_chips = static_cast<float*>(volk_gnsssdr_malloc(n_correlators*sizeof(float), volk_gnsssdr_get_alignment()));
    d_local_codes_resampled = new lv_16sc_t*[n_correlators];
    for (int n = 0; n < n_correlators; n++)
        {
            d_local_codes_resampled[n] = static_cast<lv_16sc_t*>(volk_gnsssdr_malloc(size, volk_gnsssdr_get_alignment()));
        }
    return true;
}



bool cpu_multicorrelator_16sc::set_local_code_and_taps(
		int code_length_chips,
		const lv_16sc_t* local_code_in,
		float *shifts_chips)
{
    d_local_code_in = local_code_in;
    d_shifts_chips = shifts_chips;
    d_code_length_chips = code_length_chips;
    return true;
}


bool cpu_multicorrelator_16sc::set_input_output_vectors(lv_16sc_t* corr_out, const lv_16sc_t* sig_in)
{
    // Save CPU pointers
    d_sig_in = sig_in;
    d_corr_out = corr_out;
    return true;
}

void cpu_multicorrelator_16sc::update_local_code(int correlator_length_samples,float rem_code_phase_chips, float code_phase_step_chips)
{

	for (int n = 0; n < d_n_correlators; n++)
	{
		d_tmp_code_phases_chips[n] = d_shifts_chips[n] - rem_code_phase_chips;
	}

	volk_gnsssdr_16ic_xn_resampler_16ic_xn(d_local_codes_resampled,
			d_local_code_in,
			d_tmp_code_phases_chips,
			code_phase_step_chips,
			correlator_length_samples,
			d_n_correlators,
			d_code_length_chips);
}


bool cpu_multicorrelator_16sc::Carrier_wipeoff_multicorrelator_resampler(
        float rem_carrier_phase_in_rad,
        float phase_step_rad,
        float rem_code_phase_chips,
        float code_phase_step_chips,
        int signal_length_samples)
{
    update_local_code(signal_length_samples, rem_code_phase_chips, code_phase_step_chips);
    lv_32fc_t phase_offset_as_complex[1];
    phase_offset_as_complex[0] = lv_cmake(std::cos(rem_carrier_phase_in_rad), -std::sin(rem_carrier_phase_in_rad));
    //replaced by integrated rotator + dot_product kernel
    //volk_gnsssdr_16ic_s32fc_x2_rotator_16ic(d_sig_doppler_wiped, d_sig_in, std::exp(lv_32fc_t(0, -phase_step_rad)), phase_offset_as_complex, signal_length_samples);
    //volk_gnsssdr_16ic_x2_dot_prod_16ic_xn(d_corr_out, d_sig_doppler_wiped, (const lv_16sc_t**)d_local_codes_resampled, d_n_correlators,  signal_length_samples);
    volk_gnsssdr_16ic_x2_rotator_dot_prod_16ic_xn(d_corr_out, d_sig_in, std::exp(lv_32fc_t(0, -phase_step_rad)), phase_offset_as_complex, (const lv_16sc_t**)d_local_codes_resampled, d_n_correlators,  signal_length_samples);

    return true;
}


cpu_multicorrelator_16sc::cpu_multicorrelator_16sc()
{
    d_sig_in = NULL;
    //d_nco_in = NULL;
    //d_sig_doppler_wiped = NULL;
    d_local_code_in = NULL;
    d_shifts_chips = NULL;
    d_corr_out = NULL;
    d_local_codes_resampled = NULL;
    d_code_length_chips = 0;
    d_n_correlators = 0;
}

bool cpu_multicorrelator_16sc::free()
{
    // Free memory
    //if (d_sig_doppler_wiped != NULL) volk_gnsssdr_free(d_sig_doppler_wiped);
    //if (d_nco_in != NULL) volk_gnsssdr_free(d_nco_in);
	if (d_tmp_code_phases_chips != NULL) volk_gnsssdr_free(d_tmp_code_phases_chips);
    for (int n = 0; n < d_n_correlators; n++)
        {
            volk_gnsssdr_free(d_local_codes_resampled[n]);
        }
    delete d_local_codes_resampled;
    return true;
}

