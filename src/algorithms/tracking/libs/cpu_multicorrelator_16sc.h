/*!
 * \file cpu_multicorrelator_16sc.h
 * \brief High optimized CPU vector multiTAP correlator class for lv_16sc_t (short int complex)
 * \authors <ul>
 *          <li> Javier Arribas, 2016. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_CPU_MULTICORRELATOR_16SC_H_
#define GNSS_SDR_CPU_MULTICORRELATOR_16SC_H_

#include <volk_gnsssdr/volk_gnsssdr.h>


/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class cpu_multicorrelator_16sc
{
public:
    cpu_multicorrelator_16sc();
    ~cpu_multicorrelator_16sc();
    bool init(int max_signal_length_samples, int n_correlators);
    bool set_local_code_and_taps(int code_length_chips, const lv_16sc_t *local_code_in, float *shifts_chips);
    bool set_input_output_vectors(lv_16sc_t *corr_out, const lv_16sc_t *sig_in);
    void update_local_code(int correlator_length_samples, float rem_code_phase_chips, float code_phase_step_chips);
    bool Carrier_wipeoff_multicorrelator_resampler(float rem_carrier_phase_in_rad, float phase_step_rad, float rem_code_phase_chips, float code_phase_step_chips, int signal_length_samples);
    bool free();

private:
    // Allocate the device input vectors
    const lv_16sc_t *d_sig_in;
    lv_16sc_t **d_local_codes_resampled;
    const lv_16sc_t *d_local_code_in;
    lv_16sc_t *d_corr_out;
    float *d_shifts_chips;
    int d_code_length_chips;
    int d_n_correlators;
};


#endif /* GNSS_SDR_CPU_MULTICORRELATOR_H_ */
