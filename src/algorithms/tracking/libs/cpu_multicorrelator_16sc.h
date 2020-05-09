/*!
 * \file cpu_multicorrelator_16sc.h
 * \brief Highly optimized CPU vector multiTAP correlator class for lv_16sc_t (short int complex)
 * \authors <ul>
 *          <li> Javier Arribas, 2016. jarribas(at)cttc.es
 *          </ul>
 *
 * Class that implements a highly optimized vector multiTAP correlator class for CPUs
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CPU_MULTICORRELATOR_16SC_H
#define GNSS_SDR_CPU_MULTICORRELATOR_16SC_H

#include <volk_gnsssdr/volk_gnsssdr.h>


/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class Cpu_Multicorrelator_16sc
{
public:
    Cpu_Multicorrelator_16sc();
    ~Cpu_Multicorrelator_16sc();
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


#endif  // GNSS_SDR_CPU_MULTICORRELATOR_H
