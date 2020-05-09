/*!
 * \file cpu_multicorrelator.h
 * \brief High optimized CPU vector multiTAP correlator class
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * Class that implements a high optimized vector multiTAP correlator class for CPUs
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

#ifndef GNSS_SDR_CPU_MULTICORRELATOR_H
#define GNSS_SDR_CPU_MULTICORRELATOR_H


#include <complex>

/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class Cpu_Multicorrelator
{
public:
    Cpu_Multicorrelator();
    ~Cpu_Multicorrelator();
    bool init(int max_signal_length_samples, int n_correlators);
    bool set_local_code_and_taps(int code_length_chips, const std::complex<float> *local_code_in, float *shifts_chips);
    bool set_input_output_vectors(std::complex<float> *corr_out, const std::complex<float> *sig_in);
    void update_local_code(int correlator_length_samples, float rem_code_phase_chips, float code_phase_step_chips);
    bool Carrier_wipeoff_multicorrelator_resampler(float rem_carrier_phase_in_rad, float phase_step_rad, float rem_code_phase_chips, float code_phase_step_chips, int signal_length_samples);
    bool free();

private:
    // Allocate the device input vectors
    const std::complex<float> *d_sig_in;
    std::complex<float> **d_local_codes_resampled;
    const std::complex<float> *d_local_code_in;
    std::complex<float> *d_corr_out;
    float *d_shifts_chips;
    int d_code_length_chips;
    int d_n_correlators;
};


#endif  // GNSS_SDR_CPU_MULTICORRELATOR_H
