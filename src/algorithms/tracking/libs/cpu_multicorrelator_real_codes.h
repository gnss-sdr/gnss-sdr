/*!
 * \file cpu_multicorrelator_real_codes.h
 * \brief Highly optimized CPU vector multiTAP correlator class using real-valued local codes
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          <li> Cillian O'Driscoll, 2017, cillian.odriscoll(at)gmail.com
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

#ifndef GNSS_SDR_CPU_MULTICORRELATOR_REAL_CODES_H
#define GNSS_SDR_CPU_MULTICORRELATOR_REAL_CODES_H


#include <complex>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class Cpu_Multicorrelator_Real_Codes
{
public:
    Cpu_Multicorrelator_Real_Codes();
    void set_high_dynamics_resampler(bool use_high_dynamics_resampler);
    ~Cpu_Multicorrelator_Real_Codes();
    bool init(int max_signal_length_samples, int n_correlators);
    bool set_local_code_and_taps(int code_length_chips, const float *local_code_in, float *shifts_chips);
    bool set_input_output_vectors(std::complex<float> *corr_out, const std::complex<float> *sig_in);
    void update_local_code(int correlator_length_samples, float rem_code_phase_chips, float code_phase_step_chips, float code_phase_rate_step_chips = 0.0);
    bool Carrier_wipeoff_multicorrelator_resampler(float rem_carrier_phase_in_rad, float phase_step_rad, float phase_rate_step_rad, float rem_code_phase_chips, float code_phase_step_chips, float code_phase_rate_step_chips, int signal_length_samples);
    bool Carrier_wipeoff_multicorrelator_resampler(float rem_carrier_phase_in_rad, float phase_step_rad, float rem_code_phase_chips, float code_phase_step_chips, float code_phase_rate_step_chips, int signal_length_samples);
    bool free();

private:
    // Allocate the device input vectors
    const std::complex<float> *d_sig_in;
    const float *d_local_code_in;
    std::complex<float> *d_corr_out;
    float **d_local_codes_resampled;
    float *d_shifts_chips;
    int d_code_length_chips;
    int d_n_correlators;
    bool d_use_high_dynamics_resampler;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CPU_MULTICORRELATOR_REAL_CODES_H
