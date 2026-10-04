/*!
 * \file cpu_multicorrelator_real_codes.h
 * \brief Highly optimized CPU vector multiTAP correlator class using real-valued local codes
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          <li> Cillian O'Driscoll, 2017. cillian.odriscoll(at)gmail.com
 *          <li> Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *          </ul>
 *
 * Class that implements a highly optimized vector multiTAP correlator class for CPUs
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
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
    Cpu_Multicorrelator_Real_Codes() = default;
    void set_high_dynamics_resampler(bool use_high_dynamics_resampler);
    ~Cpu_Multicorrelator_Real_Codes();
    bool init(int max_signal_length_samples, int n_correlators);
    bool init(int max_signal_length_samples, int n_correlators, bool has_data_prompt);
    bool set_local_code_and_taps(int code_length_chips, const float *local_code_in, float *shifts_chips);
    bool set_input_output_vectors(std::complex<float> *corr_out, const std::complex<float> *sig_in);
    bool set_data_code_and_prompt_tap(int code_length_chips, const float *data_code_in, float *prompt_shift_chips);
    bool set_input_output_vectors(std::complex<float> *corr_out, std::complex<float> *data_prompt_out, const std::complex<float> *sig_in);
    void update_local_code(int correlator_length_samples, float rem_code_phase_chips, float code_phase_step_chips, float code_phase_rate_step_chips = 0.0);
    bool Carrier_wipeoff_multicorrelator_resampler(float rem_carrier_phase_in_rad, float phase_step_rad, float phase_rate_step_rad, float rem_code_phase_chips, float code_phase_step_chips, float code_phase_rate_step_chips, int signal_length_samples);
    bool Carrier_wipeoff_multicorrelator_resampler(float rem_carrier_phase_in_rad, float phase_step_rad, float rem_code_phase_chips, float code_phase_step_chips, float code_phase_rate_step_chips, int signal_length_samples);
    bool Carrier_wipeoff_multicorrelator_resampler_with_data_prompt(float rem_carrier_phase_in_rad, float phase_step_rad, float phase_rate_step_rad, float rem_code_phase_chips, float code_phase_step_chips, float code_phase_rate_step_chips, int signal_length_samples);
    bool free();

private:
    // Allocate the device input vectors
    const std::complex<float> *d_sig_in{nullptr};
    const float *d_local_code_in{nullptr};
    const float *d_data_code_in{nullptr};
    std::complex<float> *d_corr_out{nullptr};
    std::complex<float> *d_data_prompt_out{nullptr};
    std::complex<float> *d_corr_out_with_data_prompt{nullptr};
    float **d_local_codes_resampled{nullptr};
    float *d_shifts_chips{nullptr};
    float *d_data_prompt_shift_chips{nullptr};
    int d_code_length_chips{0};
    int d_data_code_length_chips{0};
    int d_n_correlators{0};
    int d_n_correlators_with_data_prompt{0};
    bool d_has_data_prompt{false};
    bool d_use_high_dynamics_resampler{true};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CPU_MULTICORRELATOR_REAL_CODES_H
