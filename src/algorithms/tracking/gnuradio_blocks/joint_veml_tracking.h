/*!
 * \file joint_veml_tracking.h
 * \brief Implementation of a joint code + carrier tracking block.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 * \author Antonio Ramos, 2018. antonio.ramosdet(at)gmail.com
 * \author Gerald LaMountain, 2019. gerald(at)ece.neu.edu
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

#ifndef GNSS_SDR_JOINT_VEML_TRACKING_H
#define GNSS_SDR_JOINT_VEML_TRACKING_H

//#include "tracking_models.h"
#include "tracking_Gaussian_filter.h"

#include "cpu_multicorrelator_real_codes.h"
#include "dll_pll_conf.h"
#include "exponential_smoother.h"
#include "tracking_FLL_PLL_filter.h"  // for PLL/FLL filter
#include "tracking_loop_filter.h"     // for DLL filter
#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>   // for boost::shared_ptr
#include <gnuradio/block.h>       // for block
#include <gnuradio/gr_complex.h>  // for gr_complex
#include <gnuradio/types.h>       // for gr_vector_int, gr_vector...
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <pmt/pmt.h>              // for pmt_t
#include <cstdint>                // for int32_t
#include <fstream>                // for string, ofstream
#include <utility>                // for pair
#include <vector>

template <class OutputType>
class JointCarrierTransitionModel : public ModelFunction<OutputType>
{
public:
    // explicit CarrierTransitionModel(const float carrier_pdi) { pdi = carrier_pdi; };
    OutputType operator()(const arma::vec& input) override { 
        // input(0) = A_k;
        // input(1) = dZ_k;
        // input(2) = dPsi_k;
        // input(3) = dot_dPsi_k;
        // input(4) = ddot_dPsi_k)
        OutputType output = arma::zeros<OutputType>(5,1);
        output(0, 0) = input(0);
        output(1, 0) = input(1) + beta*t_samp*input(3) + 0.5*beta*std::pow(t_samp,2)*input(4);
        output(2, 0) = input(2) + t_samp*input(3) + 0.5*std::pow(t_samp,2)*input(4);
        output(3, 0) = input(3) + t_samp*input(4);
        output(4, 0) = input(4);

        return output;
    };
    void set_samp_length(const float ts) { t_samp = ts; };
    void set_chip_length(const float f_carr, const float tc) { beta = f_carr*tc; };

private:
    float t_samp;
    float beta;
};

template <class OutputType>
class JointCarrierMeasurementModel : public ModelFunction<OutputType>
{
public:
    OutputType operator()(const arma::vec& input) override {
        using namespace std::complex_literals;

        // Allocate memory for correlator outputs
        gr_complex *correlator_outs;
        correlator_outs = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

        // Do correlation
        // arma::mat input = arma::real(cx_input);
        do_correlation( correlator_outs, input(2), input(1) );
        
        uint32_t offset = 0;
        if (d_n_correlator_taps == 5) { offset++; }
        gr_complex R_Early = correlator_outs[offset];
        gr_complex R_Prompt = correlator_outs[offset+1];
        gr_complex R_Late = correlator_outs[offset+2];

        arma::cx_mat output = arma::zeros<arma::cx_mat>(3,1);
        gr_complex Y_Gain = static_cast<gr_complex>( input(0) * std::exp( 1i * input(2) ) );
        output(0, 0) = Y_Gain * R_Early;
        output(1, 0) = Y_Gain * R_Prompt;
        output(2, 0) = Y_Gain * R_Late;

        return output;
    };

    void setup_correlator( Cpu_Multicorrelator_Real_Codes* multicorrelator, const gr_complex* input_buffer, const int32_t n_correlator_taps)
    {

        // Correlator and input buffer pointers
        d_multicorrelator_cpu = multicorrelator;
        d_input_buffer = input_buffer;
        d_n_correlator_taps = n_correlator_taps;

    };
   
    void set_correlation_params( const double carrier_step, const double carrier_rate_step, const uint32_t samples_per_chip, const double code_phase_step, const double code_phase_rate_step)
    {
        // Correlation parameters
        d_carrier_phase_step_rad = carrier_step;
        d_carrier_phase_rate_step_rad = carrier_rate_step;
        d_code_samples_per_chip = samples_per_chip;
        d_code_phase_step_chips = code_phase_step;
        d_code_phase_rate_step_chips = code_phase_rate_step;
    };

private:

    void do_correlation(gr_complex* correlator_outs, double rem_carr_phase_rad, double rem_code_phase_chips)
    {
        // ################# CARRIER WIPEOFF AND CORRELATORS ##############################
        // perform carrier wipe-off and compute Early, Prompt and Late correlation
        d_multicorrelator_cpu->Carrier_wipeoff_multicorrelator_resampler(
            correlator_outs,
            d_input_buffer,
            rem_carr_phase_rad,
            d_carrier_phase_step_rad, d_carrier_phase_rate_step_rad,
            static_cast<float>(rem_code_phase_chips) * static_cast<float>(d_code_samples_per_chip),
            static_cast<float>(d_code_phase_step_chips) * static_cast<float>(d_code_samples_per_chip),
            static_cast<float>(d_code_phase_rate_step_chips) * static_cast<float>(d_code_samples_per_chip),
            d_vector_length);

    };
    
    Cpu_Multicorrelator_Real_Codes* d_multicorrelator_cpu;
    const gr_complex *d_input_buffer;

    int32_t d_n_correlator_taps;
    double d_carrier_phase_step_rad;
    double d_carrier_phase_rate_step_rad;
    uint32_t d_code_samples_per_chip;
    double d_code_phase_step_chips;
    double d_code_phase_rate_step_chips;
    double d_vector_length;
};


class Gnss_Synchro;
class joint_veml_tracking;

using joint_veml_tracking_sptr = boost::shared_ptr<joint_veml_tracking>;

joint_veml_tracking_sptr joint_veml_make_tracking(const Dll_Pll_Conf &conf_);

/*!
 * \brief This class implements a code DLL + carrier PLL tracking block.
 */
class joint_veml_tracking : public gr::block
{
public:
    ~joint_veml_tracking();

    void set_channel(uint32_t channel);
    void set_gnss_synchro(Gnss_Synchro *p_gnss_synchro);
    void start_tracking();
    void stop_tracking();

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

    JointCarrierTransitionModel<arma::vec> d_carrier_evolution_model;
    JointCarrierMeasurementModel<arma::cx_vec> d_correlator_output_model;

private:
    friend joint_veml_tracking_sptr joint_veml_make_tracking(const Dll_Pll_Conf &conf_);
    void msg_handler_telemetry_to_trk(const pmt::pmt_t &msg);
    joint_veml_tracking(const Dll_Pll_Conf &conf_);

    bool cn0_and_tracking_lock_status(double coh_integration_time_s);
    bool acquire_secondary();
    void do_correlation_step(const gr_complex *input_samples);
    void run_dll_pll(const gr_complex *input_samples);
    void update_tracking_vars();
    void clear_tracking_vars();
    void save_correlation_results();
    void log_data();
    int32_t save_matfile();

    // tracking configuration vars
    Dll_Pll_Conf trk_parameters;
    bool d_veml;
    bool d_cloop;
    uint32_t d_channel;
    Gnss_Synchro *d_acquisition_gnss_synchro;

    // Signal parameters
    bool d_secondary;
    double d_signal_carrier_freq;
    double d_code_period;
    double d_code_chip_rate;
    uint32_t d_secondary_code_length;
    uint32_t d_data_secondary_code_length;
    uint32_t d_code_length_chips;
    uint32_t d_code_samples_per_chip;  // All signals have 1 sample per chip code except Gal. E1 which has 2 (CBOC disabled) or 12 (CBOC enabled)
    int32_t d_symbols_per_bit;
    std::string systemName;
    std::string signal_type;
    std::string *d_secondary_code_string;
    std::string *d_data_secondary_code_string;
    std::string signal_pretty_name;

    int32_t *d_preambles_symbols;
    int32_t d_preamble_length_symbols;

    // dll filter buffer
    boost::circular_buffer<float> d_dll_filt_history;
    // tracking state machine
    int32_t d_state;

    // Integration period in samples
    int32_t d_correlation_length_ms;
    int32_t d_n_correlator_taps;

    float *d_tracking_code;
    float *d_data_code;
    float *d_local_code_shift_chips;
    float *d_prompt_data_shift;
    Cpu_Multicorrelator_Real_Codes multicorrelator_cpu;
    Cpu_Multicorrelator_Real_Codes correlator_data_cpu;  //for data channel

    /*  TODO: currently the multicorrelator does not support adding extra correlator
        with different local code, thus we need extra multicorrelator instance.
        Implement this functionality inside multicorrelator class
        as an enhancement to increase the performance
     */
    gr_complex *d_correlator_outs;
    gr_complex *d_Very_Early;
    gr_complex *d_Early;
    gr_complex *d_Prompt;
    gr_complex *d_Late;
    gr_complex *d_Very_Late;

    bool d_enable_extended_integration;
    int32_t d_extend_correlation_symbols_count;
    int32_t d_current_symbol;
    int32_t d_current_data_symbol;

    gr_complex d_VE_accu;
    gr_complex d_E_accu;
    gr_complex d_P_accu;
    gr_complex d_P_accu_old;
    gr_complex d_L_accu;
    gr_complex d_VL_accu;

    gr_complex d_P_data_accu;
    gr_complex *d_Prompt_Data;

    double d_code_phase_step_chips;
    double d_code_phase_rate_step_chips;
    boost::circular_buffer<std::pair<double, double>> d_code_ph_history;
    double d_carrier_phase_step_rad;
    double d_carrier_phase_rate_step_rad;
    boost::circular_buffer<std::pair<double, double>> d_carr_ph_history;
    
    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    float d_rem_carr_phase_rad;

    TrackingNonlinearFilter<CubatureFilter, arma::vec, arma::cx_vec> d_carrier_loop_ckf;
    Tracking_loop_filter d_code_loop_filter;
    Tracking_FLL_PLL_filter d_carrier_loop_filter;

    // acquisition
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;

    // tracking vars
    bool d_pull_in_transitory;
    bool d_corrected_doppler;
    double d_current_correlation_time_s;
    double d_carr_phase_error_hz;
    double d_carr_freq_error_hz;
    double d_carr_error_filt_hz;
    double d_code_error_chips;
    double d_code_error_filt_chips;
    double d_code_freq_chips;
    double d_carrier_doppler_hz;
    double d_acc_carrier_phase_rad;
    double d_rem_code_phase_chips;
    double T_chip_seconds;
    double T_prn_seconds;
    double T_prn_samples;
    double K_blk_samples;
    // PRN period in samples
    int32_t d_current_prn_length_samples;
    // processing samples counters
    uint64_t d_sample_counter;
    uint64_t d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int32_t d_cn0_estimation_counter;
    int32_t d_carrier_lock_fail_counter;
    int32_t d_code_lock_fail_counter;
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_carrier_lock_threshold;
    boost::circular_buffer<gr_complex> d_Prompt_circular_buffer;
    std::vector<gr_complex> d_Prompt_buffer;
    Exponential_Smoother d_cn0_smoother;
    Exponential_Smoother d_carrier_lock_test_smoother;
    // file dump
    std::ofstream d_dump_file;
    std::string d_dump_filename;
    bool d_dump;
    bool d_dump_mat;

    friend class JointCarrierTransitionModel<arma::vec>;
    friend class JointCarrierMeasurementModel<arma::cx_vec>;
};

#endif  // GNSS_SDR_JOINT_VEML_TRACKING_H
