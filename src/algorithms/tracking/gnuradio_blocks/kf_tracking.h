/*!
 * \file kf_tracking.cc
 * \brief Implementation of a Kalman filter based tracking with optional Vector
 * Tracking Loop message receiver block.
 * \author Javier Arribas, 2020. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_KF_TRACKING_H
#define GNSS_SDR_KF_TRACKING_H

#if ARMA_NO_BOUND_CHECKING
#define ARMA_NO_DEBUG 1
#endif

#include "cpu_multicorrelator_real_codes.h"
#include "exponential_smoother.h"
#include "gnss_block_interface.h"
#include "gnss_time.h"  // for timetags produced by File_Timestamp_Signal_Source
#include "kf_conf.h"
#include "tracking_FLL_PLL_filter.h"  // for PLL/FLL filter
#include "tracking_loop_filter.h"     // for DLL filter
#include <armadillo>
#include <boost/circular_buffer.hpp>
#include <gnuradio/block.h>                   // for block
#include <gnuradio/gr_complex.h>              // for gr_complex
#include <gnuradio/types.h>                   // for gr_vector_int, gr_vector...
#include <pmt/pmt.h>                          // for pmt_t
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <cstddef>                            // for size_t
#include <cstdint>                            // for int32_t
#include <fstream>                            // for ofstream
#include <memory>
#include <string>    // for string
#include <typeinfo>  // for typeid
#include <utility>   // for pair

class Gnss_Synchro;
class kf_tracking;

using kf_tracking_sptr = gnss_shared_ptr<kf_tracking>;

kf_tracking_sptr kf_make_tracking(const Kf_Conf &conf_);

/*!
 * \brief This class implements a code DLL + carrier PLL tracking block.
 */
class kf_tracking : public gr::block
{
public:
    ~kf_tracking();

    void set_channel(uint32_t channel);
    void set_gnss_synchro(Gnss_Synchro *p_gnss_synchro);
    void start_tracking();
    void stop_tracking();

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

private:
    friend kf_tracking_sptr kf_make_tracking(const Kf_Conf &conf_);
    explicit kf_tracking(const Kf_Conf &conf_);

    void init_kf(double acq_code_phase_chips, double acq_doppler_hz);
    void update_kf_narrow_integration_time();
    void update_kf_cn0(double current_cn0_dbhz);
    void run_Kf();

    void msg_handler_telemetry_to_trk(const pmt::pmt_t &msg);
    void msg_handler_pvt_to_trk(const pmt::pmt_t &msg);
    void do_correlation_step(const gr_complex *input_samples);

    void check_carrier_phase_coherent_initialization();
    void update_tracking_vars();
    void clear_tracking_vars();
    void save_correlation_results();
    void log_data();
    bool cn0_and_tracking_lock_status(double coh_integration_time_s);
    bool acquire_secondary();
    int32_t save_matfile() const;

    Cpu_Multicorrelator_Real_Codes d_multicorrelator_cpu;
    Cpu_Multicorrelator_Real_Codes d_correlator_data_cpu;  // for data channel

    Kf_Conf d_trk_parameters;

    Exponential_Smoother d_cn0_smoother;
    Exponential_Smoother d_carrier_lock_test_smoother;

    Gnss_Synchro *d_acquisition_gnss_synchro;

    volk_gnsssdr::vector<float> d_tracking_code;
    volk_gnsssdr::vector<float> d_data_code;
    volk_gnsssdr::vector<float> d_local_code_shift_chips;
    volk_gnsssdr::vector<gr_complex> d_correlator_outs;
    volk_gnsssdr::vector<gr_complex> d_Prompt_Data;
    volk_gnsssdr::vector<gr_complex> d_Prompt_buffer;

    boost::circular_buffer<gr_complex> d_Prompt_circular_buffer;
    boost::circular_buffer<std::pair<double, double>> d_code_ph_history;
    boost::circular_buffer<std::pair<double, double>> d_carr_ph_history;

    const size_t d_int_type_hash_code = typeid(int).hash_code();

    // Kalman Filter class variables
    arma::mat d_F;
    arma::mat d_H;
    arma::mat d_R;
    arma::mat d_Q;
    arma::mat d_P_old_old;
    arma::mat d_P_new_old;
    arma::mat d_P_new_new;
    arma::vec d_x_old_old;
    arma::vec d_x_new_old;
    arma::vec d_x_new_new;

    std::string d_secondary_code_string;
    std::string d_data_secondary_code_string;
    std::string d_systemName;
    std::string d_signal_type;
    std::string d_signal_pretty_name;
    std::string d_dump_filename;

    std::ofstream d_dump_file;

    gr_complex *d_Very_Early;
    gr_complex *d_Early;
    gr_complex *d_Prompt;
    gr_complex *d_Late;
    gr_complex *d_Very_Late;

    gr_complex d_VE_accu;
    gr_complex d_E_accu;
    gr_complex d_P_accu;
    gr_complex d_P_accu_old;
    gr_complex d_L_accu;
    gr_complex d_VL_accu;
    gr_complex d_P_data_accu;

    // nominal signal parameters
    double d_signal_carrier_freq;
    double d_code_period;
    double d_code_chip_rate;

    // acquisition
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;
    double d_current_correlation_time_s;

    // carrier and code discriminators output
    double d_carr_phase_error_disc_hz;
    double d_code_error_disc_chips;

    // estimated parameters
    // code
    double d_code_error_kf_chips;
    double d_code_freq_kf_chips_s;
    // carrier
    double d_carrier_phase_kf_rad;
    double d_carrier_doppler_kf_hz;
    double d_carrier_doppler_rate_kf_hz_s;

    double d_acc_carrier_phase_rad;

    double d_T_chip_seconds;
    double d_T_prn_seconds;
    double d_T_prn_samples;
    double d_K_blk_samples;
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_carrier_lock_threshold;

    // carrier NCO
    double d_carrier_phase_step_rad;
    double d_carrier_phase_rate_step_rad;

    // code NCO
    double d_code_phase_step_chips;
    double d_code_phase_rate_step_chips;
    double d_rem_code_phase_chips;
    double d_rem_code_phase_samples;

    double d_beta;

    uint64_t d_sample_counter;
    uint64_t d_acq_sample_stamp;

    float *d_prompt_data_shift;
    float d_rem_carr_phase_rad;

    uint32_t d_channel;
    uint32_t d_secondary_code_length;
    uint32_t d_data_secondary_code_length;

    int32_t d_symbols_per_bit;
    int32_t d_state;
    int32_t d_correlation_length_ms;
    int32_t d_n_correlator_taps;
    int32_t d_current_prn_length_samples;
    int32_t d_extend_correlation_symbols_count;
    int32_t d_current_symbol;
    int32_t d_current_data_symbol;
    int32_t d_cn0_estimation_counter;
    int32_t d_carrier_lock_fail_counter;
    int32_t d_code_lock_fail_counter;
    int32_t d_code_samples_per_chip;  // All signals have 1 sample per chip code except Gal. E1 which has 2 (CBOC disabled) or 12 (CBOC enabled)
    int32_t d_code_length_chips;

    bool d_pull_in_transitory;
    bool d_corrected_doppler;
    bool d_interchange_iq;
    bool d_veml;
    bool d_cloop;
    bool d_secondary;
    bool d_dump;
    bool d_dump_mat;
    bool d_acc_carrier_phase_initialized;
    bool d_enable_extended_integration;
};

#endif  // GNSS_SDR_KF_TRACKING_H
