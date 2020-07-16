/*!
 * \file dll_pll_veml_tracking.h
 * \brief Implementation of a code DLL + carrier PLL tracking block.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 * \author Antonio Ramos, 2018 antonio.ramosdet(at)gmail.com
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

#ifndef GNSS_SDR_DLL_PLL_VEML_TRACKING_H
#define GNSS_SDR_DLL_PLL_VEML_TRACKING_H

#include "cpu_multicorrelator_real_codes.h"
#include "dll_pll_conf.h"
#include "exponential_smoother.h"
#include "tracking_FLL_PLL_filter.h"  // for PLL/FLL filter
#include "tracking_loop_filter.h"     // for DLL filter
#include <boost/circular_buffer.hpp>
#include <gnuradio/block.h>                   // for block
#include <gnuradio/gr_complex.h>              // for gr_complex
#include <gnuradio/types.h>                   // for gr_vector_int, gr_vector...
#include <pmt/pmt.h>                          // for pmt_t
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <cstdint>                            // for int32_t
#include <fstream>                            // for string, ofstream
#include <string>
#include <utility>  // for pair
#if GNURADIO_USES_STD_POINTERS
#include <memory>
#else
#include <boost/shared_ptr.hpp>
#endif

class Gnss_Synchro;
class dll_pll_veml_tracking;

#if GNURADIO_USES_STD_POINTERS
using dll_pll_veml_tracking_sptr = std::shared_ptr<dll_pll_veml_tracking>;
#else
using dll_pll_veml_tracking_sptr = boost::shared_ptr<dll_pll_veml_tracking>;
#endif


dll_pll_veml_tracking_sptr dll_pll_veml_make_tracking(const Dll_Pll_Conf &conf_);

/*!
 * \brief This class implements a code DLL + carrier PLL tracking block.
 */
class dll_pll_veml_tracking : public gr::block
{
public:
    ~dll_pll_veml_tracking();

    void set_channel(uint32_t channel);
    void set_gnss_synchro(Gnss_Synchro *p_gnss_synchro);
    void start_tracking();
    void stop_tracking();

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

private:
    friend dll_pll_veml_tracking_sptr dll_pll_veml_make_tracking(const Dll_Pll_Conf &conf_);
    explicit dll_pll_veml_tracking(const Dll_Pll_Conf &conf_);

    void msg_handler_telemetry_to_trk(const pmt::pmt_t &msg);
    void do_correlation_step(const gr_complex *input_samples);
    void run_dll_pll();
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

    Dll_Pll_Conf d_trk_parameters;

    Exponential_Smoother d_cn0_smoother;
    Exponential_Smoother d_carrier_lock_test_smoother;

    Tracking_loop_filter d_code_loop_filter;
    Tracking_FLL_PLL_filter d_carrier_loop_filter;

    Gnss_Synchro *d_acquisition_gnss_synchro;

    volk_gnsssdr::vector<float> d_tracking_code;
    volk_gnsssdr::vector<float> d_data_code;
    volk_gnsssdr::vector<float> d_local_code_shift_chips;
    volk_gnsssdr::vector<gr_complex> d_correlator_outs;
    volk_gnsssdr::vector<gr_complex> d_Prompt_Data;
    volk_gnsssdr::vector<gr_complex> d_Prompt_buffer;

    boost::circular_buffer<float> d_dll_filt_history;
    boost::circular_buffer<std::pair<double, double>> d_code_ph_history;
    boost::circular_buffer<std::pair<double, double>> d_carr_ph_history;
    boost::circular_buffer<gr_complex> d_Prompt_circular_buffer;

    double d_signal_carrier_freq;
    double d_code_period;
    double d_code_chip_rate;
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;
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
    double d_T_chip_seconds;
    double d_T_prn_seconds;
    double d_T_prn_samples;
    double d_K_blk_samples;
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_carrier_lock_threshold;
    double d_carrier_phase_step_rad;
    double d_carrier_phase_rate_step_rad;
    double d_code_phase_step_chips;
    double d_code_phase_rate_step_chips;
    double d_rem_code_phase_samples;

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

    std::string d_secondary_code_string;
    std::string d_data_secondary_code_string;
    std::string d_systemName;
    std::string d_signal_type;
    std::string d_signal_pretty_name;
    std::string d_dump_filename;

    std::ofstream d_dump_file;

    uint64_t d_sample_counter;
    uint64_t d_acq_sample_stamp;

    float *d_prompt_data_shift;
    float d_rem_carr_phase_rad;

    int32_t d_symbols_per_bit;
    int32_t d_preamble_length_symbols;
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

    uint32_t d_channel;
    uint32_t d_secondary_code_length;
    uint32_t d_data_secondary_code_length;

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

#endif  // GNSS_SDR_DLL_PLL_VEML_TRACKING_H
