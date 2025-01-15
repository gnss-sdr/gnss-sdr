/*!
 * \file dll_pll_veml_tracking_fpga.h
 * \brief Implementation of a code DLL + carrier PLL tracking block using an FPGA.
 * \author Marc Majoral, 2019. marc.majoral(at)cttc.es
 * \author Javier Arribas, 2019. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_DLL_PLL_VEML_TRACKING_FPGA_H
#define GNSS_SDR_DLL_PLL_VEML_TRACKING_FPGA_H

#include "dll_pll_conf_fpga.h"
#include "exponential_smoother.h"
#include "gnss_block_interface.h"
#include "tracking_FLL_PLL_filter.h"  // for PLL/FLL filter
#include "tracking_loop_filter.h"     // for DLL filter
#include <boost/circular_buffer.hpp>
#include <gnuradio/block.h>                   // for block
#include <gnuradio/gr_complex.h>              // for gr_complex
#include <gnuradio/types.h>                   // for gr_vector_int, gr_vector...
#include <pmt/pmt.h>                          // for pmt_t
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <cstddef>                            // for size_t
#include <cstdint>                            // for int32_t
#include <fstream>                            // for string, ofstream
#include <memory>                             // for std::shared_ptr
#include <string>                             // for string
#include <typeinfo>                           // for typeid
#include <utility>                            // for pair

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_gnuradio_blocks
 * \{ */


class Fpga_Multicorrelator_8sc;
class Gnss_Synchro;
class dll_pll_veml_tracking_fpga;

using dll_pll_veml_tracking_fpga_sptr = gnss_shared_ptr<dll_pll_veml_tracking_fpga>;

dll_pll_veml_tracking_fpga_sptr dll_pll_veml_make_tracking_fpga(const Dll_Pll_Conf_Fpga &conf_);


/*!
 * \brief This class implements a code DLL + carrier PLL tracking block.
 */
class dll_pll_veml_tracking_fpga : public gr::block
{
public:
    /*!
     * \brief Destructor
     */
    ~dll_pll_veml_tracking_fpga();

    /*!
     * \brief Set the channel number and configure some multicorrelator parameters
     */
    void set_channel(uint32_t channel, const std::string &device_io_name);

    /*!
     * \brief This function is used with two purposes:
     * 1 -> To set the gnss_synchro
     * 2 -> A set_gnss_synchro command with a valid PRN is received when the system is going to run
     * acquisition with that PRN. We can use this command to pre-initialize tracking parameters and
     * variables before the actual acquisition process takes place. In this way we minimize the
     * latency between acquisition and tracking once the acquisition has been made.
     */
    void set_gnss_synchro(Gnss_Synchro *p_gnss_synchro);

    /*!
     * \brief This function starts the tracking process
     */
    void start_tracking();

    /*!
     * \brief This function sets a flag that makes general_work to stop in order to finish the tracking process.
     */
    void stop_tracking();

    /*!
     * \brief General Work
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    /*!
     * \brief This function disables the HW multicorrelator in the FPGA in order to stop the tracking process
     */
    void reset();

private:
    friend dll_pll_veml_tracking_fpga_sptr dll_pll_veml_make_tracking_fpga(const Dll_Pll_Conf_Fpga &conf_);
    explicit dll_pll_veml_tracking_fpga(const Dll_Pll_Conf_Fpga &conf_);

    void msg_handler_telemetry_to_trk(const pmt::pmt_t &msg);
    bool cn0_and_tracking_lock_status(double coh_integration_time_s);
    bool acquire_secondary();
    void do_correlation_step();
    void run_dll_pll();
    void check_carrier_phase_coherent_initialization();
    void update_tracking_vars();
    void clear_tracking_vars();
    void save_correlation_results();
    void log_data();
    int32_t save_matfile() const;

    Dll_Pll_Conf_Fpga d_trk_parameters;

    Exponential_Smoother d_cn0_smoother;
    Exponential_Smoother d_carrier_lock_test_smoother;

    Gnss_Synchro *d_acquisition_gnss_synchro;

    Tracking_loop_filter d_code_loop_filter;

    Tracking_FLL_PLL_filter d_carrier_loop_filter;

    volk_gnsssdr::vector<float> d_local_code_shift_chips;
    volk_gnsssdr::vector<gr_complex> d_correlator_outs;
    volk_gnsssdr::vector<gr_complex> d_Prompt_Data;
    volk_gnsssdr::vector<gr_complex> d_Prompt_buffer;

    boost::circular_buffer<float> d_dll_filt_history;
    boost::circular_buffer<std::pair<double, double>> d_code_ph_history;
    boost::circular_buffer<std::pair<double, double>> d_carr_ph_history;
    boost::circular_buffer<gr_complex> d_Prompt_circular_buffer;

    std::string d_systemName;
    std::string d_signal_type;
    std::string d_secondary_code_string;
    std::string d_data_secondary_code_string;
    std::string d_signal_pretty_name;
    std::string d_dump_filename;

    std::ofstream d_dump_file;

    std::shared_ptr<Fpga_Multicorrelator_8sc> d_multicorrelator_fpga;

    boost::condition_variable d_m_condition;

    boost::mutex d_mutex;

    const size_t int_type_hash_code = typeid(int).hash_code();

    double d_signal_carrier_freq;
    double d_code_period;
    double d_code_chip_rate;
    double d_code_phase_step_chips;
    double d_code_phase_rate_step_chips;
    double d_carrier_phase_step_rad;
    double d_carrier_phase_rate_step_rad;
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;
    double d_rem_code_phase_samples;
    double d_rem_code_phase_samples_prev;
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

    uint64_t d_sample_counter;
    uint64_t d_acq_sample_stamp;
    uint64_t d_sample_counter_next;

    float *d_prompt_data_shift;
    float d_rem_carr_phase_rad;

    int32_t d_symbols_per_bit;
    int32_t d_state;
    int32_t d_extend_correlation_symbols_count;
    int32_t d_current_symbol;
    int32_t d_current_data_symbol;
    int32_t d_current_integration_length_samples;
    int32_t d_cn0_estimation_counter;
    int32_t d_carrier_lock_fail_counter;
    int32_t d_code_lock_fail_counter;
    int32_t d_correlation_length_ms;
    int32_t d_n_correlator_taps;
    int32_t d_next_integration_length_samples;
    int32_t d_extend_fpga_integration_periods;

    uint32_t d_channel;
    uint32_t d_secondary_code_length;
    uint32_t d_data_secondary_code_length;
    uint32_t d_code_length_chips;
    uint32_t d_code_samples_per_chip;  // All signals have 1 sample per chip code except Gal. E1 which has 2 (CBOC disabled) or 12 (CBOC enabled)
    uint32_t d_fpga_integration_period;
    uint32_t d_current_fpga_integration_period;

    bool d_veml;
    bool d_cloop;
    bool d_secondary;
    bool d_enable_extended_integration;
    bool d_dump;
    bool d_dump_mat;
    bool d_pull_in_transitory;
    bool d_corrected_doppler;
    bool d_interchange_iq;
    bool d_acc_carrier_phase_initialized;
    bool d_worker_is_done;
    bool d_extended_correlation_in_fpga;
    bool d_current_extended_correlation_in_fpga;
    bool d_stop_tracking;
    bool d_sc_demodulate_enabled;
    bool d_Flag_PLL_180_deg_phase_locked;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_DLL_PLL_VEML_TRACKING_FPGA_H
