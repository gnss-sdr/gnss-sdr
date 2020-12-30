/*!
 * \file glonass_l2_ca_dll_pll_c_aid_tracking_sc.h
 * \brief  Implementation of a code DLL + carrier PLL tracking block
 * \author Damian Miralles, 2018. dmiralles2009(at)gmail.com
 *
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkha user, 2007
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

#ifndef GNSS_SDR_GLONASS_L2_CA_DLL_PLL_C_AID_TRACKING_SC_H
#define GNSS_SDR_GLONASS_L2_CA_DLL_PLL_C_AID_TRACKING_SC_H

#include "cpu_multicorrelator_16sc.h"
#include "glonass_l2_signal_replica.h"
#include "gnss_block_interface.h"
#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_FLL_PLL_filter.h"
#include <gnuradio/block.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <deque>
#include <fstream>
#include <map>
#include <string>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_gnuradio_blocks
 * \{ */


class glonass_l2_ca_dll_pll_c_aid_tracking_sc;

using glonass_l2_ca_dll_pll_c_aid_tracking_sc_sptr = gnss_shared_ptr<glonass_l2_ca_dll_pll_c_aid_tracking_sc>;

glonass_l2_ca_dll_pll_c_aid_tracking_sc_sptr
glonass_l2_ca_dll_pll_c_aid_make_tracking_sc(
    int64_t fs_in, uint32_t vector_length,
    bool dump,
    const std::string& dump_filename,
    float pll_bw_hz,
    float dll_bw_hz,
    float pll_bw_narrow_hz,
    float dll_bw_narrow_hz,
    int32_t extend_correlation_ms,
    float early_late_space_chips);


/*!
 * \brief This class implements a DLL + PLL tracking loop block
 */
class glonass_l2_ca_dll_pll_c_aid_tracking_sc : public gr::block
{
public:
    ~glonass_l2_ca_dll_pll_c_aid_tracking_sc();

    void set_channel(uint32_t channel);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();

    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items);

    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

private:
    friend glonass_l2_ca_dll_pll_c_aid_tracking_sc_sptr
    glonass_l2_ca_dll_pll_c_aid_make_tracking_sc(
        int64_t fs_in, uint32_t vector_length,
        bool dump,
        const std::string& dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float pll_bw_narrow_hz,
        float dll_bw_narrow_hz,
        int32_t extend_correlation_ms,
        float early_late_space_chips);

    glonass_l2_ca_dll_pll_c_aid_tracking_sc(
        int64_t fs_in, uint32_t vector_length,
        bool dump,
        const std::string& dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float pll_bw_narrow_hz,
        float dll_bw_narrow_hz,
        int32_t extend_correlation_ms,
        float early_late_space_chips);

    void msg_handler_preamble_index(const pmt::pmt_t& msg);

    void check_carrier_phase_coherent_initialization();

    int32_t save_matfile() const;

    volk_gnsssdr::vector<gr_complex> d_ca_code;
    volk_gnsssdr::vector<gr_complex> d_Prompt_buffer;
    volk_gnsssdr::vector<float> d_local_code_shift_chips;
    volk_gnsssdr::vector<lv_16sc_t> d_ca_code_16sc;
    volk_gnsssdr::vector<lv_16sc_t> d_correlator_outs_16sc;

    Cpu_Multicorrelator_16sc multicorrelator_cpu_16sc;

    // PLL and DLL filter library
    Tracking_2nd_DLL_filter d_code_loop_filter;
    Tracking_FLL_PLL_filter d_carrier_loop_filter;

    // symbol history to detect bit transition
    std::deque<lv_16sc_t> d_E_history;
    std::deque<lv_16sc_t> d_P_history;
    std::deque<lv_16sc_t> d_L_history;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;

    // tracking configuration vars
    Gnss_Synchro* d_acquisition_gnss_synchro;
    int64_t d_fs_in;
    int64_t d_glonass_freq_ch;
    double d_early_late_spc_chips;
    uint32_t d_vector_length;
    uint32_t d_channel;
    int32_t d_n_correlator_taps;

    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_rem_code_phase_chips;
    double d_rem_carrier_phase_rad;
    int32_t d_rem_code_phase_integer_samples;

    // acquisition
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;

    // tracking vars
    float d_dll_bw_hz;
    float d_pll_bw_hz;
    float d_dll_bw_narrow_hz;
    float d_pll_bw_narrow_hz;
    double d_code_freq_chips;
    double d_code_phase_step_chips;
    double d_carrier_doppler_hz;
    double d_carrier_frequency_hz;
    double d_carrier_doppler_old_hz;
    double d_carrier_phase_step_rad;
    double d_acc_carrier_phase_cycles;
    double d_code_phase_samples;
    double d_pll_to_dll_assist_secs_Ti;
    double d_carr_phase_error_secs_Ti;
    double d_code_error_chips_Ti;
    double d_preamble_timestamp_s;
    int32_t d_extend_correlation_ms;
    double d_code_error_filt_chips_s;
    double d_code_error_filt_chips_Ti;

    // Integration period in samples
    int32_t d_correlation_length_samples;

    // processing samples counters
    uint64_t d_sample_counter;
    uint64_t d_acq_sample_stamp;

    // CN0 estimation and lock detector
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_carrier_lock_threshold;
    int32_t d_carrier_lock_fail_counter;
    int32_t d_cn0_estimation_counter;

    bool d_enable_extended_integration;
    bool d_preamble_synchronized;

    // control vars
    bool d_enable_tracking;
    bool d_pull_in;
    bool d_acc_carrier_phase_initialized;

    bool d_dump;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_L2_CA_DLL_PLL_C_AID_TRACKING_SC_H
