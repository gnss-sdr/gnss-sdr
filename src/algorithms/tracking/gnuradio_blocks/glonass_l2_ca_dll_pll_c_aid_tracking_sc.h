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

#ifndef GNSS_SDR_GLONASS_L2_CA_DLL_PLL_C_AID_TRACKING_SC_H
#define GNSS_SDR_GLONASS_L2_CA_DLL_PLL_C_AID_TRACKING_SC_H

#include "glonass_l2_signal_processing.h"
#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_FLL_PLL_filter.h"
#include "cpu_multicorrelator_16sc.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <gnuradio/block.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <fstream>
#include <map>
#include <string>

class glonass_l2_ca_dll_pll_c_aid_tracking_sc;

typedef boost::shared_ptr<glonass_l2_ca_dll_pll_c_aid_tracking_sc>
    glonass_l2_ca_dll_pll_c_aid_tracking_sc_sptr;

glonass_l2_ca_dll_pll_c_aid_tracking_sc_sptr
glonass_l2_ca_dll_pll_c_aid_make_tracking_sc(
    int64_t fs_in, uint32_t vector_length,
    bool dump,
    std::string dump_filename,
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
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float pll_bw_narrow_hz,
        float dll_bw_narrow_hz,
        int32_t extend_correlation_ms,
        float early_late_space_chips);

    glonass_l2_ca_dll_pll_c_aid_tracking_sc(
        int64_t fs_in, uint32_t vector_length,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float pll_bw_narrow_hz,
        float dll_bw_narrow_hz,
        int32_t extend_correlation_ms,
        float early_late_space_chips);

    // tracking configuration vars
    uint32_t d_vector_length;
    bool d_dump;

    Gnss_Synchro* d_acquisition_gnss_synchro;
    uint32_t d_channel;

    int64_t d_fs_in;
    int64_t d_glonass_freq_ch;

    double d_early_late_spc_chips;
    int32_t d_n_correlator_taps;

    gr_complex* d_ca_code;
    lv_16sc_t* d_ca_code_16sc;
    float* d_local_code_shift_chips;
    //gr_complex* d_correlator_outs;
    lv_16sc_t* d_correlator_outs_16sc;
    //cpu_multicorrelator multicorrelator_cpu;
    cpu_multicorrelator_16sc multicorrelator_cpu_16sc;

    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_rem_code_phase_chips;
    double d_rem_carrier_phase_rad;
    int32_t d_rem_code_phase_integer_samples;

    // PLL and DLL filter library
    Tracking_2nd_DLL_filter d_code_loop_filter;
    Tracking_FLL_PLL_filter d_carrier_loop_filter;

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
    bool d_enable_extended_integration;
    bool d_preamble_synchronized;
    double d_code_error_filt_chips_s;
    double d_code_error_filt_chips_Ti;
    void msg_handler_preamble_index(pmt::pmt_t msg);

    // symbol history to detect bit transition
    std::deque<lv_16sc_t> d_E_history;
    std::deque<lv_16sc_t> d_P_history;
    std::deque<lv_16sc_t> d_L_history;

    //Integration period in samples
    int32_t d_correlation_length_samples;

    //processing samples counters
    uint64_t d_sample_counter;
    uint64_t d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int32_t d_cn0_estimation_counter;
    gr_complex* d_Prompt_buffer;
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_carrier_lock_threshold;
    int32_t d_carrier_lock_fail_counter;

    // control vars
    bool d_enable_tracking;
    bool d_pull_in;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;

    int32_t save_matfile();
};

#endif  //GNSS_SDR_GLONASS_L2_CA_DLL_PLL_C_AID_TRACKING_SC_H
