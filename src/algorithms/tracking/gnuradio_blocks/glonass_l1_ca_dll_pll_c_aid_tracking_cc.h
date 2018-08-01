/*!
 * \file glonass_l1_ca_dll_pll_c_aid_tracking_cc.h
 * \brief  Implementation of a code DLL + carrier PLL tracking block
 * \author Gabriel Araujo, 2017. gabriel.araujo.5000(at)gmail.com
 * \author Luis Esteve, 2017. luis(at)epsilon-formacion.com
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
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

#ifndef GNSS_SDR_GLONASS_L1_CA_DLL_PLL_C_AID_TRACKING_CC_H
#define GNSS_SDR_GLONASS_L1_CA_DLL_PLL_C_AID_TRACKING_CC_H

#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_FLL_PLL_filter.h"
//#include "tracking_loop_filter.h"
#include "cpu_multicorrelator.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <fstream>
#include <map>
#include <deque>
#include <string>

class glonass_l1_ca_dll_pll_c_aid_tracking_cc;

typedef boost::shared_ptr<glonass_l1_ca_dll_pll_c_aid_tracking_cc>
    glonass_l1_ca_dll_pll_c_aid_tracking_cc_sptr;

glonass_l1_ca_dll_pll_c_aid_tracking_cc_sptr
glonass_l1_ca_dll_pll_c_aid_make_tracking_cc(
    long fs_in, unsigned int vector_length,
    bool dump,
    std::string dump_filename,
    float pll_bw_hz,
    float dll_bw_hz,
    float pll_bw_narrow_hz,
    float dll_bw_narrow_hz,
    int extend_correlation_ms,
    float early_late_space_chips);


/*!
 * \brief This class implements a DLL + PLL tracking loop block
 */
class glonass_l1_ca_dll_pll_c_aid_tracking_cc : public gr::block
{
public:
    ~glonass_l1_ca_dll_pll_c_aid_tracking_cc();

    void set_channel(unsigned int channel);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();

    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items);

    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

private:
    friend glonass_l1_ca_dll_pll_c_aid_tracking_cc_sptr
    glonass_l1_ca_dll_pll_c_aid_make_tracking_cc(
        long fs_in, unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float pll_bw_narrow_hz,
        float dll_bw_narrow_hz,
        int extend_correlation_ms,
        float early_late_space_chips);

    glonass_l1_ca_dll_pll_c_aid_tracking_cc(
        long fs_in, unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float pll_bw_narrow_hz,
        float dll_bw_narrow_hz,
        int extend_correlation_ms,
        float early_late_space_chips);

    // tracking configuration vars
    unsigned int d_vector_length;
    bool d_dump;

    Gnss_Synchro* d_acquisition_gnss_synchro;
    unsigned int d_channel;

    long d_fs_in;
    double d_glonass_freq_ch;

    double d_early_late_spc_chips;
    int d_n_correlator_taps;

    gr_complex* d_ca_code;
    float* d_local_code_shift_chips;
    gr_complex* d_correlator_outs;
    cpu_multicorrelator multicorrelator_cpu;

    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_rem_code_phase_chips;
    double d_rem_carrier_phase_rad;
    int d_rem_code_phase_integer_samples;

    // PLL and DLL filter library
    //Tracking_2nd_DLL_filter d_code_loop_filter;
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
    double d_code_error_chips_Ti;
    double d_code_error_filt_chips_s;
    double d_code_error_filt_chips_Ti;
    double d_carr_phase_error_secs_Ti;

    // symbol history to detect bit transition
    std::deque<gr_complex> d_E_history;
    std::deque<gr_complex> d_P_history;
    std::deque<gr_complex> d_L_history;
    double d_preamble_timestamp_s;
    int d_extend_correlation_ms;
    bool d_enable_extended_integration;
    bool d_preamble_synchronized;
    void msg_handler_preamble_index(pmt::pmt_t msg);

    //Integration period in samples
    int d_correlation_length_samples;

    //processing samples counters
    unsigned long int d_sample_counter;
    unsigned long int d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int d_cn0_estimation_counter;
    gr_complex* d_Prompt_buffer;
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_carrier_lock_threshold;
    int d_carrier_lock_fail_counter;

    // control vars
    bool d_enable_tracking;
    bool d_pull_in;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;

    int save_matfile();
};

#endif  //GNSS_SDR_GLONASS_L1_CA_DLL_PLL_C_AID_TRACKING_CC_H
