/*!
 * \file galileo_e1_prs_veml_tracking_cc.h
 * \brief Implementation of VEML tracking for Galileo E1 PRS
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GALILEO_E1_PRS_VEML_TRACKING_CC_H
#define GNSS_SDR_GALILEO_E1_PRS_VEML_TRACKING_CC_H

#include <fstream>
#include <queue>
#include <string>
#include <map>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <gnuradio/block.h>
#include <gnuradio/msg_queue.h>
#include "concurrent_queue.h"
#include "gnss_synchro.h"
#include "tracking_loop_filter.h"
#include "correlator.h"
#include "gnss_message.h"
#include "long_code_interface.h" // for prs code gen

class galileo_e1_prs_veml_tracking_cc;

typedef boost::shared_ptr<galileo_e1_prs_veml_tracking_cc> galileo_e1_prs_veml_tracking_cc_sptr;

galileo_e1_prs_veml_tracking_cc_sptr
galileo_e1_prs_veml_make_tracking_cc(long if_freq,
                                   long fs_in, unsigned
                                   int vector_length,
                                   boost::shared_ptr<gr::msg_queue> queue,
                                   bool dump,
                                   std::string dump_filename,
                                   int   pll_loop_order,
                                   float pll_initial_bw_hz,
                                   float pll_final_bw_hz,
                                   int   dll_loop_order,
                                   float dll_initial_bw_hz,
                                   float dll_final_bw_hz,
                                   float initial_early_late_code_space_cycles,
                                   float final_early_late_code_space_cycles,
                                   float initial_very_early_late_code_space_chips,
                                   float final_very_early_late_code_space_chips,
                                   bool aid_code_with_carrier,
                                   bool use_bump_jumping,
                                   unsigned int bump_jumping_threshold,
                                   LongCodeInterface_sptr prs_code_gen);

/*!
 * \brief This class implements a double estimator tracking block for Galileo E1 signals
 */
class galileo_e1_prs_veml_tracking_cc: public gr::block
{
public:
    ~galileo_e1_prs_veml_tracking_cc();

    void set_channel(unsigned int channel);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();
    void set_channel_queue(concurrent_queue<int> *channel_internal_queue);

    /*!
     * \brief Code DLL + carrier PLL according to the algorithms described in:
     * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
     * A Software-Defined GPS and Galileo Receiver. A Single-Frequency Approach,
     * Birkhauser, 2007
     */
    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast (int noutput_items, gr_vector_int &ninput_items_required);
private:
    void start_tracking_prs();

    friend galileo_e1_prs_veml_tracking_cc_sptr
    galileo_e1_prs_veml_make_tracking_cc(long if_freq,
            long fs_in, unsigned
            int vector_length,
            boost::shared_ptr<gr::msg_queue> queue,
            bool dump,
            std::string dump_filename,
            int   pll_loop_order,
            float pll_initial_bw_hz,
            float pll_final_bw_hz,
            int   dll_loop_order,
            float dll_initial_bw_hz,
            float dll_final_bw_hz,
            float initial_early_late_code_space_cycles,
            float final_early_late_code_space_cycles,
            float initial_very_early_late_code_space_chips,
            float final_very_early_late_code_space_chips,
            bool aid_code_with_carrier,
            bool use_bump_jumping,
            unsigned int bump_jumping_threshold,
            LongCodeInterface_sptr prs_code_gen);

    galileo_e1_prs_veml_tracking_cc(long if_freq,
            long fs_in, unsigned
            int vector_length,
            boost::shared_ptr<gr::msg_queue> queue,
            bool dump,
            std::string dump_filename,
            int   pll_loop_order,
            float pll_initial_bw_hz,
            float pll_final_bw_hz,
            int   dll_loop_order,
            float dll_initial_bw_hz,
            float dll_final_bw_hz,
            float initial_early_late_code_space_cycles,
            float final_early_late_code_space_cycles,
            float initial_very_early_late_code_space_chips,
            float final_very_early_late_code_space_chips,
            bool aid_code_with_carrier,
            bool use_bump_jumping,
            unsigned int bump_jumping_threshold,
            LongCodeInterface_sptr prs_code_gen);

    void update_local_code();
    void update_local_code_prs();

    void update_local_carrier();

    // tracking configuration vars
    boost::shared_ptr<gr::msg_queue> d_queue;
    concurrent_queue<int> *d_channel_internal_queue;
    unsigned int d_vector_length;
    bool d_dump;

    Gnss_Synchro* d_acquisition_gnss_synchro;
    unsigned int d_channel;
    int d_last_seg;
    long d_if_freq;
    long d_fs_in;

    bool d_aid_code_with_carrier;

    float d_early_late_code_spc_cycles;
    float d_very_early_late_code_spc_chips;
    float d_very_early_late_code_spc_chips_prs;

    LongCodeInterface_sptr d_prs_code_gen;
    gr_complex* d_e1b_code;
    gr_complex* d_prs_code;
    std::vector< short > d_prs_code_shorts;
    uint64_t d_start_index_prs_code;
    unsigned int d_size_prs_code;
    bool d_prs_code_initialized;


    gr_complex* d_early_code;
    gr_complex* d_prompt_code;
    gr_complex* d_late_code;
    gr_complex* d_very_early_code;
    gr_complex* d_very_late_code;
    gr_complex* d_carr_sign;


    gr_complex* d_early_code_prs;
    gr_complex* d_prompt_code_prs;
    gr_complex* d_late_code_prs;
    gr_complex* d_very_early_code_prs;
    gr_complex* d_very_late_code_prs;

    gr_complex *d_Very_Early;
    gr_complex *d_Early;
    gr_complex *d_Prompt;
    gr_complex *d_Late;
    gr_complex *d_Very_Late;

    gr_complex *d_Very_Early_prs;
    gr_complex *d_Early_prs;
    gr_complex *d_Prompt_prs;
    gr_complex *d_Late_prs;
    gr_complex *d_Very_Late_prs;

    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    float d_rem_carr_phase_rad;

    double d_rem_code_phase_samples_prs;
    float d_rem_carr_phase_rad_prs;

    // PLL and DLL filter library
    Tracking_loop_filter d_code_loop_filter;
    Tracking_loop_filter d_carrier_loop_filter;


    int d_pll_loop_order;
    float d_initial_pll_bw_hz;
    float d_final_pll_bw_hz;

    int d_dll_loop_order;
    float d_initial_dll_bw_hz;
    float d_final_dll_bw_hz;

    float d_initial_early_late_code_space_cycles;
    float d_final_early_late_code_space_cycles;

    float d_initial_very_early_late_code_space_chips;
    float d_final_very_early_late_code_space_chips;

    Tracking_loop_filter d_code_loop_filter_prs;
    Tracking_loop_filter d_carrier_loop_filter_prs;

    // acquisition
    float d_acq_code_phase_samples;
    float d_acq_carrier_doppler_hz;

    // correlator
    Correlator d_correlator;

    // tracking vars
    double d_code_freq_chips;
    double d_code_phase_chips;
    float d_carrier_doppler_hz;
    double d_carrier_phase_rad;
    double d_acc_carrier_phase_rad;
    double d_acc_code_phase_secs;

    double d_code_freq_chips_prs;
    double d_code_phase_chips_prs;
    float d_carrier_doppler_hz_prs;
    double d_carrier_phase_rad_prs;
    double d_acc_carrier_phase_rad_prs;
    double d_acc_code_phase_secs_prs;

    //PRN period in samples
    int d_current_prn_length_samples;

    //processing samples counters
    unsigned long int d_sample_counter;
    unsigned long int d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int d_cn0_estimation_counter;
    gr_complex* d_Prompt_buffer;
    float d_carrier_lock_test;
    float d_CN0_SNV_dB_Hz;
    float d_carrier_lock_threshold;
    int d_carrier_lock_fail_counter;
    int d_carrier_lock_success_counter;

    // control vars
    bool d_enable_tracking;
    bool d_pull_in;

    bool d_carrier_locked;
    bool d_code_locked;
    bool d_code_locked_prs;

    bool d_tow_received;
    double d_last_tow;
    double d_timestamp_last_tow;

    bool d_rx_time_set;
    double d_tow_rx_time;
    double d_timestamp_rx_time;

    bool d_preamble_start_detected;
    double d_preamble_timestamp;

    bool d_prs_tracking_enabled;

    // Bump jumping variables:
    bool d_use_bj;
    unsigned int d_bj_ve_counter;
    unsigned int d_bj_vl_counter;

    unsigned int d_bj_ve_counter_prs;
    unsigned int d_bj_vl_counter_prs;

    unsigned int d_bj_threshold;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;

    // Handler for gnss_messages:
    void handle_gnss_message( pmt::pmt_t msg );
};

#endif //GNSS_SDR_GALILEO_E1_PRS_VEML_TRACKING_CC_H

