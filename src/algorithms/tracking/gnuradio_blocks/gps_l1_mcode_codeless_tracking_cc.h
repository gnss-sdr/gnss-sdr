/*!
 * \file gps_l1_mcode_codeless_tracking_cc.h
 * \brief Implementation of codeless tracking for GPS L1 MCODE
 * \author Cillian O'Driscoll, 2017. cillian.odriscoll(at)gmail.com
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * [1] K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * Codeless approach described in:
 * [2] D. Borio, M. Rao and C. O'Driscoll,
 * Codeless Processing of BOC Modulated Signals, IET Radar, Sonar and Navigation,
 * vol 7, no. 2, pp 143-152, 2013.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GPS_L1_MCODE_CODELESS_TRACKING_CC_H
#define GNSS_SDR_GPS_L1_MCODE_CODELESS_TRACKING_CC_H

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
#include "code_resampler.h"
#include "subcarrier_resampler.h"

class gps_l1_mcode_codeless_tracking_cc;

typedef boost::shared_ptr<gps_l1_mcode_codeless_tracking_cc> gps_l1_mcode_codeless_tracking_cc_sptr;

gps_l1_mcode_codeless_tracking_cc_sptr
gps_l1_mcode_codeless_make_tracking_cc(long if_freq,
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
                                   float initial_divergence_bw_hz,
                                   float final_divergence_bw_hz,
                                   int mcode_accumulation_length,
                                   bool close_mcode_loops,
                                   float pll_bw_hz_mcode,
                                   float dll_bw_hz_mcode);

/*!
 * \brief This class implements a codeless code tracking technique for
 * GPS L1 M Code
 */
class gps_l1_mcode_codeless_tracking_cc: public gr::block
{
public:
    ~gps_l1_mcode_codeless_tracking_cc();

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
    void start_tracking_mcode();

    friend gps_l1_mcode_codeless_tracking_cc_sptr
    gps_l1_mcode_codeless_make_tracking_cc(long if_freq,
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
            float initial_divergence_bw_hz,
            float final_divergence_bw_hz,
            int mcode_accumulation_length,
            bool close_mcode_loops,
            float pll_bw_hz_mcode,
            float dll_bw_hz_mcode);

    gps_l1_mcode_codeless_tracking_cc(long if_freq,
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
            float initial_divergence_bw_hz,
            float final_divergence_bw_hz,
            int mcode_accumulation_length,
            bool close_mcode_loops,
            float pll_bw_hz_mcode,
            float dll_bw_hz_mcode);

    void update_local_code();
    void update_local_code_mcode();

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

    bool d_close_mcode_loops;

    float d_early_late_code_spc_chips;
    float d_early_late_code_spc_cycles_mcode;
    float d_very_early_late_code_spc_chips_mcode;

    gr_complex* d_ca_code;

    std::vector< int > d_mcode_code_phase_store;

    gr_complex* d_early_code;
    gr_complex* d_prompt_code;
    gr_complex* d_late_code;


    int* d_very_early_code_phases_mcode;
    int* d_early_code_phases_mcode;
    int* d_prompt_code_phases_mcode;
    int* d_late_code_phases_mcode;
    int* d_very_late_code_phases_mcode;

    gr_complex* d_very_early_subcarrier_mcode;
    gr_complex* d_early_subcarrier_mcode;
    gr_complex* d_prompt_subcarrier_mcode;
    gr_complex* d_late_subcarrier_mcode;
    gr_complex* d_very_late_subcarrier_mcode;

    gr_complex *d_Early;
    gr_complex *d_Prompt;
    gr_complex *d_Late;

    gr_complex *d_Very_Early_mcode;
    gr_complex *d_Early_mcode;
    gr_complex *d_Prompt_mcode;
    gr_complex *d_Late_mcode;
    gr_complex *d_Very_Late_mcode;

    gr_complex d_VE_acumm_mcode;
    gr_complex d_E_acumm_mcode;
    gr_complex d_P_acumm_mcode;
    gr_complex d_L_acumm_mcode;
    gr_complex d_VL_acumm_mcode;

    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    float d_rem_carr_phase_rad;

    double d_rem_code_phase_samples_mcode;
    float d_rem_carr_phase_rad_mcode;

    // PLL and DLL filter library
    Tracking_loop_filter d_code_loop_filter;
    Tracking_loop_filter d_carrier_loop_filter;
    Tracking_loop_filter d_frequency_loop_filter;


    int d_pll_loop_order;
    float d_initial_pll_bw_hz;
    float d_final_pll_bw_hz;

    int d_dll_loop_order;
    float d_initial_dll_bw_hz;
    float d_final_dll_bw_hz;

    int d_pll_loop_order_mcode;
    float d_pll_bw_hz_mcode;

    int d_dll_loop_order_mcode;
    float d_dll_bw_hz_mcode;

    float d_initial_early_late_code_space_chips;
    float d_final_early_late_code_space_chips;

    float d_initial_very_early_late_code_space_chips;
    float d_final_very_early_late_code_space_chips;

    Tracking_loop_filter d_code_loop_filter_mcode;
    Tracking_loop_filter d_carrier_loop_filter_mcode;

    // acquisition
    float d_acq_code_phase_samples;
    float d_acq_carrier_doppler_hz;

    // correlator
    Correlator d_correlator;

    // tracking vars
    double d_code_freq_chips;
    double d_code_phase_chips;
    double d_carrier_doppler_hz;
    double d_carrier_phase_rad;
    double d_acc_carrier_phase_rad;
    double d_acc_code_phase_secs;

    double d_code_freq_chips_mcode;
    int64_t d_integer_code_phase_chips_mcode;
    double d_fractional_code_phase_chips_mcode;

    double d_carrier_doppler_hz_mcode;
    double d_carrier_phase_rad_mcode;
    double d_acc_carrier_phase_rad_mcode;
    double d_acc_code_phase_secs_mcode;

    int64_t d_integer_subcarrier_phase_cycles_mcode;
    double d_fractional_subcarrier_phase_cycles_mcode;
    double d_subcarrier_freq_cycles_mcode;

    double d_chips_to_cycles_mcode;

    // mcode tracking variables: need to maintain state during accumulation
    double d_carr_error_hz_mcode;
    double d_carr_error_filt_hz_mcode;
    double d_subcarrier_error_cycles_mcode;
    double d_subcarrier_error_filt_cycles_mcode;
    double d_code_error_chips_veml_mcode;
    double d_code_error_filt_chips_veml_mcode;

    //PRN period in samples
    int d_current_prn_length_samples;

    //processing samples counters
    unsigned long int d_sample_counter;
    unsigned long int d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int d_cn0_estimation_counter;
    int d_cn0_estimation_counter_mcode;
    gr_complex* d_Prompt_buffer;
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_CN0_SNV_dB_Hz_mcode;
    double d_carrier_lock_threshold;
    int d_carrier_lock_fail_counter;
    int d_carrier_lock_success_counter;
    int d_fll_epochs;
    int d_max_fll_epochs;

    // control vars
    bool d_enable_tracking;
    bool d_pull_in;

    bool d_carrier_locked;
    bool d_frequency_locked;

    bool d_code_locked;
    bool d_code_locked_mcode;

    bool d_tow_received;
    double d_last_tow;
    double d_timestamp_last_tow;

    bool d_rx_time_set;
    double d_tow_rx_time;
    double d_timestamp_rx_time;

    bool d_preamble_start_detected;
    double d_preamble_timestamp;

    bool d_mcode_tracking_enabled;

    int d_mcode_accumulation_length;
    int d_mcode_accumulation_index;

    // Bump jumping variables:
    bool d_use_bj;

    unsigned int d_bj_ve_counter_mcode;
    unsigned int d_bj_vl_counter_mcode;

    unsigned int d_bj_threshold;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;

    // Subcarrier aiding approach:
    bool d_use_sa;

    Tracking_loop_filter d_divergence_loop_filter_mcode;

    double d_initial_divergence_loop_filter_bandwidth;
    double d_final_divergence_loop_filter_bandwidth;

    bool d_subcarrier_locked_mcode;
    double d_mean_subcarrier_error_mcode;

    double d_mean_code_error;
    double d_mean_code_error_mcode;

    // Handler for gnss_messages:
    void handle_gnss_message( pmt::pmt_t msg );

    // Resamplers for code and subcarrier:
    boost::shared_ptr< CodeResamplerInterface< gr_complex > > d_ca_code_resampler;

    boost::shared_ptr< CodeResamplerInterface< int > > d_mcode_code_phase_resampler;
    boost::shared_ptr< SubcarrierResamplerInterface< gr_complex > > d_mcode_subcarrier_resampler;
};

#endif //GNSS_SDR_GPS_L1_MCODE_CODELESS_TRACKING_CC_H


