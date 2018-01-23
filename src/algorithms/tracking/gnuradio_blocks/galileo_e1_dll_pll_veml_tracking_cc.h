/*!
 * \file galileo_e1_dll_pll_veml_tracking_cc.h
 * \brief Implementation of a code DLL + carrier PLL VEML (Very Early
 *  Minus Late) tracking block for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#ifndef GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_CC_H
#define GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_CC_H

#include <fstream>
#include <string>
#include <map>
#include <gnuradio/block.h>
#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_2nd_PLL_filter.h"
#include "cpu_multicorrelator_real_codes.h"

class galileo_e1_dll_pll_veml_tracking_cc;

typedef boost::shared_ptr<galileo_e1_dll_pll_veml_tracking_cc> galileo_e1_dll_pll_veml_tracking_cc_sptr;

galileo_e1_dll_pll_veml_tracking_cc_sptr
galileo_e1_dll_pll_veml_make_tracking_cc(long if_freq,
                                   long fs_in, unsigned
                                   int vector_length,
                                   bool dump,
                                   std::string dump_filename,
                                   float pll_bw_hz,
                                   float dll_bw_hz,
                                   float pll_bw_narrow_hz,
                                   float dll_bw_narrow_hz,
                                   float early_late_space_chips,
                                   float very_early_late_space_chips,
                                   float early_late_space_narrow_chips,
                                   float very_early_late_space_narrow_chips,
                                   int extend_correlation_symbols,
                                   bool track_pilot);

/*!
 * \brief This class implements a code DLL + carrier PLL VEML (Very Early
 *  Minus Late) tracking block for Galileo E1 signals
 */
class galileo_e1_dll_pll_veml_tracking_cc: public gr::block
{
public:
    ~galileo_e1_dll_pll_veml_tracking_cc();

    void set_channel(unsigned int channel);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();

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
    friend galileo_e1_dll_pll_veml_tracking_cc_sptr
    galileo_e1_dll_pll_veml_make_tracking_cc(long if_freq,
            long fs_in, unsigned
            int vector_length,
            bool dump,
            std::string dump_filename,
            float pll_bw_hz,
            float dll_bw_hz,
            float pll_bw_narrow_hz,
            float dll_bw_narrow_hz,
            float early_late_space_chips,
            float very_early_late_space_chips,
            float early_late_space_narrow_chips,
            float very_early_late_space_narrow_chips,
            int extend_correlation_symbols,
            bool track_pilot);

    galileo_e1_dll_pll_veml_tracking_cc(long if_freq,
            long fs_in, unsigned
            int vector_length,
            bool dump,
            std::string dump_filename,
            float pll_bw_hz,
            float dll_bw_hz,
            float pll_bw_narrow_hz,
            float dll_bw_narrow_hz,
            float early_late_space_chips,
            float very_early_late_space_chips,
            float early_late_space_narrow_chips,
            float very_early_late_space_narrow_chips,
            int extend_correlation_symbols,
            bool track_pilot);

    bool cn0_and_tracking_lock_status();
    void do_correlation_step(const gr_complex* input_samples);
    void run_dll_pll(bool disable_costas_loop);
    void update_local_code();
    void update_local_carrier();
    bool acquire_secondary();

    void clear_tracking_vars();

    void log_data();

    // tracking configuration vars
    unsigned int d_vector_length;
    bool d_dump;

    Gnss_Synchro* d_acquisition_gnss_synchro;
    unsigned int d_channel;
    long d_if_freq;
    long d_fs_in;

    //tracking state machine
    int d_state;

    //Integration period in samples
    int d_correlation_length_samples;
    int d_n_correlator_taps;
    double d_early_late_spc_chips;
    double d_very_early_late_spc_chips;

    double d_early_late_spc_narrow_chips;
    double d_very_early_late_spc_narrow_chips;

    float* d_tracking_code;
    float* d_data_code;
    float* d_local_code_shift_chips;
    gr_complex* d_correlator_outs;
    cpu_multicorrelator_real_codes multicorrelator_cpu;
    //todo: currently the multicorrelator does not support adding extra correlator
    //with different local code, thus we need extra multicorrelator instance.
    //Implement this functionality inside multicorrelator class
    //as an enhancement to increase the performance
    float* d_local_code_data_shift_chips;
    cpu_multicorrelator_real_codes correlator_data_cpu; //for data channel

    gr_complex *d_Very_Early;
    gr_complex *d_Early;
    gr_complex *d_Prompt;
    gr_complex *d_Late;
    gr_complex *d_Very_Late;

    int d_extend_correlation_symbols;
    int d_extend_correlation_symbols_count;
    bool d_enable_extended_integration;
    int d_current_symbol;

    gr_complex d_VE_accu;
    gr_complex d_E_accu;
    gr_complex d_P_accu;
    gr_complex d_L_accu;
    gr_complex d_VL_accu;

    bool d_track_pilot;
    gr_complex *d_Prompt_Data;

    double d_code_phase_step_chips;
    double d_carrier_phase_step_rad;
    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_rem_carr_phase_rad;

    // PLL and DLL filter library
    Tracking_2nd_DLL_filter d_code_loop_filter;
    Tracking_2nd_PLL_filter d_carrier_loop_filter;

    // acquisition
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;

    // tracking parameters
    float d_dll_bw_hz;
    float d_pll_bw_hz;
    float d_dll_bw_narrow_hz;
    float d_pll_bw_narrow_hz;
    // tracking vars
    double d_carr_error_hz;
    double d_carr_error_filt_hz;
    double d_code_error_chips;
    double d_code_error_filt_chips;

    double d_K_blk_samples;

    double d_code_freq_chips;
    double d_carrier_doppler_hz;
    double d_acc_carrier_phase_rad;
    double d_rem_code_phase_chips;
    double d_code_phase_samples;

    //PRN period in samples
    int d_current_prn_length_samples;

    //processing samples counters
    unsigned long int d_sample_counter;
    unsigned long int d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int d_cn0_estimation_counter;
    std::deque<gr_complex> d_Prompt_buffer_deque;
    gr_complex* d_Prompt_buffer;
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_carrier_lock_threshold;
    int d_carrier_lock_fail_counter;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;

    int save_matfile();
};

#endif //GNSS_SDR_GALILEO_E1_DLL_PLL_VEML_TRACKING_CC_H
