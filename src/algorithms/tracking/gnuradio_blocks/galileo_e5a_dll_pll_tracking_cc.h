/*!
 * \file galileo_e5a_dll_pll_tracking_cc.h
 * \brief Implementation of a code DLL + carrier PLL
 *  tracking block for Galileo E5a signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
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

#ifndef GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_CC_H_
#define GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_CC_H_

#include <fstream>
#include <map>
#include <string>
#include <gnuradio/block.h>
#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_2nd_PLL_filter.h"
#include "cpu_multicorrelator.h"

class Galileo_E5a_Dll_Pll_Tracking_cc;

typedef boost::shared_ptr<Galileo_E5a_Dll_Pll_Tracking_cc>
        galileo_e5a_dll_pll_tracking_cc_sptr;

galileo_e5a_dll_pll_tracking_cc_sptr
galileo_e5a_dll_pll_make_tracking_cc(long if_freq,
                                   long fs_in, unsigned
                                   int vector_length,
                                   bool dump,
                                   std::string dump_filename,
                                   float pll_bw_hz,
                                   float dll_bw_hz,
                                   float pll_bw_init_hz,
                                   float dll_bw_init_hz,
                                   int ti_ms,
                                   float early_late_space_chips);



/*!
 * \brief This class implements a DLL + PLL tracking loop block
 */
class Galileo_E5a_Dll_Pll_Tracking_cc: public gr::block
{
public:
    ~Galileo_E5a_Dll_Pll_Tracking_cc();

    void set_channel(unsigned int channel);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();

    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast (int noutput_items, gr_vector_int &ninput_items_required);

private:
    friend galileo_e5a_dll_pll_tracking_cc_sptr
    galileo_e5a_dll_pll_make_tracking_cc(long if_freq,
            long fs_in, unsigned
            int vector_length,
            bool dump,
            std::string dump_filename,
            float pll_bw_hz,
            float dll_bw_hz,
            float pll_bw_init_hz,
            float dll_bw_init_hz,
            int ti_ms,
            float early_late_space_chips);

    Galileo_E5a_Dll_Pll_Tracking_cc(long if_freq,
            long fs_in, unsigned
            int vector_length,
            bool dump,
            std::string dump_filename,
            float pll_bw_hz,
            float dll_bw_hz,
            float pll_bw_init_hz,
            float dll_bw_init_hz,
            int ti_ms,
            float early_late_space_chips);
    void acquire_secondary();
    // tracking configuration vars
    unsigned int d_vector_length;
    int d_current_ti_ms;
    int d_ti_ms;
    bool d_dump;


    Gnss_Synchro* d_acquisition_gnss_synchro;
    unsigned int d_channel;
    long d_if_freq;
    long d_fs_in;

    double d_early_late_spc_chips;
    double d_dll_bw_hz;
    double d_pll_bw_hz;
    double d_dll_bw_init_hz;
    double d_pll_bw_init_hz;

    gr_complex* d_codeQ;
    gr_complex* d_codeI;

    gr_complex d_Early;
    gr_complex d_Prompt;
    gr_complex d_Late;
    gr_complex d_Prompt_data;

    gr_complex* d_Single_Early;
    gr_complex* d_Single_Prompt;
    gr_complex* d_Single_Late;
    gr_complex* d_Single_Prompt_data;


    float tmp_E;
    float tmp_P;
    float tmp_L;
    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_rem_code_phase_chips;
    double d_rem_carr_phase_rad;

    // PLL and DLL filter library
    Tracking_2nd_DLL_filter d_code_loop_filter;
    Tracking_2nd_PLL_filter d_carrier_loop_filter;

    // acquisition
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;
    // correlator
    int d_n_correlator_taps;
    float* d_local_code_shift_chips;
    gr_complex* d_correlator_outs;
    cpu_multicorrelator multicorrelator_cpu_I;
    cpu_multicorrelator multicorrelator_cpu_Q;

    // tracking vars
    double d_code_freq_chips;
    double d_carrier_doppler_hz;
    double d_acc_carrier_phase_rad;
    double d_code_phase_samples;
    double d_acc_code_phase_secs;
    double d_code_error_filt_secs;
    double d_code_phase_step_chips;
    double d_carrier_phase_step_rad;


    //PRN period in samples
    int d_current_prn_length_samples;

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
    int d_state;
    bool d_first_transition;

    // Secondary code acquisition
    bool d_secondary_lock;
    int d_secondary_delay;
    int d_integration_counter;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;

    int save_matfile();
};

#endif /* GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_CC_H_ */
