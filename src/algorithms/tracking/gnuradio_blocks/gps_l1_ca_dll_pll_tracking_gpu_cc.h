/*!
 * \file gps_l1_ca_dll_pll_tracking_gpu_cc.h
 * \brief Implementation of a code DLL + carrier PLL tracking block, GPU ACCELERATED
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency Approach,
 * Birkhauser, 2007
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

#ifndef GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_GPU_CC_H
#define GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_GPU_CC_H

#include "cuda_multicorrelator.h"
#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_FLL_PLL_filter.h"
#include <gnuradio/block.h>
#include <fstream>
#include <map>
#include <string>


class Gps_L1_Ca_Dll_Pll_Tracking_GPU_cc;

typedef boost::shared_ptr<Gps_L1_Ca_Dll_Pll_Tracking_GPU_cc>
    gps_l1_ca_dll_pll_tracking_gpu_cc_sptr;

gps_l1_ca_dll_pll_tracking_gpu_cc_sptr
gps_l1_ca_dll_pll_make_tracking_gpu_cc(long if_freq,
    long fs_in,
    unsigned int vector_length,
    bool dump,
    std::string dump_filename,
    float pll_bw_hz,
    float dll_bw_hz,
    float early_late_space_chips);


/*!
 * \brief This class implements a DLL + PLL tracking loop block
 */
class Gps_L1_Ca_Dll_Pll_Tracking_GPU_cc : public gr::block
{
public:
    ~Gps_L1_Ca_Dll_Pll_Tracking_GPU_cc();

    void set_channel(unsigned int channel);
    void set_gnss_synchro(Gnss_Synchro *p_gnss_synchro);
    void start_tracking();

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

private:
    friend gps_l1_ca_dll_pll_tracking_gpu_cc_sptr
    gps_l1_ca_dll_pll_make_tracking_gpu_cc(long if_freq,
        long fs_in,
        unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float early_late_space_chips);

    Gps_L1_Ca_Dll_Pll_Tracking_GPU_cc(long if_freq,
        long fs_in,
        unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float early_late_space_chips);
    void update_local_code();
    void update_local_carrier();

    // tracking configuration vars
    unsigned int d_vector_length;
    bool d_dump;

    Gnss_Synchro *d_acquisition_gnss_synchro;
    unsigned int d_channel;

    long d_if_freq;
    long d_fs_in;

    double d_early_late_spc_chips;
    int d_n_correlator_taps;


    //GPU HOST PINNED MEMORY IN/OUT VECTORS
    gr_complex *in_gpu;
    float *d_local_code_shift_chips;
    gr_complex *d_correlator_outs;
    cuda_multicorrelator *multicorrelator_gpu;
    gr_complex *d_ca_code;

    gr_complex *d_Early;
    gr_complex *d_Prompt;
    gr_complex *d_Late;


    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_rem_code_phase_chips;
    double d_rem_carrier_phase_rad;

    // PLL and DLL filter library
    Tracking_2nd_DLL_filter d_code_loop_filter;
    Tracking_FLL_PLL_filter d_carrier_loop_filter;

    // acquisition
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;

    // tracking vars
    double d_code_freq_chips;
    double d_code_phase_step_chips;
    double d_carrier_doppler_hz;
    double d_carrier_phase_step_rad;
    double d_acc_carrier_phase_cycles;
    double d_code_phase_samples;
    double d_pll_to_dll_assist_secs_Ti;

    //Integration period in samples
    int d_correlation_length_samples;

    //processing samples counters
    unsigned long int d_sample_counter;
    unsigned long int d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int d_cn0_estimation_counter;
    gr_complex *d_Prompt_buffer;
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
};

#endif  //GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_GPU_CC_H
