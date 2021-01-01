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

#ifndef GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_GPU_CC_H
#define GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_GPU_CC_H

#include "cuda_multicorrelator.h"
#include "gnss_block_interface.h"
#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_FLL_PLL_filter.h"
#include <gnuradio/block.h>
#include <fstream>
#include <map>
#include <string>
#include <vector>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_gnuradio_blocks
 * \{ */


class Gps_L1_Ca_Dll_Pll_Tracking_GPU_cc;

using gps_l1_ca_dll_pll_tracking_gpu_cc_sptr = gnss_shared_ptr<Gps_L1_Ca_Dll_Pll_Tracking_GPU_cc>;

gps_l1_ca_dll_pll_tracking_gpu_cc_sptr
gps_l1_ca_dll_pll_make_tracking_gpu_cc(
    int64_t fs_in,
    uint32_t vector_length,
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

    void set_channel(uint32_t channel);
    void set_gnss_synchro(Gnss_Synchro *p_gnss_synchro);
    void start_tracking();

    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

private:
    friend gps_l1_ca_dll_pll_tracking_gpu_cc_sptr
    gps_l1_ca_dll_pll_make_tracking_gpu_cc(
        int64_t fs_in,
        uint32_t vector_length,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float early_late_space_chips);

    Gps_L1_Ca_Dll_Pll_Tracking_GPU_cc(
        int64_t fs_in,
        uint32_t vector_length,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float early_late_space_chips);
    void update_local_code();
    void update_local_carrier();
    void check_carrier_phase_coherent_initialization();

    // PLL and DLL filter library
    Tracking_2nd_DLL_filter d_code_loop_filter;
    Tracking_FLL_PLL_filter d_carrier_loop_filter;

    Gnss_Synchro *d_acquisition_gnss_synchro;

    std::vector<gr_complex> d_Prompt_buffer;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;

    // tracking configuration vars
    int64_t d_if_freq;
    int64_t d_fs_in;
    double d_early_late_spc_chips;
    uint32_t d_vector_length;
    uint32_t d_channel;
    int32_t d_n_correlator_taps;

    // GPU HOST PINNED MEMORY IN/OUT VECTORS
    cuda_multicorrelator *multicorrelator_gpu;
    gr_complex *in_gpu;
    gr_complex *d_correlator_outs;
    gr_complex *d_ca_code;
    float *d_local_code_shift_chips;

    gr_complex *d_Early;
    gr_complex *d_Prompt;
    gr_complex *d_Late;

    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_rem_code_phase_chips;
    double d_rem_carrier_phase_rad;

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

    // control vars
    bool d_acc_carrier_phase_initialized;
    bool d_enable_tracking;
    bool d_pull_in;
    bool d_dump;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_GPU_CC_H
