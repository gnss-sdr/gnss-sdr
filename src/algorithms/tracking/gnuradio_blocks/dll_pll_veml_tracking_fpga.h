/*!
 * \file gps_l1_ca_dll_pll_tracking_cc.h
 * \brief Interface of a code DLL + carrier PLL tracking block
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *         Cillian O'Driscoll, 2017. cillian.odriscoll(at)gmail.com
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

#ifndef GNSS_SDR_DLL_PLL_VEML_TRACKING_FPGA_H
#define GNSS_SDR_DLL_PLL_VEML_TRACKING_FPGA_H

#include <fstream>
#include <map>
#include <string>
#include <gnuradio/block.h>
#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_2nd_PLL_filter.h"
#include "fpga_multicorrelator.h"

typedef struct
{
    /* DLL/PLL tracking configuration */
    double fs_in;
    unsigned int vector_length;
    bool dump;
    std::string dump_filename;
    float pll_bw_hz;
    float dll_bw_hz;
    float pll_bw_narrow_hz;
    float dll_bw_narrow_hz;
    float early_late_space_chips;
    float very_early_late_space_chips;
    float early_late_space_narrow_chips;
    float very_early_late_space_narrow_chips;
    int extend_correlation_symbols;
    bool track_pilot;
    char system;
    char signal[3];
    std::string device_name;
    unsigned int device_base;
    unsigned int code_length;
    int* ca_codes;
    //int f_if;
} dllpllconf_fpga_t;

class dll_pll_veml_tracking_fpga;

typedef boost::shared_ptr<dll_pll_veml_tracking_fpga>
dll_pll_veml_tracking_fpga_sptr;

//dll_pll_veml_tracking_fpga_sptr
//dll_pll_veml_make_tracking_fpga(long if_freq,
//                                   long fs_in, unsigned
//                                   int vector_length,
//                                   bool dump,
//                                   std::string dump_filename,
//                                   float pll_bw_hz,
//                                   float dll_bw_hz,
//                                   float early_late_space_chips,
//                                   std::string device_name,
//                                   unsigned int device_base,
//                                   int* ca_codes,
//                                   unsigned int code_length);

dll_pll_veml_tracking_fpga_sptr dll_pll_veml_make_tracking_fpga(dllpllconf_fpga_t conf_);


/*!
 * \brief This class implements a DLL + PLL tracking loop block
 */
class dll_pll_veml_tracking_fpga: public gr::block
{
public:
    ~dll_pll_veml_tracking_fpga();

    void set_channel(unsigned int channel);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();

    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);
    
    void reset(void);

private:
//    friend dll_pll_veml_tracking_fpga_sptr
//    dll_pll_veml_make_tracking_fpga(long if_freq,
//            long fs_in, unsigned
//            int vector_length,
//            bool dump,
//            std::string dump_filename,
//            float pll_bw_hz,
//            float dll_bw_hz,
//            float early_late_space_chips,
//            std::string device_name,
//            unsigned int device_base,
//            int* ca_codes,
//            unsigned int code_length);
    friend dll_pll_veml_tracking_fpga_sptr dll_pll_veml_make_tracking_fpga(dllpllconf_fpga_t conf_);

    dll_pll_veml_tracking_fpga(dllpllconf_fpga_t conf_);
//    dll_pll_veml_tracking_fpga(//long if_freq,
//            long fs_in, unsigned
//            int vector_length,
//            bool dump,
//            std::string dump_filename,
//            float pll_bw_hz,
//            float dll_bw_hz,
//            float early_late_space_chips,
//            std::string device_name,
//            unsigned int device_base,
//            int* ca_codes,
//            unsigned int code_length);


    dllpllconf_fpga_t trk_parameters;

    // tracking configuration vars
    unsigned int d_vector_length;
    bool d_dump;

    Gnss_Synchro* d_acquisition_gnss_synchro;
    unsigned int d_channel;

    //long d_if_freq;
    long d_fs_in;

    double d_early_late_spc_chips;

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
    //float* d_ca_code;
    //int* d_ca_code_16sc;
    
    float* d_local_code_shift_chips;
    gr_complex* d_correlator_outs;
    std::shared_ptr<fpga_multicorrelator_8sc> multicorrelator_fpga_8sc;
    
    // tracking vars
    double d_code_freq_chips;
    double d_code_phase_step_chips;
    double d_carrier_doppler_hz;
    double d_carrier_phase_step_rad;
    double d_acc_carrier_phase_rad;
    double d_code_phase_samples;

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
    bool d_enable_tracking;
    bool d_pull_in;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;
    
    // extra
    int d_correlation_length_samples;
    unsigned long int d_sample_counter_next;
    double d_rem_carrier_phase_rad;
    
    double d_K_blk_samples_previous;
    int d_offset_sample_previous;
    
};

#endif //GNSS_SDR_DLL_PLL_VEML_TRACKING_FPGA_H
