/*!
 * \file gps_l1_ca_dll_pll_tracking_cc.h
 * \brief code DLL + carrier PLL
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in [1]
 * [1] K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency Approach, Birkha user, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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
#ifndef GPS_L1_CA_DLL_PLL_TRACKING_CC_H
#define	GPS_L1_CA_DLL_PLL_TRACKING_CC_H

#include <fstream>

#include <gnuradio/gr_block.h>
#include <gnuradio/gr_msg_queue.h>
//#include <gnuradio/gr_sync_decimator.h>

#include "gps_sdr_signal_processing.h"

#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "concurrent_queue.h"

// #include "gps_sdr_structs.h"
// #include "gps_sdr_channel.h"

class gps_l1_ca_dll_pll_tracking_cc;
typedef boost::shared_ptr<gps_l1_ca_dll_pll_tracking_cc>
        gps_l1_ca_dll_pll_tracking_cc_sptr;

gps_l1_ca_dll_pll_tracking_cc_sptr
gps_l1_ca_dll_pll_make_tracking_cc(unsigned int satellite, long if_freq,
                                   long fs_in, unsigned
                                   int vector_length,
                                   gr_msg_queue_sptr queue, bool dump);

//class gps_l1_ca_dll_pll_tracking_cc: public gr_sync_decimator
class gps_l1_ca_dll_pll_tracking_cc: public gr_block
{

private:

    friend gps_l1_ca_dll_pll_tracking_cc_sptr
    gps_l1_ca_dll_pll_make_tracking_cc(unsigned int satellite, long if_freq,
                                       long fs_in, unsigned
                                       int vector_length,
                                       gr_msg_queue_sptr queue, bool dump);

    gps_l1_ca_dll_pll_tracking_cc(unsigned int satellite, long if_freq,
                                  long fs_in, unsigned
                                  int vector_length, gr_msg_queue_sptr queue,
                                  bool dump);

    void calculate_lopp_coef(float* tau1, float* tau2, float lbw, float zeta,
                             float k);
    void update_local_code_refs();
    void update_local_carrier();

    // class private vars
    gr_msg_queue_sptr d_queue;
    concurrent_queue<int> *d_channel_internal_queue;
    unsigned int d_vector_length;
    bool d_dump;
    unsigned int d_satellite;
    unsigned int d_channel; //?
    int d_last_seg;
    long d_if_freq;
    long d_fs_in;
    float d_tau1_code;
    float d_tau2_code;

    float d_tau1_carr;
    float d_tau2_carr;

    float d_early_late_spc;
    float d_pdi_code;
    float d_pdi_carr;
    float d_dllnoisebandwidth;
    float d_dlldampingratio;

    float d_pllnoisebandwidth;
    float d_plldampingratio;

    unsigned int d_code_length;
    unsigned int d_blk_size;
    float d_code_phase_step;

    gr_complex* d_ca_code;

    gr_complex* d_early_code;
    gr_complex* d_late_code;
    gr_complex* d_prompt_code;

    gr_complex* d_carr_sign;
    gr_complex* d_bb_sign;

    float d_code_freq;
    float d_rem_code_phase;

    float d_rem_carr_phase;

    float d_old_code_nco;
    float d_old_code_error;

    float d_old_carr_nco;
    float d_old_carr_error;

    float d_code_phase;
    float d_carrier_doppler;
    float d_carr_freq_basis;

    //std::complex<double> d_E;
    //std::complex<double> d_P;
    //std::complex<double> d_L;
    float d_E_I;
    float d_E_Q;
    float d_P_I;
    float d_P_Q;
    float d_L_I;
    float d_L_Q;

    float d_absolute_code_phase_samples;
    float d_code_phase_ms;

    unsigned int d_blksize;

    unsigned long int d_sample_counter;
    unsigned long int d_acq_sample_stamp;
    unsigned long int d_loops_count;

    // CN0 estimation and lock detector
    int d_cn0_estimation_counter;
    float* d_P_I_buffer;
    float* d_P_Q_buffer;
    float d_carrier_lock_test;
    float d_SNR_SNV;
    float d_SNR_MM;
    float d_SNR_SNV_dB_Hz;

    float d_carrier_lock_threshold;

    int d_carrier_lock_fail_counter;

    bool d_enable_tracking;
    bool d_pull_in;

    std::string d_dump_filename;
    std::ofstream d_dump_file;

public:

    ~gps_l1_ca_dll_pll_tracking_cc();

    void set_satellite(unsigned int satellite);
    void set_channel(unsigned int channel);
    void set_acq_code_phase(float code_phase);
    void set_acq_doppler(float doppler);
    void start_tracking();

    void set_acq_sample_stamp(unsigned long int sample_stamp)
    {
        d_acq_sample_stamp = sample_stamp;
    }

    void set_channel_queue(concurrent_queue<int> *channel_internal_queue)
    {
        d_channel_internal_queue = channel_internal_queue;
    }

    /*!
     * \brief just like gr_block::general_work, only this arranges to call consume_each for you
     *
     * The user must override work to define the signal processing code
     */
    //virtual int work (int noutput_items,
    //                  gr_vector_const_void_star &input_items,
    //                gr_vector_void_star &output_items) = 0;

   //int work(int noutput_items, gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    int general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast (int noutput_items, gr_vector_int &ninput_items_required);

};

#endif //GPS_L1_CA_DLL_PLL_TRACKING_CC_H
