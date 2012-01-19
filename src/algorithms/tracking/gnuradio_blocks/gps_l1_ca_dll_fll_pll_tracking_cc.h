/*!
 * \file gps_l1_ca_dll_fll_pll_tracking_cc.h
 * \brief Interface of a code DLL + carrier FLL/PLL tracking block
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * This is the interface of a  code Delay Locked Loop (DLL) +
 * carrier Phase Locked Loop (PLL) helped with a carrier Frequency Locked
 * Loop (FLL) according to the algorithms described in:
 * E.D. Kaplan and C. Hegarty, Understanding GPS. Principles and
 * Applications, Second Edition, Artech House Publishers, 2005.
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

#ifndef GNSS_SDR_GPS_L1_CA_DLL_FLL_PLL_TRACKING_CC_H
#define	GNSS_SDR_GPS_L1_CA_DLL_FLL_PLL_TRACKING_CC_H

#include <fstream>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <gnuradio/gr_block.h>
#include <gnuradio/gr_msg_queue.h>
//#include <gnuradio/gr_sync_decimator.h>
#include "concurrent_queue.h"
#include "gps_sdr_signal_processing.h"
#include "gnss_satellite.h"
#include "tracking_FLL_PLL_filter.h"


//#include "GPS_L1_CA.h"

class Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc;
typedef boost::shared_ptr<Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc>
gps_l1_ca_dll_fll_pll_tracking_cc_sptr;

gps_l1_ca_dll_fll_pll_tracking_cc_sptr
gps_l1_ca_dll_fll_pll_make_tracking_cc(Gnss_Satellite satellite,
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        gr_msg_queue_sptr queue,
        bool dump,
        std::string dump_filename,
        int order,
        float fll_bw_hz,
        float pll_bw_hz,
        float dll_bw_hz,
        float early_late_space_chips);

//class gps_l1_ca_dll_pll_tracking_cc: public gr_sync_decimator

/*!
 * \brief This class implements a DLL and a FLL assisted PLL tracking loop block
 */
class Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc: public gr_block
{
public:

    ~Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc();

    void set_satellite(Gnss_Satellite satellite);
    void set_channel(unsigned int channel);
    void set_acq_code_phase(float code_phase);
    void set_acq_doppler(float doppler);
    void start_tracking();
    void update_local_code();
    void update_local_carrier();
    void set_FLL_and_PLL_BW(float fll_bw_hz,float pll_bw_hz);
    void set_acq_sample_stamp(unsigned long int sample_stamp);
    void set_channel_queue(concurrent_queue<int> *channel_internal_queue);

    /*
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


private:

    friend gps_l1_ca_dll_fll_pll_tracking_cc_sptr
    gps_l1_ca_dll_fll_pll_make_tracking_cc(Gnss_Satellite satellite,
            long if_freq,
            long fs_in, unsigned
            int vector_length,
            gr_msg_queue_sptr queue,
            bool dump,
            std::string dump_filename,
            int order,
            float fll_bw_hz,
            float pll_bw_hz,
            float dll_bw_hz,
            float early_late_space_chips);

    Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc(Gnss_Satellite satellite,
            long if_freq,
            long fs_in, unsigned
            int vector_length,
            gr_msg_queue_sptr queue,
            bool dump,
            std::string dump_filename,
            int order,
            float fll_bw_hz,
            float pll_bw_hz,
            float dll_bw_hz,
            float early_late_space_chips);

    void CN0_estimation_and_lock_detectors();

    // class private vars
    gr_msg_queue_sptr d_queue;
    concurrent_queue<int> *d_channel_internal_queue;
    unsigned int d_vector_length;
    bool d_dump;
    Gnss_Satellite d_satellite;
    unsigned int d_channel;
    int d_last_seg;
    long d_if_freq;
    long d_fs_in;

    gr_complex* d_ca_code;

    gr_complex* d_early_code;
    gr_complex* d_late_code;
    gr_complex* d_prompt_code;

    gr_complex* d_carr_sign;

    gr_complex d_Early;
    gr_complex d_Prompt;
    gr_complex d_Prompt_prev;
    gr_complex d_Late;

    float d_early_late_spc_chips;


    float d_carrier_doppler_hz;
    float d_code_freq_hz;
    float d_code_phase_samples;
    int d_current_prn_length_samples;
    int d_next_prn_length_samples;
    int d_FLL_wait;
    float d_rem_carr_phase;
    float d_rem_code_phase_samples;
    float d_next_rem_code_phase_samples;
    bool d_pull_in;

    // acquisition
    float d_acq_code_phase_samples;
    float d_acq_carrier_doppler_hz;

    // FLL + PLL filter
    float d_FLL_discriminator_hz; // This is a class variable because FLL needs to have memory
    Tracking_FLL_PLL_filter d_carrier_loop_filter;
    float d_acc_carrier_phase_rad;

    unsigned long int d_sample_counter;
    double d_sample_counter_seconds;

    unsigned long int d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int d_cn0_estimation_counter;
    gr_complex* d_Prompt_buffer;
    float d_carrier_lock_test;
    float d_CN0_SNV_dB_Hz;

    float d_carrier_lock_threshold;

    int d_carrier_lock_fail_counter;

    bool d_enable_tracking;

    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif //GNSS_SDR_GPS_L1_CA_DLL_FLL_PLL_TRACKING_CC_H
