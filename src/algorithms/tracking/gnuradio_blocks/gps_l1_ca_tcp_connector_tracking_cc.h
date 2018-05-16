/*!
 * \file gps_l1_ca_tcp_connector_tracking_cc.h
 * \brief Interface of a TCP connector block based on code DLL + carrier PLL
 * \author David Pubill, 2012. dpubill(at)cttc.es
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency Approach,
 * Birkhauser, 2007
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

#ifndef GNSS_SDR_GPS_L1_CA_TCP_CONNECTOR_TRACKING_CC_H
#define GNSS_SDR_GPS_L1_CA_TCP_CONNECTOR_TRACKING_CC_H

#include "cpu_multicorrelator.h"
#include "gnss_synchro.h"
#include "tcp_communication.h"
#include <gnuradio/block.h>
#include <fstream>
#include <map>
#include <string>


class Gps_L1_Ca_Tcp_Connector_Tracking_cc;

typedef boost::shared_ptr<Gps_L1_Ca_Tcp_Connector_Tracking_cc> gps_l1_ca_tcp_connector_tracking_cc_sptr;

gps_l1_ca_tcp_connector_tracking_cc_sptr
gps_l1_ca_tcp_connector_make_tracking_cc(long if_freq,
    long fs_in, unsigned int vector_length,
    bool dump,
    std::string dump_filename,
    float early_late_space_chips,
    size_t port_ch0);


/*!
 * \brief This class implements a DLL + PLL tracking loop block
 */
class Gps_L1_Ca_Tcp_Connector_Tracking_cc : public gr::block
{
public:
    ~Gps_L1_Ca_Tcp_Connector_Tracking_cc();

    void set_channel(unsigned int channel);
    void set_gnss_synchro(Gnss_Synchro *p_gnss_synchro);
    void start_tracking();

    /*
     * \brief just like gr_block::general_work, only this arranges to call consume_each for you
     *
     * The user must override work to define the signal processing code
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast(int noutput_items, gr_vector_int &ninput_items_required);

private:
    friend gps_l1_ca_tcp_connector_tracking_cc_sptr
    gps_l1_ca_tcp_connector_make_tracking_cc(long if_freq,
        long fs_in, unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float early_late_space_chips,
        size_t port_ch0);

    Gps_L1_Ca_Tcp_Connector_Tracking_cc(long if_freq,
        long fs_in, unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float early_late_space_chips,
        size_t port_ch0);

    // tracking configuration vars
    unsigned int d_vector_length;
    bool d_dump;

    Gnss_Synchro *d_acquisition_gnss_synchro;
    unsigned int d_channel;

    long d_if_freq;
    long d_fs_in;
    int d_correlation_length_samples;
    int d_n_correlator_taps;
    double d_early_late_spc_chips;

    double d_code_phase_step_chips;

    gr_complex *d_ca_code;

    gr_complex *d_Early;
    gr_complex *d_Prompt;
    gr_complex *d_Late;

    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_next_rem_code_phase_samples;
    float d_rem_carr_phase_rad;

    // acquisition
    float d_acq_code_phase_samples;
    float d_acq_carrier_doppler_hz;
    // correlator
    float *d_local_code_shift_chips;
    gr_complex *d_correlator_outs;
    cpu_multicorrelator multicorrelator_cpu;

    // tracking vars
    double d_code_freq_hz;
    double d_carrier_doppler_hz;
    double d_acc_carrier_phase_rad;
    double d_code_phase_samples;
    size_t d_port_ch0;
    size_t d_port;
    int d_listen_connection;
    float d_control_id;
    tcp_communication d_tcp_com;

    //PRN period in samples
    int d_current_prn_length_samples;
    int d_next_prn_length_samples;
    double d_sample_counter_seconds;

    //processing samples counters
    unsigned long int d_sample_counter;
    unsigned long int d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int d_cn0_estimation_counter;
    gr_complex *d_Prompt_buffer;
    float d_carrier_lock_test;
    float d_CN0_SNV_dB_Hz;
    float d_carrier_lock_threshold;
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

#endif  //GNSS_SDR_GPS_L1_CA_TCP_CONNECTOR_TRACKING_CC_H
