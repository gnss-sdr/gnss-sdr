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

#ifndef GNSS_SDR_GPS_L1_CA_TCP_CONNECTOR_TRACKING_CC_H
#define GNSS_SDR_GPS_L1_CA_TCP_CONNECTOR_TRACKING_CC_H

#include "cpu_multicorrelator.h"
#include "gnss_block_interface.h"
#include "gnss_synchro.h"
#include "tcp_communication.h"
#include <gnuradio/block.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>  // for volk_gnsssdr::vector
#include <fstream>
#include <map>
#include <string>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_gnuradio_blocks
 * \{ */


class Gps_L1_Ca_Tcp_Connector_Tracking_cc;

using gps_l1_ca_tcp_connector_tracking_cc_sptr = gnss_shared_ptr<Gps_L1_Ca_Tcp_Connector_Tracking_cc>;

gps_l1_ca_tcp_connector_tracking_cc_sptr
gps_l1_ca_tcp_connector_make_tracking_cc(
    int64_t fs_in, uint32_t vector_length,
    bool dump,
    const std::string &dump_filename,
    float early_late_space_chips,
    size_t port_ch0);


/*!
 * \brief This class implements a DLL + PLL tracking loop block
 */
class Gps_L1_Ca_Tcp_Connector_Tracking_cc : public gr::block
{
public:
    ~Gps_L1_Ca_Tcp_Connector_Tracking_cc();

    void set_channel(uint32_t channel);
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
    gps_l1_ca_tcp_connector_make_tracking_cc(
        int64_t fs_in, uint32_t vector_length,
        bool dump,
        const std::string &dump_filename,
        float early_late_space_chips,
        size_t port_ch0);

    Gps_L1_Ca_Tcp_Connector_Tracking_cc(
        int64_t fs_in, uint32_t vector_length,
        bool dump,
        const std::string &dump_filename,
        float early_late_space_chips,
        size_t port_ch0);

    volk_gnsssdr::vector<gr_complex> d_ca_code;
    // correlator
    volk_gnsssdr::vector<float> d_local_code_shift_chips;
    volk_gnsssdr::vector<gr_complex> d_correlator_outs;
    volk_gnsssdr::vector<gr_complex> d_Prompt_buffer;
    Cpu_Multicorrelator multicorrelator_cpu;
    Tcp_Communication d_tcp_com;
    Gnss_Synchro *d_acquisition_gnss_synchro;
    // tracking configuration vars

    gr_complex *d_Early;
    gr_complex *d_Prompt;
    gr_complex *d_Late;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;

    double d_early_late_spc_chips;
    double d_code_phase_step_chips;
    double d_rem_code_phase_samples;
    double d_next_rem_code_phase_samples;
    double d_code_freq_hz;
    double d_carrier_doppler_hz;
    double d_acc_carrier_phase_rad;
    double d_code_phase_samples;
    double d_sample_counter_seconds;

    int64_t d_fs_in;
    uint64_t d_sample_counter;
    uint64_t d_acq_sample_stamp;

    size_t d_port_ch0;
    size_t d_port;

    uint32_t d_vector_length;
    uint32_t d_channel;

    int32_t d_correlation_length_samples;
    int32_t d_n_correlator_taps;
    int32_t d_listen_connection;
    int32_t d_current_prn_length_samples;
    int32_t d_next_prn_length_samples;
    int32_t d_cn0_estimation_counter;
    int32_t d_carrier_lock_fail_counter;

    float d_rem_carr_phase_rad;
    float d_acq_code_phase_samples;
    float d_acq_carrier_doppler_hz;
    float d_carrier_lock_test;
    float d_CN0_SNV_dB_Hz;
    float d_carrier_lock_threshold;
    float d_control_id;

    bool d_enable_tracking;
    bool d_pull_in;
    bool d_dump;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L1_CA_TCP_CONNECTOR_TRACKING_CC_H
