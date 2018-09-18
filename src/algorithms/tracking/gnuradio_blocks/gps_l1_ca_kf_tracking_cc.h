/*!
 * \file gps_l1_ca_kf_tracking_cc.cc
 * \brief Interface of a processing block of a DLL + Kalman carrier
 * tracking loop for GPS L1 C/A signals
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 * \author Jordi Vila-Valls 2018. jvila(at)cttc.es
 * \author Carles Fernandez-Prades 2018. cfernandez(at)cttc.es
 *
 * Reference:
 * J. Vila-Valls, P. Closas, M. Navarro and C. Fernandez-Prades,
 * "Are PLLs Dead? A Tutorial on Kalman Filter-based Techniques for Digital
 * Carrier Synchronization", IEEE Aerospace and Electronic Systems Magazine,
 * Vol. 32, No. 7, pp. 28–45, July 2017. DOI: 10.1109/MAES.2017.150260
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GPS_L1_CA_KF_TRACKING_CC_H
#define GNSS_SDR_GPS_L1_CA_KF_TRACKING_CC_H

#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_2nd_PLL_filter.h"
#include "cpu_multicorrelator_real_codes.h"
#include "bayesian_estimation.h"
#include <armadillo>
#include <gnuradio/block.h>
#include <fstream>
#include <map>
#include <string>

class Gps_L1_Ca_Kf_Tracking_cc;

typedef boost::shared_ptr<Gps_L1_Ca_Kf_Tracking_cc>
    gps_l1_ca_kf_tracking_cc_sptr;

gps_l1_ca_kf_tracking_cc_sptr
gps_l1_ca_kf_make_tracking_cc(uint32_t order,
    int64_t if_freq,
    int64_t fs_in, uint32_t vector_length,
    bool dump,
    std::string dump_filename,
    float pll_bw_hz,
    float early_late_space_chips,
    bool bce_run,
    uint32_t bce_ptrans,
    uint32_t bce_strans,
    int32_t bce_nu,
    int32_t bce_kappa);


/*!
 * \brief This class implements a DLL + PLL tracking loop block
 */
class Gps_L1_Ca_Kf_Tracking_cc : public gr::block
{
public:
    ~Gps_L1_Ca_Kf_Tracking_cc();

    void set_channel(uint32_t channel);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();

    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items);

    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

private:
    friend gps_l1_ca_kf_tracking_cc_sptr
    gps_l1_ca_kf_make_tracking_cc(uint32_t order,
        int64_t if_freq,
        int64_t fs_in, uint32_t vector_length,
        bool dump,
        std::string dump_filename,
        float dll_bw_hz,
        float early_late_space_chips,
        bool bce_run,
        uint32_t bce_ptrans,
        uint32_t bce_strans,
        int32_t bce_nu,
        int32_t bce_kappa);

    Gps_L1_Ca_Kf_Tracking_cc(uint32_t order,
        int64_t if_freq,
        int64_t fs_in, uint32_t vector_length,
        bool dump,
        std::string dump_filename,
        float dll_bw_hz,
        float early_late_space_chips,
        bool bce_run,
        uint32_t bce_ptrans,
        uint32_t bce_strans,
        int32_t bce_nu,
        int32_t bce_kappa);

    // tracking configuration vars
    uint32_t d_order;
    uint32_t d_vector_length;
    bool d_dump;

    Gnss_Synchro* d_acquisition_gnss_synchro;
    uint32_t d_channel;

    int64_t d_if_freq;
    int64_t d_fs_in;

    double d_early_late_spc_chips;

    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_rem_code_phase_chips;
    double d_rem_carr_phase_rad;

    // Kalman filter variables
    arma::mat kf_P_x_ini;  // initial state error covariance matrix
    arma::mat kf_P_x;      // state error covariance matrix
    arma::mat kf_P_x_pre;  // Predicted state error covariance matrix
    arma::mat kf_P_y;      // innovation covariance matrix

    arma::mat kf_F;  // state transition matrix
    arma::mat kf_H;  // system matrix
    arma::mat kf_R;  // measurement error covariance matrix
    arma::mat kf_Q;  // system error covariance matrix

    arma::colvec kf_x;      // state vector
    arma::colvec kf_x_pre;  // predicted state vector
    arma::colvec kf_y;      // measurement vector
    arma::mat kf_K;         // Kalman gain matrix

    // Bayesian estimator
    Bayesian_estimator bayes_estimator;
    arma::mat kf_R_est;  // measurement error covariance
    uint32_t bayes_ptrans;
    uint32_t bayes_strans;
    int32_t bayes_nu;
    int32_t bayes_kappa;

    bool bayes_run;
    uint32_t kf_iter;

    // PLL and DLL filter library
    Tracking_2nd_DLL_filter d_code_loop_filter;
    // Tracking_2nd_PLL_filter d_carrier_loop_filter;

    // acquisition
    double d_acq_carrier_doppler_step_hz;
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;
    // correlator
    int32_t d_n_correlator_taps;
    float* d_ca_code;
    float* d_local_code_shift_chips;
    gr_complex* d_correlator_outs;
    cpu_multicorrelator_real_codes multicorrelator_cpu;

    // tracking vars
    double d_code_freq_chips;
    double d_code_phase_step_chips;
    double d_code_phase_rate_step_chips;
    double d_carrier_doppler_hz;
    double d_carrier_dopplerrate_hz2;
    double d_carrier_phase_step_rad;
    double d_acc_carrier_phase_rad;
    double d_carr_phase_error_rad;
    double d_carr_phase_sigma2;
    double d_code_phase_samples;
    double code_error_chips;
    double code_error_filt_chips;

    // PRN period in samples
    int32_t d_current_prn_length_samples;

    // processing samples counters
    uint64_t d_sample_counter;
    uint64_t d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int32_t d_cn0_estimation_counter;
    gr_complex* d_Prompt_buffer;
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_carrier_lock_threshold;
    int32_t d_carrier_lock_fail_counter;

    // control vars
    bool d_enable_tracking;
    bool d_pull_in;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;

    int32_t save_matfile();
};

#endif  // GNSS_SDR_GPS_L1_CA_KF_TRACKING_CC_H
