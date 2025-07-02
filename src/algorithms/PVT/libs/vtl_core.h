/*!
 * \file vtl_core.h
 * \brief Class that implements a Vector Tracking Loop (VTL)
 * \author Pedro Pereira, 2025. pereirapedrocp@gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_VTL_CORE_H
#define GNSS_SDR_VTL_CORE_H

#include "MATH_CONSTANTS.h"
#include "pvt_conf.h"
#include "rtklib.h"
#include "trackingcmd.h"
#include "vtl_data.h"
#include <armadillo>
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


class Vtl_Core
{
public:
    /**
     * Vector Tracking Loop (VTL) class constructor.
     * @param conf PVT configuration for VTL.
     */
    explicit Vtl_Core(const Pvt_Conf &conf);
    ~Vtl_Core();

    /**
     * Initializes VTL.
     * @param conf PVT configuration for VTL.
     */
    void vtl_init(const Pvt_Conf &conf);

    /**
     * Processes a new epoch of GNSS data.
     * @param rtklib_data Input data for the VTL.
     * @return True if successful, false otherwise.
     */
    bool vtl_work(const Vtl_Data &rtklib_data);

    // Accessors
    /**
     * Accessor for VTL tracking commands.
     * @return Vector of tracking commands.
     */
    const std::vector<TrackingCmd> &get_trk_cmd_outs() const;  // acessor for tracking comands
                                                               /**
                                                                * Accessor for VTL receiver PVT.
                                                                * @return Receiver paraters estimation.
                                                                */
    const arma::colvec &get_rx_pvt() const;                    // acessor for receiver PVT


private:
    // VTL configuration
    bool vtl_kinematic;
    double vtl_init_pos_ecef_var_m2;       // initial variance of receiver pos (m^2)
    double vtl_init_vel_ecef_var_m2s2;     // initial variance of receiver vel ((m/s)^2)
    double vtl_init_clk_b_var_m2;          // initial variance of receiver clock bias (m^2)
    double vtl_init_clk_d_var_m2s2;        // initial variance of receiver clock drift ((m/s)^2)
    double vtl_sys_acc_noise_var_m2s4;     // Receiver acceleration noise PSD
    double vtl_meas_prange_var_m2;         // nominal variance of pseudorange (m^2)
    double vtl_meas_prange_rate_var_m2s2;  // nominal variance of pseudorange rate ((m/s)^2)
    // Process noise covariance - Clock noise spectral densities
    double vtl_sys_clk_b_noise_var_m2;     // additive clock bias noise
    double vtl_sys_clk_d_noise_var_m2s2;   // additive clock drift noise
    static constexpr double h_0 = 2e-16;   // RWFM noise PSD
    static constexpr double h_m2 = 5e-20;  // WFM noise PSD
    static constexpr double S_f = 0.5 * h_0 * SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S;
    static constexpr double S_g = (2.0 * M_PI * M_PI / 3.0) * h_m2 * SPEED_OF_LIGHT_M_S * SPEED_OF_LIGHT_M_S;

    static constexpr u_int16_t num_recovery_epochs = 5;
    static constexpr double default_vtl_dt = 0.02;

    // tracking command from VTL to tracking channels
    std::vector<TrackingCmd> trk_cmd_outs;

    // per channel
    arma::colvec rho;          // geometric range
    arma::colvec rho_squared;  // geometric range
    arma::colvec u_x;          // LOS unit vector (x axis)
    arma::colvec u_y;          // LOS unit vector (y axis)
    arma::colvec u_z;          // LOS unit vector (z axis)

    // EKF
    arma::colvec ekf_X;            // state
    arma::mat ekf_F;               // state transition
    arma::mat ekf_F_t;             // transpose state transition
    arma::mat ekf_S;               // residual covariance
    arma::mat ekf_K;               // gain
    arma::mat ekf_J;               // jacobian
    arma::mat ekf_P;               // state estimate covariance
    arma::mat ekf_P_J_t;           // P * J.t()
    arma::mat ekf_P_aux;           // state estimate covariance auxiliar
    arma::mat ekf_eye;             // eye matrix
    arma::mat ekf_R;               // measurement noise covariance
    arma::mat ekf_Q;               // process noise
    arma::colvec ekf_X_updt;       // state update
    arma::colvec ekf_obs_Z;        // observed measurements
    arma::colvec ekf_comp_Z;       // computed measurements
    arma::colvec ekf_updt_comp_Z;  // updated computed measurment
    arma::colvec ekf_updt_comp_Z_copy;
    arma::colvec ekf_updt_comp_Z_last;  // last updated computed measurments
    arma::colvec ekf_preFit;            // measurements - observed minus computed (omc)
    arma::colvec ekf_postFit;           // measurements - updated omc

    const u_int8_t N_gps_ch;  // number of channels GPS
    const u_int8_t N_gal_ch;  // number of channels GAL
    bool dump_enabled;        // dump
    std::string dump_filename;
    const u_int8_t N_ch;                    // number of channels
    const u_int8_t N_st;                    // number of states
    const u_int8_t N_meas;                  // number of measurements
    u_int64_t epoch;                        // vtl epoch
    const u_int32_t N_meascov_updt = 1000;  // recorded prefit epochs
    u_int32_t N_meascov_updt_cnt;
    std::ofstream dump_file;
    const arma::uvec N_st_idx;

    const u_int8_t i_px = 0;      // position X
    const u_int8_t i_vx = 1;      // velocity X
    const u_int8_t i_py = 2;      // position Y
    const u_int8_t i_vy = 3;      // velocity Y
    const u_int8_t i_pz = 4;      // position Z
    const u_int8_t i_vz = 5;      // velocity Z
    const u_int8_t i_clkb_G = 6;  // clk bias GPS
    const u_int8_t i_clkb_E;      // clk bias GAL
    const u_int8_t i_clkd;        // clk drift

    arma::colvec vtl_feedback;  // VTL feedback indicator

    u_int8_t epochs_to_recover;  // VTL recovery from PVT fail

    double dt_s;

    // matrix indexants
    arma::uvec aidx;
    arma::uvec ext_aidx;


    /**
     * Reset VTL state.
     * @param rtk_data Input data for the VTL.
     */
    void vtl_reset(const Vtl_Data &rtk_data);

    /**
     * Update of EKF Transition matrix.
     * @param dt_s VTL epoch interval.
     */
    void update_process(double dt_s);

    /**
     * Update of EKF Jacobian matrix.
     * @param rtk_data Input data for the VTL.
     */
    void update_jacobian(const Vtl_Data &rtk_data);

    /**
     * Compute EKF prefit.
     * @param rtk_data Input data for the VTL.
     */
    void compute_prefit(const Vtl_Data &rtk_data);

    /**
     * Update of EKF Measurement Covariance matrix.
     * @param rtk_data Input data for the VTL.
     * @param meas_idx Indexant for measurements.
     * @param preFit_available LOS unit vector available for satellite selection.
     */
    void update_meas_cov(const Vtl_Data &rtk_data, const arma::uvec &meas_idx, bool preFit_available);

    /**
     * Send VTL feedback to tracking channels.
     * @param rtk_data Input data for the VTL.
     */
    void send_vtl_feedback(const Vtl_Data &rtk_data);

    /**
     * Satellite selection using fast weighted downdate method.
     * @param w Per-satellite weights.
     * @return Indexes of remained satellites.
     */
    const arma::vec ss_downdate_fast(const arma::vec &w);

    /**
     * Save VTL and RTKLib variables in file.
     * @param rtk_data Input data for the VTL.
     */
    void saveVTLdata(const Vtl_Data &rtk_data);
};

/** \} */
/** \} */
#endif  // GNSS_SDR_VTL_CORE_H