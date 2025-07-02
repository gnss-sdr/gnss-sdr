/*!
 * \file vtl_core.cc
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

#include "vtl_core.h"
#include "gnss_sdr_filesystem.h"
#include <fstream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


Vtl_Core::Vtl_Core(const Pvt_Conf &conf)
    : N_gps_ch(conf.vtl_gps_channels),                      // allocated GPS channels
      N_gal_ch(conf.vtl_gal_channels),                      // allocated GAL channels
      dump_enabled(conf.vtl_dump),                          // vtl dump flag
      dump_filename(conf.vtl_dump_filename),                // vtl dump file name
      N_ch(conf.vtl_gps_channels + conf.vtl_gal_channels),  // total allocated channels
      N_st((conf.vtl_gal_channels > 0) ? 9 : 8),            // x, vx, y, vy, z, vz, b (gps), b (gal), d
      N_meas(N_ch * 2),                                     // 3 measurements (pr, prr, pracc) per channel
      N_st_idx(arma::linspace<arma::uvec>(0, N_st - 1, N_st)),
      i_clkb_E(i_clkb_G + 1),
      i_clkd((conf.vtl_gal_channels > 0) ? (i_clkb_G + 2) : (i_clkb_G + 1))
{
    if (dump_enabled == true)
        {
            if (dump_file.is_open() == false)
                {
                    try
                        {
                            dump_file.exceptions(std::ofstream::failbit | std::ofstream::badbit);
                            dump_file.open(dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "VTL lib dump enabled Log file: " << dump_filename.c_str();
                        }
                    catch (const std::ofstream::failure &e)
                        {
                            dump_enabled = false;
                            LOG(WARNING) << "Exception opening RTKLIB dump file " << e.what();
                        }
                }
        }
}

Vtl_Core::~Vtl_Core()
{
    if (dump_file.is_open() == true)
        {
            const auto pos = dump_file.tellp();
            try
                {
                    dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the VTL dump file " << ex.what();
                }
        }
}

void Vtl_Core::vtl_init(const Pvt_Conf &conf)
{
    epoch = 0;
    N_meascov_updt_cnt = 0;

    vtl_kinematic = conf.vtl_kinematic;
    vtl_init_pos_ecef_var_m2 = std::pow(conf.vtl_init_pos_ecef_sd_m, 2);
    vtl_init_vel_ecef_var_m2s2 = std::pow(conf.vtl_init_vel_ecef_sd_ms, 2);
    vtl_init_clk_b_var_m2 = std::pow(conf.vtl_init_clk_b_sd_m, 2);
    vtl_init_clk_d_var_m2s2 = std::pow(conf.vtl_init_clk_d_sd_ms, 2);
    vtl_sys_acc_noise_var_m2s4 = std::pow(conf.vtl_sys_acc_noise_sd_ms2, 2);
    vtl_sys_clk_b_noise_var_m2 = std::pow(conf.vtl_sys_clk_b_noise_sd_m, 2);
    vtl_sys_clk_d_noise_var_m2s2 = std::pow(conf.vtl_sys_clk_d_noise_sd_ms, 2);
    vtl_meas_prange_var_m2 = std::pow(conf.vtl_meas_prange_sd_m, 2);
    vtl_meas_prange_rate_var_m2s2 = std::pow(conf.vtl_meas_prange_rate_sd_ms, 2);

    ekf_X = arma::zeros(N_st, 1);
    ekf_X_updt = arma::zeros(N_st, 1);
    ekf_obs_Z = arma::zeros(N_meas, 1);
    ekf_comp_Z = arma::zeros(N_meas, 1);
    ekf_updt_comp_Z = arma::zeros(N_meas, 1);
    ekf_updt_comp_Z_copy = arma::zeros(N_meas, 1);
    ekf_updt_comp_Z_last = arma::zeros(N_meas, 1);
    ekf_preFit = arma::zeros(N_meas, 1);
    ekf_postFit = arma::zeros(N_meas, 1);
    ekf_S = arma::zeros(N_meas, N_meas);
    ekf_K = arma::zeros(N_st, N_meas);
    rho = arma::zeros(N_ch, 1);
    rho_squared = arma::zeros(N_ch, 1);
    u_x = arma::zeros(N_ch, 1);
    u_y = arma::zeros(N_ch, 1);
    u_z = arma::zeros(N_ch, 1);
    vtl_feedback = arma::zeros(N_ch, 1);
    epochs_to_recover = num_recovery_epochs;

    // initialize transition matrix
    ekf_F = arma::eye(N_st, N_st);
    ekf_F_t = arma::eye(N_st, N_st);

    // initialize process noise covariance
    ekf_Q.zeros(N_st, N_st);

    // initialize measurement noise covariance matrix
    ekf_R = arma::zeros(N_meas, N_meas);
}

bool Vtl_Core::vtl_work(const Vtl_Data &rtklib_data)
{
    // indexants for available satellites
    aidx = arma::find(rtklib_data.active_ch == 1);
    ext_aidx = arma::join_cols(aidx, aidx + N_ch);

    // timming
    dt_s = rtklib_data.dt_s;
    // recover from observables jump when PVT fails
    if (dt_s > 2)
        {
            epochs_to_recover = num_recovery_epochs;
        }

    if (epochs_to_recover == 0)
        {
            //*************************
            // EKF PREDICTION
            //*************************
            update_process(dt_s);  // irregular vtl epoch intervals
            ekf_X = ekf_F * ekf_X;
            ekf_P = ekf_F * ekf_P * ekf_F_t + ekf_Q;


            //*************************
            // EKF MEASUREMENT
            //*************************
            compute_prefit(rtklib_data);


            //*************************
            // EKF UPDATE
            //*************************
            const arma::uvec newidx = arma::find(rtklib_data.new_ch == 1);
            if ((newidx.n_elem > 0) || (++N_meascov_updt_cnt == N_meascov_updt))
                {
                    update_meas_cov(rtklib_data, aidx, true);
                    N_meascov_updt_cnt = 0;
                }

            update_jacobian(rtklib_data);

            const arma::mat J_sub = ekf_J(ext_aidx, N_st_idx);
            const arma::mat R_sub = ekf_R(ext_aidx, ext_aidx);
            ekf_P_J_t = ekf_P * J_sub.t();
            ekf_S = J_sub * ekf_P_J_t + R_sub;
            ekf_K = arma::solve(ekf_S, ekf_P_J_t.t()).t();
            ekf_P_aux = ekf_eye - ekf_K * J_sub;
            ekf_P = ekf_P_aux * ekf_P * ekf_P_aux.t() + ekf_K * R_sub * ekf_K.t();
            ekf_X_updt = ekf_K * ekf_preFit(ext_aidx);
            ekf_X += ekf_X_updt;


            // post fits/residuals
            ekf_postFit = ekf_preFit - (ekf_J * ekf_X_updt);
            // updated computed measurement
            ekf_updt_comp_Z = ekf_obs_Z - ekf_postFit;


            //*************************
            // VTL LOOP CLOSURE
            //*************************
            send_vtl_feedback(rtklib_data);
        }
    else
        {
            vtl_reset(rtklib_data);
            epochs_to_recover--;
        }


    // log VTL data
    if (dump_enabled)
        {
            saveVTLdata(rtklib_data);
        }

    epoch++;
    return true;
}

void Vtl_Core::vtl_reset(const Vtl_Data &rtk_data)
{
    // initialize with rx position from rtklib
    ekf_X(i_px) = rtk_data.rx_p(0);
    ekf_X(i_vx) = rtk_data.rx_v(0);
    ekf_X(i_py) = rtk_data.rx_p(1);
    ekf_X(i_vy) = rtk_data.rx_v(1);
    ekf_X(i_pz) = rtk_data.rx_p(2);
    ekf_X(i_vz) = rtk_data.rx_v(2);
    ekf_X(i_clkb_G) = rtk_data.rx_clk(0);
    if (N_gal_ch > 0)
        {
            ekf_X(i_clkb_E) = rtk_data.rx_clk(1);
        }
    ekf_X(i_clkd) = rtk_data.rx_clk(2);

    dt_s = default_vtl_dt;

    // initialize state covariance matrix
    ekf_P = arma::zeros(N_st, N_st);
    ekf_P_aux = arma::zeros(N_st, N_st);
    ekf_eye = arma::eye(size(ekf_P));
    ekf_P(i_px, i_px) = vtl_init_pos_ecef_var_m2;
    ekf_P(i_py, i_py) = vtl_init_pos_ecef_var_m2;
    ekf_P(i_pz, i_pz) = vtl_init_pos_ecef_var_m2;
    ekf_P(i_vx, i_vx) = vtl_init_vel_ecef_var_m2s2;
    ekf_P(i_vy, i_vy) = vtl_init_vel_ecef_var_m2s2;
    ekf_P(i_vz, i_vz) = vtl_init_vel_ecef_var_m2s2;
    ekf_P(i_clkb_G, i_clkb_G) = vtl_init_clk_b_var_m2;
    if (N_gal_ch > 0)
        {
            ekf_P(i_clkb_E, i_clkb_E) = vtl_init_clk_b_var_m2;
        }
    ekf_P(i_clkd, i_clkd) = vtl_init_clk_d_var_m2s2;

    // initialize jacobian matrix
    ekf_J = arma::zeros(N_meas, N_st);
    ekf_P_J_t = arma::zeros(N_st, N_meas);
    // derivative of pseudorange w.r.t. clk bias - GPS/GAL
    ekf_J(arma::span(0, N_gps_ch - 1), i_clkb_G).ones();
    if (N_gal_ch > 0)
        {
            ekf_J(arma::span(N_gps_ch, N_ch - 1), i_clkb_E).ones();
        }
    // derivative of pseudorange rate w.r.t. clk drift
    ekf_J(arma::span(N_ch, 2 * N_ch - 1), i_clkd).ones();


    // initialize measurement noise covariance
    update_meas_cov(rtk_data, aidx, false);
    N_meascov_updt_cnt = 0;
}

void Vtl_Core::update_process(double dt_s)
{
    // update transition matrix
    const double T = dt_s;
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    double p_v = vtl_kinematic ? T : 0;

    ekf_F(i_px, i_vx) = p_v;
    ekf_F(i_py, i_vy) = p_v;
    ekf_F(i_pz, i_vz) = p_v;
    ekf_F(i_clkb_G, i_clkd) = T;
    if (N_gal_ch > 0)
        {
            ekf_F(i_clkb_E, i_clkd) = T;
        }
    ekf_F_t = ekf_F.t();


    // update process noise matrix
    const double q_pp = vtl_sys_acc_noise_var_m2s4 * T4 / 4;
    const double q_pv = vtl_sys_acc_noise_var_m2s4 * T3 / 2;
    const double q_vv = vtl_sys_acc_noise_var_m2s4 * T2;

    // PVA
    arma::mat Q_PV(2, 2, arma::fill::zeros);
    Q_PV(i_px, i_px) = q_pp;
    Q_PV(i_px, i_vx) = q_pv;
    Q_PV(i_vx, i_px) = q_pv;
    Q_PV(i_vx, i_vx) = q_vv;
    // 3 motion axes (x, y, z)
    ekf_Q.submat(i_px, i_px, i_vx, i_vx) = vtl_kinematic ? Q_PV : arma::zeros(2, 2);
    ekf_Q.submat(i_py, i_py, i_vy, i_vy) = vtl_kinematic ? Q_PV : arma::zeros(2, 2);
    ekf_Q.submat(i_pz, i_pz, i_vz, i_vz) = vtl_kinematic ? Q_PV : arma::zeros(2, 2);

    // Clock
    const double clk_b_noise_var_m2 = S_f + vtl_sys_clk_b_noise_var_m2;
    const double clk_d_noise_var_m2s2 = S_g + vtl_sys_clk_d_noise_var_m2s2;
    const double q_bb = clk_b_noise_var_m2 * T2 + clk_d_noise_var_m2s2 * T4 / 4;
    const double q_bd = clk_d_noise_var_m2s2 * T3 / 2;
    const double q_dd = clk_d_noise_var_m2s2 * T2;

    ekf_Q(i_clkb_G, i_clkb_G) = q_bb;
    ekf_Q(i_clkb_G, i_clkd) = q_bd;
    ekf_Q(i_clkd, i_clkb_G) = q_bd;
    ekf_Q(i_clkd, i_clkd) = q_dd;
    if (N_gal_ch > 0)
        {
            ekf_Q(i_clkb_E, i_clkb_E) = q_bb;
            ekf_Q(i_clkb_E, i_clkb_G) = q_bb;
            ekf_Q(i_clkb_G, i_clkb_E) = q_bb;
            ekf_Q(i_clkb_E, i_clkd) = q_bd;
            ekf_Q(i_clkd, i_clkb_E) = q_bd;
        }
}

void Vtl_Core::compute_prefit(const Vtl_Data &rtk_data)
{
    // observed measurements
    ekf_obs_Z(aidx) = rtk_data.obs_pr(aidx);          // pseudorange
    ekf_obs_Z(aidx + N_ch) = rtk_data.obs_prr(aidx);  // pseudorange rate

    // computed measurements
    // Geometric distance
    const arma::vec dx = rtk_data.sv_p(aidx, arma::uvec{0}) - ekf_X(i_px);  // Delta X
    const arma::vec dy = rtk_data.sv_p(aidx, arma::uvec{1}) - ekf_X(i_py);  // Delta Y
    const arma::vec dz = rtk_data.sv_p(aidx, arma::uvec{2}) - ekf_X(i_pz);  // Delta Z
    rho_squared(aidx) = arma::square(dx) + arma::square(dy) + arma::square(dz);
    rho(aidx) = arma::sqrt(rho_squared(aidx));
    // LOS unit vector
    u_x(aidx) = dx / rho(aidx);
    u_y(aidx) = dy / rho(aidx);
    u_z(aidx) = dz / rho(aidx);

    // computed pseudorange
    arma::vec clk_bias(aidx.n_elem);
    arma::uvec gps_sel = arma::find(aidx < N_gps_ch);
    arma::uvec gal_sel = arma::find(aidx >= N_gps_ch);
    clk_bias(gps_sel).fill(ekf_X(i_clkb_G));
    if (N_gal_ch > 0)
        {
            clk_bias(gal_sel).fill(ekf_X(i_clkb_E));
        }
    arma::vec sagnac_pr = (rtk_data.sv_p(aidx, arma::uvec{0}) * ekf_X(i_py) - rtk_data.sv_p(aidx, arma::uvec{1}) * ekf_X(i_px)) * GNSS_OMEGA_EARTH_DOT / SPEED_OF_LIGHT_M_S;
    ekf_comp_Z(aidx) = rho(aidx) + clk_bias - rtk_data.sv_clk(aidx, arma::uvec{0}) + rtk_data.tropo_bias(aidx) + rtk_data.iono_bias(aidx) + rtk_data.code_bias(aidx) + sagnac_pr;


    // computed pseudorange rate
    const arma::vec vx = rtk_data.sv_v(aidx, arma::uvec{0}) - ekf_X(i_vx);  // Velocity X
    const arma::vec vy = rtk_data.sv_v(aidx, arma::uvec{1}) - ekf_X(i_vy);  // Velocity Y
    const arma::vec vz = rtk_data.sv_v(aidx, arma::uvec{2}) - ekf_X(i_vz);  // Velocity Z
    arma::vec sagnac_prr = (rtk_data.sv_v(aidx, arma::uvec{1}) * ekf_X(i_px) + rtk_data.sv_p(aidx, arma::uvec{1}) * ekf_X(i_vx) -
                               rtk_data.sv_v(aidx, arma::uvec{0}) * ekf_X(i_py) - rtk_data.sv_p(aidx, arma::uvec{0}) * ekf_X(i_vy)) *
                           GNSS_OMEGA_EARTH_DOT / SPEED_OF_LIGHT_M_S;
    ekf_comp_Z(aidx + N_ch) = (vx % u_x(aidx)) + (vy % u_y(aidx)) + (vz % u_z(aidx)) +
                              ekf_X(i_clkd) - rtk_data.sv_clk(aidx, arma::uvec{1}) +
                              sagnac_prr;


    // measurement innovation (observed minus computed)
    ekf_preFit(ext_aidx) = ekf_obs_Z(ext_aidx) - ekf_comp_Z(ext_aidx);
}

void Vtl_Core::update_jacobian(const Vtl_Data &rtk_data)
{
    // Pre-cache unit vector slices
    const arma::colvec ux = u_x(aidx);
    const arma::colvec uy = u_y(aidx);
    const arma::colvec uz = u_z(aidx);
    const arma::colvec rho_v = rho(aidx);
    const arma::colvec rho_squared_v = rho_squared(aidx);

    // derivative of pseudorange w.r.t. receiver position
    ekf_J(aidx, arma::uvec{i_px}) = -ux;
    ekf_J(aidx, arma::uvec{i_py}) = -uy;
    ekf_J(aidx, arma::uvec{i_pz}) = -uz;
    // derivative of pseudorange rate w.r.t. receiver position
    const arma::colvec aux = (rtk_data.sv_v(aidx, arma::uvec{0}) - ekf_X(i_vx)) % (rtk_data.sv_p(aidx, arma::uvec{0}) - ekf_X(i_px)) +
                             (rtk_data.sv_v(aidx, arma::uvec{1}) - ekf_X(i_vy)) % (rtk_data.sv_p(aidx, arma::uvec{1}) - ekf_X(i_py)) +
                             (rtk_data.sv_v(aidx, arma::uvec{2}) - ekf_X(i_vz)) % (rtk_data.sv_p(aidx, arma::uvec{2}) - ekf_X(i_pz));
    ekf_J(aidx + N_ch, arma::uvec{i_px}) = (-rho_v % (rtk_data.sv_v(aidx, arma::uvec{0}) - ekf_X(i_vx)) + ux % aux) / rho_squared_v;
    ekf_J(aidx + N_ch, arma::uvec{i_py}) = (-rho_v % (rtk_data.sv_v(aidx, arma::uvec{1}) - ekf_X(i_vy)) + uy % aux) / rho_squared_v;
    ekf_J(aidx + N_ch, arma::uvec{i_pz}) = (-rho_v % (rtk_data.sv_v(aidx, arma::uvec{2}) - ekf_X(i_vz)) + uz % aux) / rho_squared_v;
    // derivative of pseudorange rate w.r.t. receiver velocity
    ekf_J(aidx + N_ch, arma::uvec{i_vx}) = -ux;
    ekf_J(aidx + N_ch, arma::uvec{i_vy}) = -uy;
    ekf_J(aidx + N_ch, arma::uvec{i_vz}) = -uz;
}

void Vtl_Core::update_meas_cov(const Vtl_Data &rtk_data, const arma::uvec &meas_idx, bool preFit_available)
{
    arma::vec ss_score_norm;
    arma::vec inv_Sin2elev;
    double meas_cov;

    if (preFit_available)
        {
            double max_cn0 = rtk_data.CN0_dB_hz.max();
            arma::vec w = rtk_data.CN0_dB_hz(aidx) / max_cn0;
            const arma::vec ss_score = ss_downdate_fast(w);
            ss_score_norm = ss_score * 10;
        }
    else
        {
            arma::vec sin_elev = arma::sin(rtk_data.sv_elev(meas_idx));
            inv_Sin2elev = 1.0 / arma::square(sin_elev);
        }

    for (arma::uword i = 0; i < meas_idx.n_elem; i++)
        {
            meas_cov = preFit_available ? (1 / ss_score_norm(i)) : inv_Sin2elev(i);
            ekf_R(meas_idx(i), meas_idx(i)) = vtl_meas_prange_var_m2 * meas_cov;
            ekf_R(meas_idx(i) + N_ch, meas_idx(i) + N_ch) = vtl_meas_prange_rate_var_m2s2 * meas_cov;
        }
}

void Vtl_Core::send_vtl_feedback(const Vtl_Data &rtk_data)
{
    trk_cmd_outs.clear();

    // unavailable signals go back to traditional tracking
    const arma::uvec oldidx = arma::find(rtk_data.old_ch == 1);

    size_t ratio = (rtk_data.ionoopt == IONOOPT_IFLC ? 2 : 1);
    trk_cmd_outs.reserve((aidx.n_elem + oldidx.n_elem) * ratio);

    vtl_feedback.zeros();

    TrackingCmd trk_cmd;

    double carrier_lambda = 0;
    uint32_t code_freq = 0;
    double range_factor = 0;
    for (arma::uword i = 0; i < aidx.n_elem; ++i)
        {
            if (rtk_data.band(aidx(i)))  // L5/E5
                {
                    carrier_lambda = Lambda_GPS_L5;
                    code_freq = L5E5_CODE_FREQ;
                    range_factor = RANGE_TO_FREQ_L5E5_FACTOR;
                }
            else  // L1/E1
                {
                    carrier_lambda = Lambda_GPS_L1;
                    code_freq = L1E1_CODE_FREQ;
                    range_factor = RANGE_TO_FREQ_L1E1_FACTOR;
                }

            // channel info
            trk_cmd.channel_id = aidx(i);
            trk_cmd.prn_id = rtk_data.sv_id(aidx(i));
            trk_cmd.ch_sample_counter = rtk_data.ch_sample_counter(aidx(i));
            // PLL
            trk_cmd.enable_pll_vtl_feedack = false;
            trk_cmd.pll_vtl_freq_hz = -ekf_updt_comp_Z(aidx(i) + N_ch) / carrier_lambda;  // pseudorange rate
            trk_cmd.carrier_freq_rate_hz_s = 0;                                           // pseudorange acceleration
            // DLL
            trk_cmd.enable_dll_vtl_feedack = rtk_data.loop_closure(aidx(i));                       // VDLL only
            trk_cmd.dll_vtl_freq_hz = code_freq - ekf_updt_comp_Z(aidx(i) + N_ch) * range_factor;  // pseudorange rate

            trk_cmd_outs.push_back(trk_cmd);

            // when iono-free linear combination is used VTL should control tracking channels of both frequencies
            // because both observations are being used. That's not the case when other ionosphere option is used
            if (rtk_data.ionoopt == IONOOPT_IFLC)
                {
                    // channel info
                    trk_cmd.channel_id = rtk_data.rx_ch2(aidx(i));
                    trk_cmd.ch_sample_counter = rtk_data.ch2_sample_counter(aidx(i));
                    // PLL
                    trk_cmd.pll_vtl_freq_hz = -ekf_updt_comp_Z(aidx(i) + N_ch) / Lambda_GPS_L5;  // pseudorange rate
                    trk_cmd.carrier_freq_rate_hz_s = 0;                                          // pseudorange acceleration
                    // DLL
                    trk_cmd.dll_vtl_freq_hz = L5E5_CODE_FREQ - ekf_updt_comp_Z(aidx(i) + N_ch) * RANGE_TO_FREQ_L5E5_FACTOR;  // pseudorange rate

                    trk_cmd_outs.push_back(trk_cmd);
                }
            vtl_feedback(aidx(i)) = trk_cmd.dll_vtl_freq_hz;  // log
        }
    ekf_updt_comp_Z_last = ekf_updt_comp_Z;  // store current computed measurement

    for (arma::uword i = 0; i < oldidx.n_elem; ++i)
        {
            // channel info
            trk_cmd.channel_id = oldidx(i);
            trk_cmd.prn_id = rtk_data.sv_id(oldidx(i));
            trk_cmd.ch_sample_counter = 0;
            // PLL
            trk_cmd.enable_pll_vtl_feedack = false;
            trk_cmd.pll_vtl_freq_hz = 0;
            trk_cmd.carrier_freq_rate_hz_s = 0;
            // DLL
            trk_cmd.enable_dll_vtl_feedack = false;
            trk_cmd.dll_vtl_freq_hz = 0;
            trk_cmd_outs.push_back(trk_cmd);

            if (rtk_data.ionoopt == IONOOPT_IFLC)
                {
                    // channel info
                    trk_cmd.channel_id = rtk_data.rx_ch2(oldidx(i));
                    trk_cmd_outs.push_back(trk_cmd);
                }
        }
}

const arma::colvec &Vtl_Core::get_rx_pvt() const
{
    return ekf_X;
}

const std::vector<TrackingCmd> &Vtl_Core::get_trk_cmd_outs() const
{
    return trk_cmd_outs;
}

const arma::vec Vtl_Core::ss_downdate_fast(const arma::vec &w)
{
    arma::uvec idx = arma::regspace<arma::uvec>(0, aidx.n_elem - 1);  // initial indices into aidx
    // Build G matrix directly from LOS vectors
    arma::mat G(aidx.n_elem, 3);
    G.col(0) = u_x.elem(aidx);
    G.col(1) = u_y.elem(aidx);
    G.col(2) = u_z.elem(aidx);

    // Weighted normal equations: C = inv(G^t W G)
    const arma::mat W = arma::diagmat(w);
    const arma::mat WG = W * G;
    const arma::mat GTWG = G.t() * WG;
    const arma::mat C = arma::pinv(GTWG);

    // Compute S = C * G^t * W
    // Weighed Least Squares
    const arma::mat S = C * G.t() * W;

    // Compute P diagonal: only need diagonal of W - WGS
    // P acts as the residuals of each satellite contribution to the solution
    const arma::mat P_diag_vec = W.diag() - arma::diagvec(WG * S);

    // Compute influence score: sum(S^2) / P_diag
    arma::vec influence = arma::sum(arma::square(S), 0).t();
    // normalization - to mix the weight given to each satellite and their geometric value
    arma::vec score = influence / P_diag_vec;

    // Handle NaNs or infs (e.g., division by zero)
    score.elem(arma::find_nonfinite(score)).zeros();

    return score;
}

void Vtl_Core::saveVTLdata(const Vtl_Data &rtk_data)
{
    try
        {
            double tmp_double;
            uint32_t tmp_uint32;
            // epoch
            tmp_uint32 = epoch;
            dump_file.write(reinterpret_cast<char *>(&tmp_uint32), sizeof(uint32_t));
            // rx time
            tmp_double = rtk_data.rx_time;
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            // dt
            tmp_double = rtk_data.dt_s;
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            // ekf rx p
            tmp_double = ekf_X(i_px);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            tmp_double = ekf_X(i_py);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            tmp_double = ekf_X(i_pz);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            // ekf rx v
            tmp_double = ekf_X(i_vx);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            tmp_double = ekf_X(i_vy);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            tmp_double = ekf_X(i_vz);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            // ekf rx b gps
            tmp_double = ekf_X(i_clkb_G);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            if (N_gal_ch > 0)
                {
                    // ekf rx b gal
                    tmp_double = ekf_X(i_clkb_E);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            // ekf rx d
            tmp_double = ekf_X(i_clkd);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            // rtk rx p
            tmp_double = rtk_data.rx_p(0);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            tmp_double = rtk_data.rx_p(1);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            tmp_double = rtk_data.rx_p(2);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            // rtk rx v
            tmp_double = rtk_data.rx_v(0);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            tmp_double = rtk_data.rx_v(1);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            tmp_double = rtk_data.rx_v(2);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            // rtk rx b gps
            tmp_double = rtk_data.rx_clk(0);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            // rtk rx b gal
            tmp_double = rtk_data.rx_clk(1);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
            // rtk rx d
            tmp_double = rtk_data.rx_clk(2);
            dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));

            // active channels
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = rtk_data.active_ch(i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }

            // ekf pre-fit
            for (int i = 0; i < 2 * N_ch; i++)
                {
                    tmp_double = ekf_preFit(i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            // ekf post-fit
            for (int i = 0; i < 2 * N_ch; i++)
                {
                    tmp_double = ekf_postFit(i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }

            // ekf measurement covariance
            for (int i = 0; i < 2 * N_ch; i++)
                {
                    tmp_double = ekf_R(i, i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }

            // ekf process noise
            for (int i = 0; i < N_st; i++)
                {
                    tmp_double = ekf_Q(i, i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }

            // rtk code pseudorange
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = rtk_data.obs_pr(i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            // rtk code pseudorange rate
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = rtk_data.obs_prr(i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            // vtl code pseudorange
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = ekf_comp_Z(i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            // vtl code pseudorange rate
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = ekf_comp_Z(i + N_ch);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            // VTL code freq
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = vtl_feedback(i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }

            // SV position X
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = rtk_data.sv_p(i, 0);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            // SV position Y
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = rtk_data.sv_p(i, 1);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            // SV position Z
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = rtk_data.sv_p(i, 2);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }

            // SV clock bias
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = rtk_data.sv_clk(i, 0);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            // SV clock drift
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = rtk_data.sv_clk(i, 1);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }

            // tropo bias
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = rtk_data.tropo_bias(i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            // iono bias
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = rtk_data.iono_bias(i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
            // code bias
            for (int i = 0; i < N_ch; i++)
                {
                    tmp_double = rtk_data.code_bias(i);
                    dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                }
        }
    catch (const std::ofstream::failure &e)
        {
            LOG(WARNING) << "Exception writing VTL dump file " << e.what();
        }
}
