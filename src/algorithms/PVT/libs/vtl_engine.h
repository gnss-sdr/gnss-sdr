/*!
 * \file vtl_engine.h
 * \brief Class that implements a Vector Tracking Loop (VTL) Kalman filter engine
 * \author Javier Arribas, 2022. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_VTL_ENGINE_H
#define GNSS_SDR_VTL_ENGINE_H

#include "MATH_CONSTANTS.h"
#include "trackingcmd.h"
#include "vtl_conf.h"
#include "vtl_data.h"
#include <armadillo>
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


class Vtl_Engine
{
public:
    Vtl_Engine();

    ~Vtl_Engine();

    void configure(Vtl_Conf config_);  // set config parameters

    // TODO: output functions here (output for tracking KF updates, VTL computed user PVT, etc...)
    bool vtl_loop(Vtl_Data new_data);
    void reset();        // reset all internal states
    void debug_print();  // print debug information

    std::vector<TrackingCmd> trk_cmd_outs;            // vector holding the Tracking command states updates to be sent to tracking KFs
    std::vector<double> get_position_ecef_m();        // get_position_ecef_m
    std::vector<double> get_velocity_ecef_m_s();      // get_velocity_ecef_m_s
    std::vector<double> get_accel_ecef_m_s2();        // get_accel_ecef_m_s2
    std::vector<double> get_position_var_ecef_m();    // get_position_var_ecef_m
    std::vector<double> get_velocity_var_ecef_m_s();  // get_velocity_var_ecef_m_s
    std::vector<double> get_accel_var_ecef_m_s2();    // get_accel_var_ecef_m_s2
    std::vector<double> get_geodetic_rad_m();         // get_geodetic_rad_m
    double get_user_clock_offset_s();                 // get_user_clock_offset_s;
    double get_user_clock_offset_drift_s_s();         // get_user_clock_offset_drift_s/s;

private:
    Vtl_Conf config;
    // TODO: Internal VTL persistent variables here

    // Transformation variables
    arma::colvec d;
    arma::colvec rho_pri;
    arma::colvec rhoDot_pri;
    arma::colvec rhoDot2_pri;
    arma::colvec rho_pri_filt;
    arma::colvec rhoDot_pri_filt;
    arma::colvec doppler_hz_filt;
    arma::colvec a_x;
    arma::colvec a_y;
    arma::colvec a_z;

    // Kalman filter matrices
    arma::mat kf_P_x_ini;  // initial state error covariance matrix
    // arma::mat kf_P_x;      // state error covariance matrix
    arma::mat kf_P_x_pre;  // Predicted state error covariance matrix
    arma::mat kf_P_x;
    arma::mat kf_S;  // innovation covariance matrix

    arma::mat kf_F;  // state transition matrix
    arma::mat kf_H;  // system matrix
    arma::mat kf_R;  // measurement error covariance matrix
    arma::mat kf_Q;  // system error covariance matrix

    arma::mat kf_x;      // state vector
    arma::mat kf_x_pre;  // predicted state vector
    arma::mat kf_y;      // measurement vector
    arma::mat kf_yerr;   // ERROR measurement vector
    arma::mat kf_xerr;   // ERROR state vector
    arma::mat kf_K;      // Kalman gain matrix

    // Gaussian estimator
    arma::mat kf_R_est;  // measurement error covariance


    uint32_t counter;
    int n_of_states;
    uint64_t refSampleCounter;
    double delta_t_cmd = 0;

    void kf_H_fill(arma::mat &kf_H, int sat_number, arma::colvec ax, arma::colvec ay, arma::colvec az, double kf_dt); /*  */                                                                                    // Observation Matrix constructor
    void kf_F_fill_rocket(arma::mat &kf_F, double kf_dt, arma::mat &kf_x);                                                                                                                                      // System Matrix constructor
    void kf_F_fill(arma::mat &kf_F, double kf_dt, arma::mat &kf_x);                                                                                                                                             // System Matrix constructor
    void obsv_calc(arma::mat &rho_pri, arma::mat &rhoDot_pri, arma::mat &rhoDot2_pri, arma::colvec &ax, arma::colvec &ay, arma::colvec &az, int sat_number, arma::mat sat_p, arma::mat sat_v, arma::mat kf_x);  // Observables calculation
    void kf_measurements(arma::mat &kf_yerr, int sat_number, arma::mat rho_pri, arma::mat rhoDot_pri, arma::mat rhoDot2_pri, arma::colvec pr_m, arma::colvec doppler_hz, arma::mat kf_x);
};

/** \} */
/** \} */
#endif  // GNSS_SDR_VTL_ENGINE_H
