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

#include "trackingcmd.h"
#include "vtl_conf.h"
#include "vtl_data.h"
#include "MATH_CONSTANTS.h"
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

    void configure(Vtl_Conf config_);  //set config parameters

    //TODO: output functions here (output for tracking KF updates, VTL computed user PVT, etc...)
    bool vtl_loop(Vtl_Data new_data);
    void reset();        // reset all internal states
    void debug_print();  // print debug information

    std::vector<TrackingCmd> trk_cmd_outs;  // vector holding the Tracking command states updates to be sent to tracking KFs

private:
    Vtl_Conf config;
    //TODO: Internal VTL persistent variables here
   
    //State variables
    double x_u;
    double y_u;
    double z_u;
    double xDot_u;
    double yDot_u;
    double zDot_u;
    double cdeltat_u;
    double cdeltatDot_u;
    
    // Transformation variables
    arma::colvec d;
    arma::colvec rho_pri;
    arma::colvec rhoDot_pri;
    arma::colvec a_x;
    arma::colvec a_y;
    arma::colvec a_z;
    
    // Kalman filter matrices
    arma::mat kf_P_x_ini;  // initial state error covariance matrix
    arma::mat kf_P_x;      // state error covariance matrix
    arma::mat kf_P_x_pre;  // Predicted state error covariance matrix
    arma::mat kf_S;      // innovation covariance matrix

    arma::mat kf_F;  // state transition matrix
    arma::mat kf_H;  // system matrix
    arma::mat kf_R;  // measurement error covariance matrix
    arma::mat kf_Q;  // system error covariance matrix

    arma::mat kf_x;      // state vector
    arma::mat kf_x_pre;  // predicted state vector
    arma::mat kf_y;      // measurement vector
    arma::mat kf_yerr;   // ERROR measurement vector
    arma::mat kf_xerr;   // ERROR state vector
    arma::mat kf_K;         // Kalman gain matrix

    // Gaussian estimator
    arma::mat kf_R_est;  // measurement error covariance
};


/** \} */
/** \} */
#endif  // GNSS_SDR_VTL_ENGINE_H
