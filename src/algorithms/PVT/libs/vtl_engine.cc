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

#include "vtl_engine.h"

Vtl_Engine::Vtl_Engine()
{
}

Vtl_Engine::~Vtl_Engine()
{
}

bool Vtl_Engine::vtl_loop(Vtl_Data new_data)
{
    //TODO: Implement main VTL loop here
//     // ################## Kalman filter initialization ######################################

//     // covariances (static)
//     kf_P_x_ini = arma::zeros(8, 8);
//     kf_x_pri   = arma::zeros(8, 1);
//     kf_R = arma::zeros(2*n_sats, 2*n_sats);
//     kf_dt=1e-3;
//     kf_Q = arma::zeros(8, 8);

//     kf_F = arma::zeros(8, 8);
//     kf_F(0, 0) = 1.0; kf_F(0, 3) = kf_dt;
//     kf_F(1, 1) = 1.0; kf_F(1, 4) = kf_dt;
//     kf_F(2, 2) = 1.0; kf_F(2, 5) = kf_dt;
//     kf_F(3, 3) = 1.0; 
//     kf_F(4, 4) = 1.0; 
//     kf_F(5, 5) = 1.0;
//     kf_F(6, 6) = 1.0; kf_F(6, 7) = kf_dt;
//     kf_F(7, 7) = 1.0;

//     kf_H = arma::zeros(8, 2*n_sats);
//     kf_x = arma::zeros(8, 1);
//     kf_y = arma::zeros(2*n_sats, 1);
//     kf_P_y = arma::zeros(2*n_sats, 2*n_sats);

//     // ################## Kalman Tracking ######################################
//     // receiver solution from rtklib_solver
//     kf_x(0)=new_vtl_data.rx_p(0);
//     kf_x(1)=new_vtl_data.rx_p(1);
//     kf_x(2)=new_vtl_data.rx_p(2);
//     kf_x(3)=new_vtl_data.rx_v(0);
//     kf_x(4)=new_vtl_data.rx_v(1);
//     kf_x(5)=new_vtl_data.rx_v(2);
//     kf_x(6)=new_vtl_data.rx_dts(0);
//     kf_x(7)=new_vtl_data.rx_dts(1);

//     // Kalman state prediction (time update)
//     kf_x_pri = kf_F * kf_x;                        // state prediction
//     //kf_P_x_pri = kf_F * kf_P_x * kf_F.t() + kf_Q;  // state error covariance prediction

//     //from error state variables to variables
//     //x_u=x_u0+kf_x_pri(0);
//     //y_u=y_u0+kf_x_pri(1);
//     //z_u=z_u0+kf_x_pri(2);
//     //xDot_u=xDot_u0+kf_x_pri(3);
//     //yDot_u=yDot_u0+kf_x_pri(4);
//     //zDot_u=zDot_u0+kf_x_pri(5);
//     //cdeltat_u=cdeltat_u0+kf_x_pri(6);
//     //cdeltatDot_u=cdeltatDot_u+kf_x_pri(7);
//     //from state variables definition
//     x_u=kf_x_pri(0);
//     y_u=kf_x_pri(1);
//     z_u=kf_x_pri(2);
//     xDot_u=kf_x_pri(3);
//     yDot_u=kf_x_pri(4);
//     zDot_u=kf_x_pri(5);
//     cdeltat_u=kf_x_pri(6);
//     cdeltatDot_u=kf_x_pri(7);
//     for (int32_t i = 0; i < n_sats; n++) //neccesary quantities
//     {
//         d(i)=sqrt(square(new_vtl_data.sat_p(i, 0)-x_u)+square(new_vtl_data.sat_p(i, 1)-y_u)+square(new_vtl_data.sat_p(i, 2)-z_u));
//         //compute pseudorange estimation 
//         rho_pri(i)=d(i)+cdeltat_u;
//         //compute LOS sat-receiver vector components 
//         a_x(i)=-(new_vtl_data.sat_p(i, 0)-x_u)/d(i);
//         a_y(i)=-(new_vtl_data.sat_p(i, 1)-y_u)/d(i);;
//         a_z(i)=-(new_vtl_data.sat_p(i, 2)-z_u)/d(i);;
//         //compute pseudorange rate estimation
//         rhoDot_pri(i)=(new_vtl_data.sat_v(i, 0)-xDot_u)*a_x(i)+(new_vtl_data.sat_v(i, 1)-yDot_u)*a_y(i)+(new_vtl_data.sat_v(i, 2)-zDot_u)*a_z(i)+cdeltatDot_u;
//     }

//     kf_H = arma::zeros(8, 2*n_sats);

//     for (int32_t i = 0; i < n_sats; n++) // Measurement matrix H assembling
//     {
//         // It has 8 columns (8 states) and 2*NSat rows (NSat psudorange error;NSat pseudo range rate error) 
//         kf_H(i, 0) = a_x(i); kf_H(i, 1) = a_y(i); kf_H(i, 2) = a_z(i); kf_H(i, 6) = 1.0;
//         kf_H(i+n_sats, 3) = a_x(i); kf_H(i+n_sats, 4) = a_y(i); kf_H(i+n_sats, 5) = a_z(i); kf_H(i+n_sats, 7) = 1.0;
//     }

//     // Kalman estimation (measurement update)
//     for (int32_t i = 0; i < n_sats; n++) // Measurement vector
//     {
//         kf_y(i) = delta_rho(i); // i-Satellite 
//         kf_y(i+n_sats) = delta_rhoDot(i);  // i-Satellite   
//     }

//     for (int32_t i = 0; i < n_sats; n++) // Measurement error Covariance Matrix R assembling
//     {
//         // It is diagonal 2*NSatellite x 2*NSatellite (NSat psudorange error;NSat pseudo range rate error) 
//         kf_R(i, i) = 1.0;
//         kf_R(i+n_sats, i+n_sats) = 1.0;
//     }

//     // Kalman filter update step
//    // kf_P_y = kf_H * kf_P_x_pri * kf_H.t() + kf_R;                       // innovation covariance matrix (S)
//    // kf_K = (kf_P_x_pri * kf_H.t()) * arma::inv(kf_P_y);                 // Kalman gain
        
//     for (int32_t i = 0; i < n_sats; n++) //Error measurement vector
//     {
//        // kf_delta_y(i)=rho(i)+delta_rho(i)-rho_pri(i); // pseudorange error
//        // kf_delta_y(i+n_sats)=rhoDot(i)+delta_F*(-lambdaC)-rhoDot_pri(i); // pseudorange rate error
//     }

//    // kf_delta_x = kf_K * kf_delta_y;                                     // updated error state estimation
//    // kf_P_x = (arma::eye(size(kf_P_x_pri)) - kf_K * kf_H) * kf_P_x_pri;  // update state estimation error covariance matrix

//   //  kf_x = kf_x_pri+kf_delta_x;                                         // compute PVT  from priori and error estimation (neccesary?)


//     // ################## Geometric Transformation ######################################

//     for (int32_t i = 0; i < n_sats; n++) //neccesary quantities at posteriori
//     {
//         //compute pseudorange posteriori estimation 
//        // rho_est(i)=;
//         //compute LOS sat-receiver vector components posteriori 
//        // a_x(i)=;
//        // a_y(i)=;
//        // a_z(i)=;
//         //compute pseudorange rate posteriori estimation
//        // rhoDot_est(i)=;
//     }

//     kf_H = arma::zeros(8, 2*n_sats);

//     for (int32_t i = 0; i < n_sats; n++) // Measurement matrix H posteriori assembling
//     {
//         // It has 8 columns (8 states) and 2*NSat rows (NSat psudorange error;NSat pseudo range rate error) 
//         kf_H(i, 0) = a_x(i); kf_H(i, 1) = a_y(i); kf_H(i, 2) = a_z(i); kf_H(i, 6) = 1.0;
//         kf_H(i+n_sats, 3) = a_x(i); kf_H(i+n_sats, 4) = a_y(i); kf_H(i+n_sats, 5) = a_z(i); kf_H(i+n_sats, 7) = 1.0;
//     }

//     //Re-calculate error measurement vector with the most recent data available
//     //kf_delta_y=kf_H*kf_delta_x 
//     //Filtered pseudorange error measurement (in m):
//     //delta_rho_filt=;
//     //Filtered Doppler error measurement (in Hz):
//     //delta_doppler_filt=;

    //TODO: Fill the tracking commands outputs
    // Notice: keep the same satellite order as in the Vtl_Data matrices
    // sample code
    TrackingCmd trk_cmd;
    trk_cmd.carrier_freq_hz = 0;
    trk_cmd.carrier_freq_rate_hz_s = 0;
    trk_cmd.code_freq_chips = 0;
    trk_cmd.enable_carrier_nco_cmd = true;
    trk_cmd.enable_code_nco_cmd = true;
    trk_cmd.sample_counter = new_data.sample_counter;
    trk_cmd_outs.push_back(trk_cmd);
    return true;
}

void Vtl_Engine::reset()
{
    //TODO
}

void Vtl_Engine::debug_print()
{
    //TODO
}

void Vtl_Engine::configure(Vtl_Conf config_)
{
    config = config_;
    //TODO: initialize internal variables
}
