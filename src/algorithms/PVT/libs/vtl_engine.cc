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
    using arma::as_scalar;
    using arma::dot;
//     // ################## Kalman filter initialization ######################################

//     // covariances (static)
    kf_P_x_ini  = arma::zeros(8, 8); //TODO: use a real value.
    kf_x        = arma::zeros(8, 1);
    kf_R        = arma::zeros(2*new_data.sat_number, 2*new_data.sat_number);
    double kf_dt=1e-3;
    kf_Q = arma::zeros(8, 8);

    kf_F = arma::zeros(8, 8);
    kf_F(0, 0) = 1.0; kf_F(0, 3) = kf_dt;
    kf_F(1, 1) = 1.0; kf_F(1, 4) = kf_dt;
    kf_F(2, 2) = 1.0; kf_F(2, 5) = kf_dt;
    kf_F(3, 3) = 1.0; 
    kf_F(4, 4) = 1.0; 
    kf_F(5, 5) = 1.0;
    kf_F(6, 6) = 1.0; kf_F(6, 7) = kf_dt;
    kf_F(7, 7) = 1.0;

    kf_H = arma::zeros(8, 2*new_data.sat_number);
    kf_x = arma::zeros(8, 1);
    kf_y = arma::zeros(2*new_data.sat_number, 1);
    kf_S = arma::zeros(2*new_data.sat_number, 2*new_data.sat_number); // kf_P_y innovation covariance matrix

//     // ################## Kalman Tracking ######################################
//     // receiver solution from rtklib_solver
    kf_x(0)=new_data.rx_p(0);
    kf_x(1)=new_data.rx_p(1);
    kf_x(2)=new_data.rx_p(2);
    kf_x(3)=new_data.rx_v(0);
    kf_x(4)=new_data.rx_v(1);
    kf_x(5)=new_data.rx_v(2);
    kf_x(6)=new_data.rx_dts(0);
    kf_x(7)=new_data.rx_dts(1);

//     // Kalman state prediction (time update)
    kf_x = kf_F * kf_x;                        // state prediction
    kf_P_x= kf_F * kf_P_x * kf_F.t() + kf_Q;  // state error covariance prediction

//     //from error state variables to variables
//     //x_u=x_u0+kf_x_pri(0);
//     //y_u=y_u0+kf_x_pri(1);
//     //z_u=z_u0+kf_x_pri(2);
//     //xDot_u=xDot_u0+kf_x_pri(3);
//     //yDot_u=yDot_u0+kf_x_pri(4);
//     //zDot_u=zDot_u0+kf_x_pri(5);
//     //cdeltat_u=cdeltat_u0+kf_x_pri(6);
//     //cdeltatDot_u=cdeltatDot_u+kf_x_pri(7);
// From state variables definition
    x_u=kf_x(0);
    y_u=kf_x(1);
    z_u=kf_x(2);
    xDot_u=kf_x(3);
    yDot_u=kf_x(4);
    zDot_u=kf_x(5);
    cdeltat_u=kf_x(6);
    cdeltatDot_u=kf_x(7);

    d = arma::zeros(new_data.sat_number, 1);
    rho_pri = arma::zeros(new_data.sat_number, 1);
    rhoDot_pri = arma::zeros(new_data.sat_number, 1);
    a_x = arma::zeros(new_data.sat_number, 1);
    a_y = arma::zeros(new_data.sat_number, 1);
    a_z = arma::zeros(new_data.sat_number, 1);

    for (int32_t i = 0; i < new_data.sat_number; i++) //neccesary quantities
    {
        d(i)=(sqrt((new_data.sat_p(i, 0)-x_u)*(new_data.sat_p(i, 0)-x_u)+(new_data.sat_p(i, 1)-y_u)*(new_data.sat_p(i, 1)-y_u)+(new_data.sat_p(i, 2)-z_u)*(new_data.sat_p(i, 2)-z_u)));
        //compute pseudorange estimation 
        rho_pri(i)=d(i)+cdeltat_u;
        //compute LOS sat-receiver vector components 
        a_x(i)=-(new_data.sat_p(i, 0)-x_u)/d(i);
        a_y(i)=-(new_data.sat_p(i, 1)-y_u)/d(i);;
        a_z(i)=-(new_data.sat_p(i, 2)-z_u)/d(i);;
        //compute pseudorange rate estimation
        rhoDot_pri(i)=(new_data.sat_v(i, 0)-xDot_u)*a_x(i)+(new_data.sat_v(i, 1)-yDot_u)*a_y(i)+(new_data.sat_v(i, 2)-zDot_u)*a_z(i)+cdeltatDot_u;
    }

    kf_H = arma::zeros(8, 2*new_data.sat_number);

    for (int32_t i = 0; i < new_data.sat_number; i++) // Measurement matrix H assembling
    {
        // It has 8 columns (8 states) and 2*NSat rows (NSat psudorange error;NSat pseudo range rate error) 
        kf_H(i, 0) = a_x(i); kf_H(i, 1) = a_y(i); kf_H(i, 2) = a_z(i); kf_H(i, 6) = 1.0;
        kf_H(i+new_data.sat_number, 3) = a_x(i); kf_H(i+new_data.sat_number, 4) = a_y(i); kf_H(i+new_data.sat_number, 5) = a_z(i); kf_H(i+new_data.sat_number, 7) = 1.0;
    }

    // Kalman estimation (measurement update)
   for (int32_t i = 0; i < new_data.sat_number; i++) // Measurement vector
   {
        //kf_y(i) = delta_rho(i); // i-Satellite 
        kf_y(i)=new_data.pr_m(i);
        //kf_y(i+new_data.sat_number) = delta_rhoDot(i);  // i-Satellite
        kf_y(i+new_data.sat_number)=new_data.doppler_hz(i);
   }

    for (int32_t i = 0; i < new_data.sat_number; i++) // Measurement error Covariance Matrix R assembling
    {
        // It is diagonal 2*NSatellite x 2*NSatellite (NSat psudorange error;NSat pseudo range rate error) 
        kf_R(i, i) = 1.0; //TODO: use a real value.
        kf_R(i+new_data.sat_number, i+new_data.sat_number) = 1.0;
    }

    // Kalman filter update step
    kf_S = kf_H * kf_P_x* kf_H.t() + kf_R;                       // innovation covariance matrix (S)
    kf_K = (kf_P_x * kf_H.t()) * arma::inv(kf_S);                // Kalman gain
        
    //for (int32_t i = 0; i < new_data.sat_number; i++) //Error measurement vector
    //{
    //   kf_delta_y(i)=new_data.pr_m(i)+delta_rho(i)-rho_pri(i); // pseudorange error
    //   kf_delta_y(i+new_data.sat_number)=new_data.doppler_hz(i)+delta_F*(-lambdaC)-rhoDot_pri(i); // pseudorange rate error
    //}

    //kf_delta_x = kf_K * kf_delta_y;                                   // updated error state estimation
    kf_x = kf_K * (kf_y-dot(kf_H,kf_x));                                // updated error state estimation
    kf_P_x = (arma::eye(size(kf_P_x)) - kf_K * kf_H) * kf_P_x;          // update state estimation error covariance matrix

//   //  kf_x = kf_x_pri+kf_delta_x;                                         // compute PVT  from priori and error estimation (neccesary?)


//     // ################## Geometric Transformation ######################################

//     for (int32_t i = 0; i < new_data.sat_number; n++) //neccesary quantities at posteriori
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

//     kf_H = arma::zeros(8, 2*new_data.sat_number);

//     for (int32_t i = 0; i < new_data.sat_number; n++) // Measurement matrix H posteriori assembling
//     {
//         // It has 8 columns (8 states) and 2*NSat rows (NSat psudorange error;NSat pseudo range rate error) 
//         kf_H(i, 0) = a_x(i); kf_H(i, 1) = a_y(i); kf_H(i, 2) = a_z(i); kf_H(i, 6) = 1.0;
//         kf_H(i+new_data.sat_number, 3) = a_x(i); kf_H(i+new_data.sat_number, 4) = a_y(i); kf_H(i+new_data.sat_number, 5) = a_z(i); kf_H(i+new_data.sat_number, 7) = 1.0;
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
