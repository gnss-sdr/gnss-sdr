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
#include "iostream"
#include <fstream>

using namespace std;

Vtl_Engine::Vtl_Engine()
{
}

Vtl_Engine::~Vtl_Engine()
{
}

bool Vtl_Engine::vtl_loop(Vtl_Data& new_data)
{
    //TODO: Implement main VTL loop here
    using arma::as_scalar;

    // ################## Kalman filter initialization ######################################
    // covariances (static)
    kf_P_x  = arma::eye(8, 8)*1.0; //TODO: use a real value.
    kf_x    = arma::zeros(8, 1);
    kf_R    = arma::zeros(2*new_data.sat_number, 2*new_data.sat_number);
    double kf_dt=0.1;
    kf_Q = arma::eye(8, 8);

    kf_F = arma::eye(8, 8);
    kf_F(0, 3) = kf_dt;
    kf_F(1, 4) = kf_dt;
    kf_F(2, 5) = kf_dt;
    kf_F(6, 7) = kf_dt;

    kf_H = arma::zeros(2*new_data.sat_number,8);
    kf_y = arma::zeros(2*new_data.sat_number, 1);
    kf_yerr = arma::zeros(2*new_data.sat_number, 1);
    kf_xerr = arma::zeros(8, 1);
    kf_S = arma::zeros(2*new_data.sat_number, 2*new_data.sat_number); // kf_P_y innovation covariance matrix
    
    // ################## Kalman Tracking ######################################
    static uint32_t counter=0; //counter  
    counter = counter+1; //uint64_t 
    cout << "counter" << counter<<endl;
    //new_data.kf_state.print("new_data kf initial");
    if(counter<3000){ //
        // receiver solution from rtklib_solver
        kf_x(0) = new_data.rx_p(0);
        kf_x(1) = new_data.rx_p(1);
        kf_x(2) = new_data.rx_p(2);
        kf_x(3) = new_data.rx_v(0);
        kf_x(4) = new_data.rx_v(1);
        kf_x(5) = new_data.rx_v(2);
        kf_x(6) = new_data.rx_dts(0)*SPEED_OF_LIGHT_M_S; 
        kf_x(7) = new_data.rx_dts(1)*SPEED_OF_LIGHT_M_S;

        kf_x = kf_F * kf_x;                        // state prediction

    } else {
        // receiver solution from previous KF step
        kf_x(0) = new_data.kf_state[0];
        kf_x(1) = new_data.kf_state[1];
        kf_x(2) = new_data.kf_state[2];
        kf_x(3) = new_data.kf_state[3];
        kf_x(4) = new_data.kf_state[4];
        kf_x(5) = new_data.kf_state[5];
        kf_x(6) = new_data.kf_state[6]; 
        kf_x(7) = new_data.kf_state[7];   
        kf_P_x=new_data.kf_P;
    }
    // State error Covariance Matrix Q (PVT)
    // for (int32_t i = 0; i < 8; i++) 
    // {
    //     // It is diagonal 8x8 matrix 
    //     kf_Q(i, i) = 1.0;//new_data.rx_pvt_var(i); //careful, values for V and T could not be adecuate.
    // }
    // Measurement error Covariance Matrix R assembling
    for (int32_t i = 0; i < new_data.sat_number; i++) 
    {
        // It is diagonal 2*NSatellite x 2*NSatellite (NSat psudorange error;NSat pseudo range rate error) 
        kf_R(i, i) = 10.0; //TODO: fill with real values.
        kf_R(i+new_data.sat_number, i+new_data.sat_number) = 30.0;
    }

    // Kalman state prediction (time update)
    //new_data.kf_state=kf_x; 
    //kf_x = kf_F * kf_x;                        // state prediction
    kf_P_x= kf_F * kf_P_x * kf_F.t() + kf_Q;  // state error covariance prediction
    //from error state variables to variables
    // From state variables definition
    // TODO: cast to type properly
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

    rho_pri_filt= arma::zeros(new_data.sat_number, 1);
    rhoDot_pri_filt = arma::zeros(new_data.sat_number, 1);
    doppler_hz_filt = arma::zeros(new_data.sat_number, 1);

    a_x = arma::zeros(new_data.sat_number, 1);
    a_y = arma::zeros(new_data.sat_number, 1);
    a_z = arma::zeros(new_data.sat_number, 1);
    for (int32_t i = 0; i < new_data.sat_number; i++) //neccesary quantities
    {
        //d(i) is the distance sat(i) to receiver
        d(i)=(new_data.sat_p(i, 0)-x_u)*(new_data.sat_p(i, 0)-x_u);
        d(i)=d(i)+(new_data.sat_p(i, 1)-y_u)*(new_data.sat_p(i, 1)-y_u);
        d(i)=d(i)+(new_data.sat_p(i, 2)-z_u)*(new_data.sat_p(i, 2)-z_u);
        d(i)=sqrt(d(i)); 

        //compute pseudorange estimation 
        rho_pri(i)=d(i)+cdeltat_u;
        //compute LOS sat-receiver vector components 
        a_x(i)=-(new_data.sat_p(i, 0)-x_u)/d(i);
        a_y(i)=-(new_data.sat_p(i, 1)-y_u)/d(i);
        a_z(i)=-(new_data.sat_p(i, 2)-z_u)/d(i);
        new_data.sat_LOS(i,0)=a_x(i);
        new_data.sat_LOS(i,1)=a_y(i);
        new_data.sat_LOS(i,2)=a_z(i);
        //compute pseudorange rate estimation
        rhoDot_pri(i)=(new_data.sat_v(i, 0)-xDot_u)*a_x(i)+(new_data.sat_v(i, 1)-yDot_u)*a_y(i)+(new_data.sat_v(i, 2)-zDot_u)*a_z(i);
    }

    kf_H = arma::zeros(2*new_data.sat_number,8);

    for (int32_t i = 0; i < new_data.sat_number; i++) // Measurement matrix H assembling
    {
        // It has 8 columns (8 states) and 2*NSat rows (NSat psudorange error;NSat pseudo range rate error) 
        kf_H(i, 0) = a_x(i); kf_H(i, 1) = a_y(i); kf_H(i, 2) = a_z(i); kf_H(i, 6) = 1.0;
        kf_H(i+new_data.sat_number, 3) = a_x(i); kf_H(i+new_data.sat_number, 4) = a_y(i); kf_H(i+new_data.sat_number, 5) = a_z(i); kf_H(i+new_data.sat_number, 7) = 1.0;
    }
    // Kalman estimation (measurement update)
   for (int32_t i = 0; i < new_data.sat_number; i++) // Measurement vector
   {
        //kf_y(i) = new_data.pr_m(i); // i-Satellite 
        //kf_y(i+new_data.sat_number) = rhoDot_pri(i)/Lambda_GPS_L1; // i-Satellite
        kf_yerr(i)=rho_pri(i)-new_data.pr_m(i);
        kf_yerr(i+new_data.sat_number)=(new_data.doppler_hz(i)*Lambda_GPS_L1)-rhoDot_pri(i);

   }

    // Kalman filter update step
    kf_S = kf_H * kf_P_x* kf_H.t() + kf_R;                      // innovation covariance matrix (S)
    kf_K = (kf_P_x * kf_H.t()) * arma::inv(kf_S);               // Kalman gain  
    kf_xerr = kf_K * (kf_yerr);                                 // Error state estimation
    kf_x = kf_x - kf_xerr;                                      // updated state estimation (a priori + error)
    kf_P_x = (arma::eye(size(kf_P_x)) - kf_K * kf_H) * kf_P_x;  // update state estimation error covariance matrix
    new_data.kf_P=kf_P_x;
    new_data.kf_state=kf_x; //updated state estimation
 
//     // ################## Geometric Transformation ######################################

//     // x_u=kf_x(0);
//     // y_u=kf_x(1);
//     // z_u=kf_x(2);
//     // xDot_u=kf_x(3);
//     // yDot_u=kf_x(4);
//     // zDot_u=kf_x(5);
//     // cdeltat_u=kf_x(6)*SPEED_OF_LIGHT_M_S;
//     // cdeltatDot_u=kf_x(7)*SPEED_OF_LIGHT_M_S;

   for (int32_t i = 0; i < new_data.sat_number; i++) //neccesary quantities
    {
        //d(i) is the distance sat(i) to receiver
        d(i)=(new_data.sat_p(i, 0)-kf_x(0))*(new_data.sat_p(i, 0)-kf_x(0));
        d(i)=d(i)+(new_data.sat_p(i, 1)-kf_x(1))*(new_data.sat_p(i, 1)-kf_x(1));
        d(i)=d(i)+(new_data.sat_p(i, 2)-kf_x(2))*(new_data.sat_p(i, 2)-kf_x(2));
        d(i)=sqrt(d(i)); 

        //compute pseudorange estimation 
        rho_pri(i)=d(i)+kf_x(6);
        //compute LOS sat-receiver vector components 
        a_x(i)=-(new_data.sat_p(i, 0)-kf_x(0))/d(i);
        a_y(i)=-(new_data.sat_p(i, 1)-kf_x(1))/d(i);
        a_z(i)=-(new_data.sat_p(i, 2)-kf_x(2))/d(i);
        //compute pseudorange rate estimation
        rhoDot_pri(i)=(new_data.sat_v(i, 0)-kf_x(3))*a_x(i)+(new_data.sat_v(i, 1)-kf_x(4))*a_y(i)+(new_data.sat_v(i, 2)-kf_x(5))*a_z(i)+kf_x(7);
    }

    kf_H = arma::zeros(2*new_data.sat_number,8);

    for (int32_t i = 0; i < new_data.sat_number; i++) // Measurement matrix H assembling
    {
        // It has 8 columns (8 states) and 2*NSat rows (NSat psudorange error;NSat pseudo range rate error) 
        kf_H(i, 0) = a_x(i); kf_H(i, 1) = a_y(i); kf_H(i, 2) = a_z(i); kf_H(i, 6) = 1.0;
        kf_H(i+new_data.sat_number, 3) = a_x(i); kf_H(i+new_data.sat_number, 4) = a_y(i); kf_H(i+new_data.sat_number, 5) = a_z(i); kf_H(i+new_data.sat_number, 7) = 1.0;
    }

//  Re-calculate error measurement vector with the most recent data available: kf_delta_y=kf_H*kf_delta_x 
    kf_yerr=kf_H*kf_xerr;
//  Filtered pseudorange error measurement (in m) AND Filtered Doppler shift measurements (in Hz):

   for (int32_t i = 0; i < new_data.sat_number; i++) // Measurement vector
   {

        rho_pri_filt(i)=new_data.pr_m(i)+kf_yerr(i); // now filtered
        rhoDot_pri_filt(i)=(new_data.doppler_hz(i)*Lambda_GPS_L1+kf_x(7))-kf_yerr(i+new_data.sat_number); // now filtered
        // TO DO: convert rhoDot_pri to doppler shift!
        // Doppler shift defined as pseudorange rate measurement divided by the negative of carrier wavelength.
        doppler_hz_filt(i)=(rhoDot_pri_filt(i)-kf_x(7))/Lambda_GPS_L1;
   }
    
     
 	fstream dump_vtl_file;
	dump_vtl_file.open("dump_vtl_file.csv", ios::out|ios::app);
    dump_vtl_file.precision(15);
	if (!dump_vtl_file) {
		cout << "File not created!";
	}
	else {
        dump_vtl_file << "kf_xerr"<< ","<<kf_xerr(0)<< ","<<kf_xerr(1)<< ","<<kf_xerr(2)<< ","<<kf_xerr(3)<< ","<<kf_xerr(4)<< ","<<kf_xerr(5)<< ","<<kf_xerr(6)<< ","<<kf_xerr(7)<<endl;
        dump_vtl_file << "kf_state"<< ","<<new_data.kf_state(0)<< ","<<new_data.kf_state(1)<< ","<<new_data.kf_state(2)<< ","<<new_data.kf_state(3)<< ","<<new_data.kf_state(4)<< ","<<new_data.kf_state(5)<< ","<<new_data.kf_state(6)<< ","<<new_data.kf_state(7)<<endl;
		dump_vtl_file << "rtklib_state"<< ","<<new_data.rx_p(0)<< ","<< new_data.rx_p(1)<<","<< new_data.rx_p(2)<<","<< new_data.rx_v(0)<<","<< new_data.rx_v(1)<<","<< new_data.rx_v(2)<<","<< new_data.rx_dts(0)<<","<< new_data.rx_dts(1)<<endl; 
        dump_vtl_file << "filt_dopp_sat"<< ","<<doppler_hz_filt(0)<< ","<< doppler_hz_filt(1)<<","<< doppler_hz_filt(2)<<","<< doppler_hz_filt(3)<<","<< doppler_hz_filt(4)<<endl;   
        dump_vtl_file.close(); 
	}
    
    
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
