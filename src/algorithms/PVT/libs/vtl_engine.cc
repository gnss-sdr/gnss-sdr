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
    
    static uint64_t refSampleCounter = new_data.sample_counter;
    double delta_t_vtl = (new_data.sample_counter - refSampleCounter) / 5000000.0; 
    refSampleCounter = new_data.sample_counter;
    // ################## Kalman filter initialization ######################################
    //State variables
    int n_of_states=11;
    static arma::mat kf_P_x = arma::eye(n_of_states, n_of_states) * 1.0;  //TODO: use a real value.; 
    static arma::mat kf_x = arma::zeros(n_of_states, 1);
    static arma::mat kf_dx = arma::zeros(n_of_states, 1);
    // covariances (static)
      
    kf_R = arma::zeros(3 * new_data.sat_number, 3 * new_data.sat_number);
    double kf_dt = delta_t_vtl; //0.05;
    kf_Q = arma::eye(n_of_states, n_of_states);

    kf_F = arma::eye(n_of_states, n_of_states);
    bool test = kf_F_fill(kf_F,kf_dt);

    //kf_H = arma::zeros(3 * new_data.sat_number, n_of_states);
    kf_y = arma::zeros(3 * new_data.sat_number, 1);
    kf_yerr = arma::zeros(3 * new_data.sat_number, 1);
    kf_xerr = arma::zeros(n_of_states, 1);
    kf_S = arma::zeros(3 * new_data.sat_number, 3 * new_data.sat_number);  // kf_P_y innovation covariance matrix
    kf_K = arma::zeros(n_of_states, 3 * new_data.sat_number); ;
    // ################## Kalman Tracking ######################################
    static uint32_t counter = 0;  //counter
    counter = counter + 1;        //uint64_t
    //new_data.kf_state.print("new_data kf initial");
    uint32_t closure_point=3;
    
    if (counter < closure_point)
        {  
            // // receiver solution from rtklib_solver
            kf_dx=kf_x;
            kf_x(0) = new_data.rx_p(0);
            kf_x(1) = new_data.rx_p(1);
            kf_x(2) = new_data.rx_p(2);
            kf_x(3) = new_data.rx_v(0);
            kf_x(4) = new_data.rx_v(1);
            kf_x(5) = new_data.rx_v(2);
            kf_x(6) = 0;
            kf_x(7) = 0;
            kf_x(8) = 0;
            kf_x(9) = new_data.rx_dts(0) * SPEED_OF_LIGHT_M_S;
            kf_x(10) = new_data.rx_dts(1) * SPEED_OF_LIGHT_M_S;

            kf_dx = kf_x-kf_dx;
            kf_dx = kf_F * kf_dx;  // state prediction
        }
    else
        {
            // receiver solution from previous KF step
            double acc_x = 0;
            double acc_y = 0;
            double acc_z = 0;
            test = model3DoF(acc_x,acc_x,acc_x,kf_x,kf_dt);
            kf_x(6) = acc_x;
            kf_x(7) = acc_y;
            kf_x(8) = acc_z;
            // kf_x(6) = (kf_x(6)-kf_dx(6))/kf_dt;
            // kf_x(7) = (kf_x(7)-kf_dx(7))/kf_dt;
            // kf_x(8) = (kf_x(8)-kf_dx(8))/kf_dt;
        }
   
    // State error Covariance Matrix Q (PVT)
    //careful, values for V and T could not be adecuate.
    kf_Q(0, 0) = 100.0;
    kf_Q(1, 1) = 100.0;
    kf_Q(2, 2) = 100.0;
    kf_Q(3, 3) = 8.0;
    kf_Q(4, 4) = 8.0;
    kf_Q(5, 5) = 8.0;
    kf_Q(6, 6) = .10;
    kf_Q(7, 7) = .10;
    kf_Q(8, 8) = .10;
    kf_Q(9, 9) = 4.0;
    kf_Q(10, 10) = 10.0;
 
    // Measurement error Covariance Matrix R assembling
    for (int32_t i = 0; i < new_data.sat_number; i++)
        {
            // It is diagonal 2*NSatellite x 2*NSatellite (NSat psudorange error;NSat pseudo range rate error)
            kf_R(i, i) = 80.0;//*50.0/new_data.sat_CN0_dB_hz(i);  //TODO: fill with real values.
            kf_R(i + new_data.sat_number, i + new_data.sat_number) = 20.0;//*50.0/new_data.sat_CN0_dB_hz(i);
            kf_R(i + 2*new_data.sat_number, i + 2*new_data.sat_number) = 100.0;//*50.0/new_data.sat_CN0_dB_hz(i);
            
            if(80.0*50.0/new_data.sat_CN0_dB_hz(i)>90||20.0*50.0/new_data.sat_CN0_dB_hz(i)>25){
                kf_R(i, i) = 10e4;
                kf_R(i + new_data.sat_number, i + new_data.sat_number) = 10e4;
                kf_R(i + 2*new_data.sat_number, i + 2*new_data.sat_number) = 10e4;
                cout<<"channel: "<<i<<"discarded"<<endl;
            }
        }

    // Kalman state prediction (time update)
    //new_data.kf_state=kf_x;
    //kf_x = kf_F * kf_x;                        // state prediction
    //from error state variables to variables
    // From state variables definition
    // TODO: cast to type properly

    d = arma::zeros(new_data.sat_number, 1);
    rho_pri = arma::zeros(new_data.sat_number, 1);
    rhoDot_pri = arma::zeros(new_data.sat_number, 1);

    rho_pri_filt = arma::zeros(new_data.sat_number, 1);
    rhoDot_pri_filt = arma::zeros(new_data.sat_number, 1);
    doppler_hz_filt = arma::zeros(new_data.sat_number, 1);

    a_x = arma::zeros(new_data.sat_number, 1);
    a_y = arma::zeros(new_data.sat_number, 1);
    a_z = arma::zeros(new_data.sat_number, 1);

    test = obsv_calc(rho_pri,rhoDot_pri,a_x, a_y, a_z,new_data.sat_number,new_data.sat_p,new_data.sat_v,kf_x);
    for (int32_t i = 0; i < new_data.sat_number; i++)  //neccesary quantities
        {
            new_data.sat_LOS(i, 0) = a_x(i);
            new_data.sat_LOS(i, 1) = a_y(i);
            new_data.sat_LOS(i, 2) = a_z(i);
        }

    kf_H = arma::zeros(2 * new_data.sat_number, n_of_states);
    test = kf_H_fill(kf_H,new_data.sat_number,a_x, a_y, a_z, kf_dt);
    // Kalman estimation (measurement update)
    test = kf_measurements(kf_yerr, new_data.sat_number, rho_pri, rhoDot_pri, new_data.pr_m, new_data.doppler_hz, kf_x);

    kf_P_x = kf_F * kf_P_x * kf_F.t() + kf_Q;  // state error covariance prediction
    // Kalman filter update step
    kf_S = kf_H * kf_P_x * kf_H.t() + kf_R;                     // innovation covariance matrix (S)
    arma::mat B= (kf_P_x * kf_H.t()) ;
    kf_K = B * arma::inv(kf_S);               // Kalman gain

    kf_xerr = kf_K * (kf_yerr);                                 // Error state estimation
    //kf_xerr.row(5)=kf_K.row(5)*kf_yerr;
    arma::mat A = (arma::eye(size(kf_P_x)) - kf_K * kf_H);
    kf_P_x =  A * kf_P_x * A.t() + kf_K * kf_R * kf_K.t() ;  // update state estimation error covariance matrix
    kf_dx=kf_x;
    kf_x = kf_x-kf_xerr;                                      // updated state estimation (a priori + error)
    
    //     // ################## Geometric Transformation ######################################
    test = obsv_calc(rho_pri,rhoDot_pri,a_x, a_y, a_z,new_data.sat_number,new_data.sat_p,new_data.sat_v,kf_x);
    test = kf_H_fill(kf_H,new_data.sat_number,a_x, a_y, a_z, kf_dt);
    //  Re-calculate error measurement vector with the most recent data available: kf_delta_y=kf_H*kf_delta_x
    kf_yerr = kf_H * kf_xerr;
    //  Filtered pseudorange error measurement (in m) AND Filtered Doppler shift measurements (in Hz):

    TrackingCmd trk_cmd;

    for (int32_t channel = 0; channel < new_data.sat_number; channel++)  // Measurement vector
        {
            rho_pri_filt(channel) = new_data.pr_m(channel) + kf_yerr(channel);                                                             // now filtered
            rhoDot_pri_filt(channel) = (new_data.doppler_hz(channel) * Lambda_GPS_L1) + kf_yerr(channel + new_data.sat_number);  // now filtered
            doppler_hz_filt(channel) = (rhoDot_pri_filt(channel)) / Lambda_GPS_L1; 
            //TODO: Fill the tracking commands outputs
            // Notice: keep the same satellite order as in the Vtl_Data matrices
            // sample code
            trk_cmd.carrier_phase_rads = 0;                                                // difficult of calculation
            trk_cmd.carrier_freq_hz = doppler_hz_filt(channel);  //+ kf_x(7)/Lambda_GPS_L1; // this is el doppler WITHOUTH sintony correction
            trk_cmd.carrier_freq_rate_hz_s =-(a_x(channel)*kf_x(6)+a_y(channel)*kf_x(7)+a_z(channel)*kf_x(8))/ Lambda_GPS_L1;
            trk_cmd.code_phase_chips = kf_yerr(channel)/SPEED_OF_LIGHT_M_S*1023e3;
            trk_cmd.enable_carrier_nco_cmd = true;
            trk_cmd.enable_code_nco_cmd = true;
            trk_cmd.sample_counter = new_data.sample_counter;
            trk_cmd.channel_id = 0;
            trk_cmd_outs.push_back(trk_cmd);
            // if (channel == 0)
            //     {
            //         std::cout << "[" << trk_cmd.sample_counter << "] CH " << channel
            //                   << " Doppler vtl commanded: " << doppler_hz_filt(channel) << " [Hz]"
            //                   << " \n";
            //     }
        }

    fstream dump_vtl_file;
    dump_vtl_file.open("dump_vtl_file.csv", ios::out | ios::app);
    dump_vtl_file.precision(15);
    if (!dump_vtl_file)
        {
            cout << "File not created!";
        }
    else
        {   

            if(new_data.sat_number>5){      
                dump_vtl_file << "pr_m"
                            << "," << kf_yerr(0)<< "," << kf_yerr(1)<< "," << kf_yerr(2) 
                            << "," << kf_yerr(3) << "," << kf_yerr(4)<< "," << kf_yerr(5)<<endl;
                dump_vtl_file << "prDot_m"
                            << "," << kf_yerr(new_data.sat_number)<< "," << kf_yerr(new_data.sat_number+1)<< "," << kf_yerr(new_data.sat_number+2) 
                            << "," << kf_yerr(new_data.sat_number+3) << "," << kf_yerr(new_data.sat_number+4)<<  "," << kf_yerr(new_data.sat_number+5)<< endl;
                // dump_vtl_file << "K_column"
                //             << "," << kf_K.row(5)(6) << "," << kf_K.row(5)(7) << "," <<kf_K.row(5)(8)<< "," << kf_K.row(5)(9)
                //             << "," << kf_K.row(5)(10) << "," << kf_K.row(5)(11) << endl;
                // dump_vtl_file << "K_column2"
                //             << "," << kf_K.row(5)(0) << "," << kf_K.row(5)(1) << "," <<kf_K.row(5)(2)<< "," << kf_K.row(5)(3)
                //             << "," << kf_K.row(5)(4) << "," << kf_K.row(5)(5) << endl;
            }
            dump_vtl_file << "kf_state"
                          << "," << kf_x(0) << "," << kf_x(1) << "," << kf_x(2) << "," << kf_x(3) << "," << kf_x(4) << "," << kf_x(5) << "," << kf_x(6) << "," << kf_x(7)<< "," << kf_x(8) <<"," << kf_x(9) <<"," << kf_x(10)<< endl;
            dump_vtl_file << "kf_xerr"
                          << "," << kf_xerr(0) << "," << kf_xerr(1) << "," << kf_xerr(2) << "," << kf_xerr(3) << "," << kf_xerr(4) << "," << kf_xerr(5) << "," << kf_xerr(6) << "," << kf_xerr(7)<< "," << kf_xerr(8) <<"," << kf_xerr(9) <<"," << kf_xerr(10)<< endl;
            dump_vtl_file << "rtklib_state"
                          << "," << new_data.rx_p(0) << "," << new_data.rx_p(1) << "," << new_data.rx_p(2) << "," << new_data.rx_v(0) << "," << new_data.rx_v(1) << "," << new_data.rx_v(2) << "," << new_data.rx_dts(0) << "," << new_data.rx_dts(1) << "," << delta_t_vtl  << endl;
            dump_vtl_file << "filt_dopp_sat"
                          << "," << doppler_hz_filt(0) << "," << doppler_hz_filt(1) << "," << doppler_hz_filt(2) << "," << doppler_hz_filt(3) << "," << doppler_hz_filt(4) <<endl;
            dump_vtl_file.close();
        }
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

bool Vtl_Engine::kf_H_fill(arma::mat &kf_H,int sat_number, arma::colvec ax, arma::colvec ay, arma::colvec az, double kf_dt)
{
	    for (int32_t i = 0; i < sat_number; i++)  // Measurement matrix H assembling
        {
            // It has n_of_states columns (n_of_states states) and 2*NSat rows (NSat psudorange error;NSat pseudo range rate error)
            kf_H(i, 0) = ax(i);
            kf_H(i, 1) = ay(i);
            kf_H(i, 2) = az(i);
            kf_H(i, 9) = 1.0;
            kf_H(i,10) = kf_dt;

            kf_H(i + sat_number, 3) = ax(i);
            kf_H(i + sat_number, 4) = ay(i);
            kf_H(i + sat_number, 5) = az(i);
            kf_H(i + sat_number, 6) = ax(i)*kf_dt;
            kf_H(i + sat_number, 7) = ay(i)*kf_dt;
            kf_H(i + sat_number, 8) = az(i)*kf_dt;
            kf_H(i + sat_number, 10) = 1.0;

        }

    return -1;
}

bool Vtl_Engine::kf_F_fill(arma::mat &kf_F,double kf_dt)
{
    kf_F(0, 3) = kf_dt; kf_F(0, 6) = kf_dt*kf_dt/2;
    kf_F(1, 4) = kf_dt; kf_F(1, 7) = kf_dt*kf_dt/2;
    kf_F(2, 5) = kf_dt; kf_F(2, 8) = kf_dt*kf_dt/2;

    kf_F(3, 6) = kf_dt;
    kf_F(4, 7) = kf_dt;
    kf_F(5, 8) = kf_dt;
    
    kf_F(9, 10) = kf_dt; 

    return -1;
}

bool Vtl_Engine::obsv_calc(arma::mat &rho_pri,arma::mat &rhoDot_pri,arma::colvec &ax, arma::colvec &ay, arma::colvec &az,int sat_number,arma::mat sat_p,arma::mat sat_v,arma::mat kf_x)
{
    for (int32_t i = 0; i < sat_number; i++)  //neccesary quantities
    {
        //d(i) is the distance sat(i) to receiver
        d(i) = (sat_p(i, 0) - kf_x(0)) * (sat_p(i, 0) - kf_x(0));
        d(i) = d(i) + (sat_p(i, 1) - kf_x(1)) * (sat_p(i, 1) - kf_x(1));
        d(i) = d(i) + (sat_p(i, 2) - kf_x(2)) * (sat_p(i, 2) - kf_x(2));
        d(i) = sqrt(d(i));

        //compute pseudorange estimation
        rho_pri(i) = d(i) + kf_x(9);
        //compute LOS sat-receiver vector componentsx
        ax(i) = -(sat_p(i, 0) - kf_x(0)) / d(i);
        ay(i) = -(sat_p(i, 1) - kf_x(1)) / d(i);
        az(i) = -(sat_p(i, 2) - kf_x(2)) / d(i);
        //compute pseudorange rate estimation
        rhoDot_pri(i) = (sat_v(i, 0) - kf_x(3)) * a_x(i) + (sat_v(i, 1) - kf_x(4)) * a_y(i) + (sat_v(i, 2) - kf_x(5)) * a_z(i);
        //rhoDot_pri(i) = rhoDot_pri(i) + a_x(i)*xDot2_u*kf_dt+a_y(i)*yDot2_u*kf_dt+a_z(i)*zDot2_u*kf_dt;
    }
	return -1;
}

bool Vtl_Engine::kf_measurements(arma::mat &kf_yerr, int sat_number, arma::mat rho_pri, arma::mat rhoDot_pri, arma::colvec pr_m, arma::colvec doppler_hz, arma::mat kf_x)
{
    for (int32_t i = 0; i < sat_number; i++)  // Measurement vector
    {
        kf_yerr(i) = rho_pri(i) - pr_m(i);
        kf_yerr(i + sat_number) = (doppler_hz(i) * Lambda_GPS_L1+kf_x(10)) - rhoDot_pri(i);
    }
    return -1;
}

bool Vtl_Engine::model3DoF(double acc_x,double acc_y,double acc_z,arma::mat kf_x,double dt)
{
    arma::colvec u_vec;
    arma::colvec acc_vec;
    arma::colvec u_dir;
    arma::colvec gravity_ECEF = {-7.826024, 0.8616969, -5.833042}; //lat=36.533333 lon=-6.283333
    static double t_disparo;
    double Empuje;
    double densidad=1.0;
    double ballistic_coef = 0.007;
    double PI = acos(-1.0);
    //vector velocidad
    u_vec = kf_x.rows(4, 6);
    // double u=sqrt(pow(kf_x(4),2)+pow(kf_x(5),2)+pow(kf_x(6),2));

    //modulo de la velocidad
    double u = norm(u_vec, 2);
    
    if(u>4){
        t_disparo=t_disparo+dt;
        double diam_cohete=120.0e-3;// 120 mm
        double mass_rocket=50.0; //50Kg
        
        if(t_disparo<.2){
            u_dir={.90828, -.13984, .388756};
        }else{
            u_dir = u_vec / u;        
        }
        // lla= ecef2lla([kf_State(1) kf_State(2) kf_State(3)]);
        // [T, sound_v, P, densidad] = atmosisa(lla(3));
        // sound_v=320;% @ 5km and -17.5C
        // Mach=u/sound_v;
        // CD0 = Cd0_M_LookTable(Mach);
        // % ballistic_coef is Cd0/mass_rocket;
        // ballistic_coef=CD0/mass_rocket;
        // Empuje = EmpujeLookTable(t_disparo);
                
        acc_vec = -(PI*densidad*diam_cohete*diam_cohete/8)*ballistic_coef*u*u_dir
        +gravity_ECEF+Empuje*u_dir;
        
        // % return
        acc_x = acc_vec(0);
        acc_y = acc_vec(1);
        acc_z = acc_vec(2); 
    }else{
        // % return
        acc_x = 0;
        acc_y = 0;
        acc_z = 0;
    }

       return -1;
}

double Vtl_Engine::EmpujeLkTable(double t_disparo)
{
    return -1.0;
}