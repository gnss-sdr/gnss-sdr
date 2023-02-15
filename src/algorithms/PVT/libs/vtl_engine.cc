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

    //TODO: resolver el problema cuando cambie el numero de sat!!
    // static arma::colvec rhoDot_pri_old = arma::zeros(new_data.sat_number, 1);
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
            test = model3DoF(acc_x,acc_y,acc_z,kf_x,kf_dt);
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
            kf_R(i + 2*new_data.sat_number, i + 2*new_data.sat_number) = 400.0;//*50.0/new_data.sat_CN0_dB_hz(i);
            
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
    rhoDot2_pri = arma::zeros(new_data.sat_number, 1);

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

    kf_H = arma::zeros(3 * new_data.sat_number, n_of_states);
    test = kf_H_fill(kf_H,new_data.sat_number,a_x, a_y, a_z, kf_dt);

    for (int32_t i = 0; i < new_data.sat_number; i++)  //neccesary quantities
    {
        // rhoDot2_pri(i)=(rhoDot_pri(i)-rhoDot_pri_old(i))/kf_dt;
    }
    // Kalman estimation (measurement update)
    test = kf_measurements(kf_yerr, new_data.sat_number, rho_pri, rhoDot_pri, rhoDot_pri*0, new_data.pr_m, new_data.doppler_hz, kf_x);

    kf_P_x = kf_F * kf_P_x * kf_F.t() + kf_Q;  // state error covariance prediction
    // Kalman filter update step
    kf_S = kf_H * kf_P_x * kf_H.t() + kf_R;    // innovation covariance matrix (S)
    arma::mat B= (kf_P_x * kf_H.t()) ;
    kf_K = B * arma::inv(kf_S);               // Kalman gain

    kf_xerr = kf_K * (kf_yerr);               // Error state estimation
    //kf_xerr.row(5)=kf_K.row(5)*kf_yerr;
    arma::mat A = (arma::eye(size(kf_P_x)) - kf_K * kf_H);
    kf_P_x =  A * kf_P_x * A.t() + kf_K * kf_R * kf_K.t() ;  // update state estimation error covariance matrix
    kf_dx=kf_x;
    kf_x = kf_x-kf_xerr;  // updated state estimation (a priori + error)
    
    //     // ################## Geometric Transformation ######################################
    test = obsv_calc(rho_pri,rhoDot_pri,a_x, a_y, a_z,new_data.sat_number,new_data.sat_p,new_data.sat_v,kf_x);
    
       for (int32_t i = 0; i < new_data.sat_number; i++)  //neccesary quantities
    {
        //acc_effect(i)=(a_x(i)*kf_state(7,t)+a_y(chan,t)*kf_state(8,t)+a_z(chan,t)*kf_state(9,t));
        //rhoDot2_pri(chan,t)=(rhoDot_pri(chan,t)-rhoDot_pri(chan,t-1))/kf_dt;
        //rhoDot2_pri(chan,t)=-acc_effect(chan,t);
    }
    
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
            trk_cmd.carrier_freq_rate_hz_s =-(a_x(channel)*kf_x(6)+a_y(channel)*kf_x(7)+a_z(channel)*kf_x(8)) / Lambda_GPS_L1;
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

            kf_H(i + 2*sat_number, 3) = 0;//ax(i);
            kf_H(i + 2*sat_number, 4) = 0;//ay(i);
            kf_H(i + 2*sat_number, 5) = 0;//az(i);
            kf_H(i + 2*sat_number, 6) = ax(i);
            kf_H(i + 2*sat_number, 7) = ay(i);
            kf_H(i + 2*sat_number, 8) = az(i);
            kf_H(i + 2*sat_number, 10) = kf_dt;
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

bool Vtl_Engine::kf_measurements(arma::mat &kf_yerr, int sat_number, arma::mat rho_pri, arma::mat rhoDot_pri, arma::mat rhoDot2_pri, arma::colvec pr_m, arma::colvec doppler_hz, arma::mat kf_x)
{
    for (int32_t i = 0; i < sat_number; i++)  // Measurement vector
    {
        kf_yerr(i) = rho_pri(i) - pr_m(i);
        kf_yerr(i + sat_number) = (doppler_hz(i) * Lambda_GPS_L1+kf_x(10)) - rhoDot_pri(i);
        kf_yerr(i + 2*sat_number) = -rhoDot2_pri(i);
    }
    return -1;
}

bool Vtl_Engine::model3DoF(double &acc_x,double &acc_y,double &acc_z,arma::mat kf_x,double dt)
{
    arma::colvec u_vec;
    arma::colvec acc_vec;
    arma::colvec u_dir;
    arma::colvec gravity_ECEF = {-7.826024, 0.8616969, -5.833042}; //lat=36.533333 lon=-6.283333
    static double t_disparo=0;
    double Empuje;
    double densidad=1.0;
    double ballistic_coef = 0.007;
    //vector velocidad

    u_vec = kf_x.rows(3, 5);

    //modulo de la velocidad
    double u = norm(u_vec, 2);
  
    if(u>4){
        t_disparo=t_disparo+dt;
        cout<<"t_disparo: "<<t_disparo<<endl;
        double diam_cohete=120.0e-3;// 120 mm
        double mass_rocket=50.0; //50Kg
        
        if(t_disparo<.2){
            u_dir={.90828, -.13984, -.388756}; 
        }else{
            u_dir = u_vec / u;        
        }
        // u_dir.print("u_dir");
        // lla= ecef2lla([kf_State(1) kf_State(2) kf_State(3)]);
        // [T, sound_v, P, densidad] = atmosisa(lla(3));
        // sound_v=320;% @ 5km and -17.5C
        // Mach=u/sound_v;
        // CD0 = Cd0_M_LookTable(Mach);
        // % ballistic_coef is Cd0/mass_rocket;
        // ballistic_coef=CD0/mass_rocket;
        Empuje = EmpujeLkTable(t_disparo);
        cout<<"Empuje: "<<Empuje<<endl;  

        acc_vec = -(GNSS_PI*densidad*diam_cohete*diam_cohete/8)*ballistic_coef*u*u_dir
        +gravity_ECEF+Empuje*u_dir;

        // acc_vec.print("acc_vec");
        // % return
        acc_x = acc_vec(0);
        acc_y = acc_vec(1);
        acc_z = acc_vec(2); 
    }else{
        t_disparo=0;
        // % return
        acc_x = 0;
        acc_y = 0;
        acc_z = 0;
    }

       return -1;
}

double Vtl_Engine::EmpujeLkTable(double t_disparo)
{   
    double E;
    arma::mat LkTable={
    {0.0,	                391.083112445424},
    {0.0100578034682081,	385.626317230813},
    {0.0201156069364162,	379.253652903964},
    {0.0301734104046243,	372.850418310078},
    {0.0402312138728324,	366.435105395212},
    {0.0502890173410405,	359.948724887310},
    {0.0603468208092486,	353.452370826679},
    {0.0704046242774566,	346.915160536406},
    {0.0804624277456647,	340.353374212744},
    {0.0905202312138728,	339.982366920698},
    {0.100578034682081,	339.649644036322},
    {0.110635838150289,	339.313119301332},
    {0.120693641618497,	338.971249841340},
    {0.130751445086705,	338.626092336370},
    {0.140809248554913,	338.277926096280},
    {0.150867052023121,	337.923860794114},
    {0.160924855491329,	337.564686821652},
    {0.170982658959538,	337.204565166310},
    {0.181040462427746,	336.836327593982},
    {0.191098265895954,	337.612574596978},
    {0.201156069364162,	338.389812277202},
    {0.211213872832370,	339.172197383571},
    {0.221271676300578,	339.950864133333},
    {0.231329479768786,	340.734286946588},
    {0.241387283236994,	341.513599696996},
    {0.251445086705202,	342.296487929689},
    {0.261502890173410,	343.077730186615},
    {0.271560693641619,	343.862401553296},
    {0.281618497109827,	344.645045182721},
    {0.291676300578035,	345.430222609429},
    {0.301734104046243,	346.216363751051},
    {0.311791907514451,	347.003086022300},
    {0.321849710982659,	347.790403464500},
    {0.331907514450867,	348.576632890101},
    {0.341965317919075,	349.365041767204},
    {0.352023121387283,	350.154345504950},
    {0.362080924855491,	350.943780688588},
    {0.372138728323699,	351.735714016931},
    {0.382196531791908,	352.523676077225},
    {0.392254335260116,	353.317052045327},
    {0.402312138728324,	354.110411623268},
    {0.412369942196532,	354.903279067692},
    {0.422427745664740,	355.692559158889},
    {0.432485549132948,	356.490274978154},
    {0.442543352601156,	357.283451900588},
    {0.452601156069364,	358.078507291348},
    {0.462658959537572,	358.871323078342},
    {0.472716763005780,	359.668187019647},
    {0.482774566473988,	360.465893772213},
    {0.492832369942197,	361.264486481857},
    {0.502890173410405,	362.057713788838},
    {0.512947976878613,	362.855398652135},
    {0.523005780346821,	363.651958513907},
    {0.533063583815029,	364.453139119421},
    {0.543121387283237,	365.248494477390},
    {0.553179190751445,	366.047440571574},
    {0.563236994219653,	366.844541693072},
    {0.573294797687861,	367.645722991578},
    {0.583352601156069,	368.431281642859},
    {0.593410404624277,	369.218032671753},
    {0.603468208092486,	370.005763838889},
    {0.613526011560694,	370.793449087224},
    {0.623583815028902,	371.577331994297},
    {0.633641618497110,	372.361872668242},
    {0.643699421965318,	373.139234321453},
    {0.653757225433526,	373.922395308171},
    {0.663815028901734,	374.698638233448},
    {0.673872832369942,	375.477674739791},
    {0.683930635838150,	376.256375688174},
    {0.693988439306358,	377.561519397833},
    {0.704046242774567,	378.859304679191},
    {0.714104046242775,	380.167718611141},
    {0.724161849710983,	381.477582211590},
    {0.734219653179191,	382.785006750526},
    {0.744277456647399,	384.092555921096},
    {0.754335260115607,	385.403347233778},
    {0.764393063583815,	386.680222025901},
    {0.774450867052023,	387.960348600215},
    {0.784508670520231,	389.241465657245},
    {0.794566473988439,	390.517609695133},
    {0.804624277456647,	391.778341124162},
    {0.814682080924856,	393.038366474572},
    {0.824739884393064,	394.291210028831},
    {0.834797687861272,	395.545183108074},
    {0.844855491329480,	396.779476009029},
    {0.854913294797688,	398.005131564908},
    {0.864971098265896,	399.217607990930},
    {0.875028901734104,	400.433585037519},
    {0.885086705202312,	401.639603766860},
    {0.895144508670520,	402.824960888722},
    {0.905202312138728,	403.999818917039},
    {0.915260115606936,	405.164413592803},
    {0.925317919075145,	406.332094769783},
    {0.935375722543353,	407.489826389568},
    {0.945433526011561,	408.638695158694},
    {0.955491329479769,	409.788208430892},
    {0.965549132947977,	410.920696471959},
    {0.975606936416185,	412.071117188526},
    {0.985664739884393,	413.227362102269},
    {0.995722543352601,	414.376295532213},
    {1.00578034682081,	415.517208260982},
    {1.01583815028902,	416.685111645473},
    {1.02589595375723,	417.874034388355},
    {1.03595375722543,	419.050152356493},
    {1.04601156069364,	420.226540416000},
    {1.05606936416185,	421.431674949807},
    {1.06612716763006,	422.651832857732},
    {1.07618497109827,	423.879432015756},
    {1.08624277456647,	425.096859052146},
    {1.09630057803468,	426.370514159926},
    {1.10635838150289,	427.681420607676},
    {1.11641618497110,	428.990653815442},
    {1.12647398843931,	430.308007852515},
    {1.13653179190751,	431.623136377095},
    {1.14658959537572,	432.942354714805},
    {1.15664739884393,	434.260396644686},
    {1.16670520231214,	435.580636665589},
    {1.17676300578035,	436.903819649131},
    {1.18682080924855,	438.235977095674},
    {1.19687861271676,	439.760282970165},
    {1.20693641618497,	441.285566201751},
    {1.21699421965318,	442.823791210454},
    {1.22705202312139,	444.354330708051},
    {1.23710982658960,	445.893589623130},
    {1.24716763005780,	447.437414139676},
    {1.25722543352601,	449.016306037988},
    {1.26728323699422,	450.605733417807},
    {1.27734104046243,	452.197570123945},
    {1.28739884393064,	453.799041921642},
    {1.29745664739884,	455.395152193126},
    {1.30751445086705,	457.004209080179},
    {1.31757225433526,	458.623015526914},
    {1.32763005780347,	460.238728741943},
    {1.33768786127168,	461.860881948233},
    {1.34774566473988,	463.495649237401},
    {1.35780346820809,	465.129826626932},
    {1.36786127167630,	466.767033528504},
    {1.37791907514451,	468.408766204263},
    {1.38797687861272,	470.059173838784},
    {1.39803468208092,	471.721126584341},
    {1.40809248554913,	473.414912587257},
    {1.41815028901734,	475.097553575053},
    {1.42820809248555,	476.805256760032},
    {1.43826589595376,	478.512506689219},
    {1.44832369942197,	480.230235676597},
    {1.45838150289017,	481.950516788809},
    {1.46843930635838,	483.675596525192},
    {1.47849710982659,	485.408520318711},
    {1.48855491329480,	487.160694244723},
    {1.49861271676301,	488.912737035966},
    {1.50867052023121,	490.676335211920},
    {1.51872832369942,	492.446193358219},
    {1.52878612716763,	494.230761014528},
    {1.53884393063584,	496.014213821540},
    {1.54890173410405,	497.800012215749},
    {1.55895953757225,	499.586844428474},
    {1.56901734104046,	501.389791490613},
    {1.57907514450867,	503.197873321221},
    {1.58913294797688,	505.017976823598},
    {1.59919075144509,	506.842077493572},
    {1.60924855491329,	508.681008225194},
    {1.61930635838150,	510.531745060971},
    {1.62936416184971,	512.383464849326},
    {1.63942196531792,	514.249250804510},
    {1.64947976878613,	516.119098877381},
    {1.65953757225434,	518.005308788433},
    {1.66959537572254,	519.910761252352},
    {1.67965317919075,	521.812275649970},
    {1.68971098265896,	523.727747256148},
    {1.69976878612717,	526.524959987653},
    {1.70982658959538,	529.326074922632},
    {1.71988439306358,	532.152158438731},
    {1.72994219653179,	534.995939192065},
    {1.74000000000000,	537.866310625605},};

    //encuentra el mas cercano justo anterior.
    // int index_E = LkTable.elem(find(LkTable<=t_disparo)).max(); 
    arma::uvec index_E = find(LkTable<=t_disparo, 1, "last");
    index_E.print("indice E: ");
    // uint kk = index_E(0);
    if (index_E(0)<(LkTable.n_rows-1)){  
        double tdisparo1=LkTable(index_E(0),0);
        double tdisparo2=LkTable(index_E(0)+1,0);
        double E1=LkTable(index_E(0),1);
        double E2=LkTable(index_E(0)+1,1);
        
        E=(t_disparo-tdisparo1)*(E2-E1)/(tdisparo2-tdisparo1)+E1;
    }else{
        E=0;
    }
    
    return E;
}