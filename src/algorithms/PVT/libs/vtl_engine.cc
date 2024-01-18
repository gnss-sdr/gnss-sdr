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
    counter = 0;
    refSampleCounter = 0;
    n_of_states = 11;
    delta_t_cmd = 0;

    kf_P_x = arma::eye(n_of_states, n_of_states) * 1.0;  // TODO: use a real value.;
    kf_x = arma::zeros(n_of_states, 1);
}

Vtl_Engine::~Vtl_Engine()
{
}

bool Vtl_Engine::vtl_loop(Vtl_Data new_data)
{
    // TODO: Implement main VTL loop here
    using arma::as_scalar;

    if (refSampleCounter == 0)
        {
            refSampleCounter = new_data.sample_counter;
        }
    double delta_t_vtl = (new_data.sample_counter - refSampleCounter) / 5000000.0;
    refSampleCounter = new_data.sample_counter;

    bool flag_cmd = false;
    bool flag_time_cmd = false;
    delta_t_cmd = delta_t_cmd + delta_t_vtl;  // update timer for vtl trk command
    if (delta_t_cmd >= 0.3)
        {
            flag_cmd = true;
            delta_t_cmd = 0;  // reset timer for vtl trk command
        }
    // ################## Kalman filter initialization ######################################
    // State variables

    arma::mat kf_dx = arma::zeros(n_of_states, 1);
    // covariances (static)

    kf_R = arma::zeros(3 * new_data.sat_number, 3 * new_data.sat_number);
    double kf_dt = delta_t_vtl;  // 0.05;
    kf_Q = arma::eye(n_of_states, n_of_states);

    kf_F = arma::eye(n_of_states, n_of_states);

    kf_y = arma::zeros(3 * new_data.sat_number, 1);
    kf_yerr = arma::zeros(3 * new_data.sat_number, 1);
    kf_xerr = arma::zeros(n_of_states, 1);
    kf_S = arma::zeros(3 * new_data.sat_number, 3 * new_data.sat_number);  // kf_P_y innovation covariance matrix
    kf_K = arma::zeros(n_of_states, 3 * new_data.sat_number);
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
    // ################## Kalman Tracking ######################################
    counter++;  // uint64_t

    if (counter > 2500)
        {
            flag_time_cmd = true;
        }
    uint32_t closure_point = 3;

    // State error Covariance Matrix Q (PVT)
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
            kf_R(i, i) = 80.0;
            kf_R(i + new_data.sat_number, i + new_data.sat_number) = 20.0;
            kf_R(i + 2 * new_data.sat_number, i + 2 * new_data.sat_number) = 40.0;

            if (i == 6)
                {
                    kf_R(i, i) = 10e5;
                    kf_R(i + new_data.sat_number, i + new_data.sat_number) = 10e5;
                    kf_R(i + 2 * new_data.sat_number, i + 2 * new_data.sat_number) = 10e6;
                }
        }

    //**************************************
    // Kalman state prediction (time update)
    //**************************************

    if (counter < closure_point)
        {
            // receiver solution from rtklib_solver
            kf_dx = kf_x;
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

            kf_dx = kf_x - kf_dx;
            kf_dx = kf_F * kf_dx;  // state prediction/*  */
        }
    else
        {
            // receiver solution from previous KF step
            kf_x = kf_x;
            // acceleration model
            double acc_x = 0;
            double acc_y = 0;
            double acc_z = 0;
            kf_x(6) = acc_x;
            kf_x(7) = acc_y;
            kf_x(8) = acc_z;
            // kf_x(6) = (kf_x(6)-kf_dx(6))/kf_dt;
            // kf_x(7) = (kf_x(7)-kf_dx(7))/kf_dt;
            // kf_x(8) = (kf_x(8)-kf_dx(8))/kf_dt;
        }
    kf_F_fill(kf_F, kf_dt, kf_x);
    obsv_calc(rho_pri, rhoDot_pri, rhoDot2_pri, a_x, a_y, a_z, new_data.sat_number, new_data.sat_p, new_data.sat_v, kf_x);
    kf_P_x = kf_F * kf_P_x * kf_F.t() + kf_Q;  // state error covariance prediction

    //**************************************
    // Kalman estimation (measurement update)
    //**************************************

    kf_H = arma::zeros(3 * new_data.sat_number, n_of_states);
    kf_H_fill(kf_H, new_data.sat_number, a_x, a_y, a_z, kf_dt);
    kf_measurements(kf_yerr, new_data.sat_number, rho_pri, rhoDot_pri, rhoDot2_pri, new_data.pr_m, new_data.doppler_hz, kf_x);
    //**************************************
    // Kalman filter update step
    //**************************************

    kf_S = kf_H * kf_P_x * kf_H.t() + kf_R;  // innovation covariance matrix (S)
    arma::mat B = (kf_P_x * kf_H.t());
    kf_K = B * arma::inv(kf_S);  // Kalman gain

    kf_xerr = kf_K * (kf_yerr);  // Error state estimation
    arma::mat A = (arma::eye(size(kf_P_x)) - kf_K * kf_H);
    kf_P_x = A * kf_P_x * A.t() + kf_K * kf_R * kf_K.t();  // update state estimation error covariance matrix
    kf_x = kf_x - kf_xerr;                                 // updated state estimation (a priori + error)
    kf_dx = kf_x;

    //*************************
    // Geometric Transformation
    //*************************

    obsv_calc(rho_pri, rhoDot_pri, rhoDot2_pri, a_x, a_y, a_z, new_data.sat_number, new_data.sat_p, new_data.sat_v, kf_x);
    kf_H_fill(kf_H, new_data.sat_number, a_x, a_y, a_z, kf_dt);

    //  Re-calculate error measurement vector with the most recent data available: kf_delta_y=kf_H*kf_delta_x
    kf_yerr = kf_H * kf_xerr;
    //  Filtered pseudorange error measurement (in m) AND Filtered Doppler shift measurements (in Hz):

    TrackingCmd trk_cmd;

    for (int32_t channel = 0; channel < new_data.sat_number; channel++)  // Measurement vector
        {
            rho_pri_filt(channel) = new_data.pr_m(channel) + kf_yerr(channel);                                                   // now filtered
            rhoDot_pri_filt(channel) = (new_data.doppler_hz(channel) * Lambda_GPS_L1) + kf_yerr(channel + new_data.sat_number);  // now filtered
            doppler_hz_filt(channel) = (rhoDot_pri_filt(channel)) / Lambda_GPS_L1;

            trk_cmd.carrier_phase_rads = 0;                      // difficult of calculation
            trk_cmd.carrier_freq_hz = doppler_hz_filt(channel);  // this is el doppler WITHOUTH sintony correction
            trk_cmd.carrier_freq_rate_hz_s = -(a_x(channel) * kf_x(6) + a_y(channel) * kf_x(7) + a_z(channel) * kf_x(8)) / Lambda_GPS_L1;
            trk_cmd.code_phase_chips = 0;  // kf_yerr(channel)/SPEED_OF_LIGHT_M_S*1023e3;

            if (flag_time_cmd)
                {
                    if (flag_cmd)
                        {
                            trk_cmd.enable_carrier_nco_cmd = true;
                        }
                    else
                        {
                            trk_cmd.enable_carrier_nco_cmd = false;  // do NOT apply corrections! loop convergence issue
                        }
                }
            else
                {
                    trk_cmd.enable_carrier_nco_cmd = false;  // do NOT apply corrections! loop convergence issue
                }


            trk_cmd.sample_counter = new_data.sample_counter;
            trk_cmd.channel_id = channel;
            trk_cmd_outs.push_back(trk_cmd);
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
            dump_vtl_file << "kf_state"
                          << "," << kf_x(0) << "," << kf_x(1) << "," << kf_x(2) << "," << kf_x(3) << "," << kf_x(4) << "," << kf_x(5) << "," << kf_x(6) << "," << kf_x(7) << "," << kf_x(8) << "," << kf_x(9) << "," << kf_x(10) << endl;
            dump_vtl_file << "kf_xerr"
                          << "," << kf_xerr(0) << "," << kf_xerr(1) << "," << kf_xerr(2) << "," << kf_xerr(3) << "," << kf_xerr(4) << "," << kf_xerr(5) << "," << kf_xerr(6) << "," << kf_xerr(7) << "," << kf_xerr(8) << "," << kf_xerr(9) << "," << kf_xerr(10) << endl;
            dump_vtl_file << "rtklib_state"
                          << "," << new_data.rx_p(0) << "," << new_data.rx_p(1) << "," << new_data.rx_p(2) << "," << new_data.rx_v(0) << "," << new_data.rx_v(1) << "," << new_data.rx_v(2) << "," << new_data.rx_dts(0) << "," << new_data.rx_dts(1) << "," << delta_t_vtl << endl;
            // dump_vtl_file << "filt_dopp_sat"
            //               << "," << doppler_hz_filt(0) << "," << doppler_hz_filt(1) << "," << doppler_hz_filt(2) << "," << doppler_hz_filt(3) << "," << doppler_hz_filt(4) <<endl;
            dump_vtl_file.close();
        }
    return true;
}

void Vtl_Engine::reset()
{
    // TODO
}

void Vtl_Engine::debug_print()
{
    // TODO
}

void Vtl_Engine::configure(Vtl_Conf config_)
{
    config = config_;
    // TODO: initialize internal variables
}

void Vtl_Engine::kf_H_fill(arma::mat &kf_H, int sat_number, arma::colvec ax, arma::colvec ay, arma::colvec az, double kf_dt)
{
    for (int32_t i = 0; i < sat_number; i++)  // Measurement matrix H assembling
        {
            // It has n_of_states columns (n_of_states states) and 2*NSat rows (NSat psudorange error;NSat pseudo range rate error)
            kf_H(i, 0) = ax(i);
            kf_H(i, 1) = ay(i);
            kf_H(i, 2) = az(i);
            kf_H(i, 9) = 1.0;
            kf_H(i, 10) = kf_dt;

            kf_H(i + sat_number, 3) = ax(i);
            kf_H(i + sat_number, 4) = ay(i);
            kf_H(i + sat_number, 5) = az(i);
            kf_H(i + sat_number, 6) = ax(i) * kf_dt;
            kf_H(i + sat_number, 7) = ay(i) * kf_dt;
            kf_H(i + sat_number, 8) = az(i) * kf_dt;
            kf_H(i + sat_number, 10) = 1.0;

            kf_H(i + 2 * sat_number, 3) = 0;  // ax(i);
            kf_H(i + 2 * sat_number, 4) = 0;  // ay(i);
            kf_H(i + 2 * sat_number, 5) = 0;  // az(i);
            kf_H(i + 2 * sat_number, 6) = ax(i);
            kf_H(i + 2 * sat_number, 7) = ay(i);
            kf_H(i + 2 * sat_number, 8) = az(i);
            kf_H(i + 2 * sat_number, 10) = kf_dt;
        }
}
void Vtl_Engine::kf_F_fill(arma::mat &kf_F, double kf_dt, arma::mat &kf_x)
{
    // modulo de la velocidad
    double vx = kf_x(3);
    double vy = kf_x(4);
    double vz = kf_x(5);
    double u = norm(kf_x.rows(3, 5), 2);

    kf_F(0, 3) = kf_dt;
    kf_F(0, 6) = kf_dt * kf_dt / 2;
    kf_F(1, 4) = kf_dt;
    kf_F(1, 7) = kf_dt * kf_dt / 2;
    kf_F(2, 5) = kf_dt;
    kf_F(2, 8) = kf_dt * kf_dt / 2;

    kf_F(3, 6) = kf_dt;
    kf_F(4, 7) = kf_dt;
    kf_F(5, 8) = kf_dt;

    kf_F(6, 3) = (vx * vx / u + u);
    kf_F(7, 4) = (vy * vy / u + u);
    kf_F(8, 5) = (vz * vz / u + u);

    kf_F(9, 10) = kf_dt;
}
void Vtl_Engine::kf_F_fill_rocket(arma::mat &kf_F, double kf_dt, arma::mat &kf_x)
{
    double densidad = 1.0;
    double ballistic_coef = 0.007;
    double diam_cohete = 120.0e-3;  // 120 mm
    double beta = (GNSS_PI * densidad * diam_cohete * diam_cohete / 8) * ballistic_coef;
    // modulo de la velocidad
    double vx = kf_x(3);
    double vy = kf_x(4);
    double vz = kf_x(5);
    double u = norm(kf_x.rows(3, 5), 2);

    kf_F(0, 3) = kf_dt;
    kf_F(0, 6) = kf_dt * kf_dt / 2;
    kf_F(1, 4) = kf_dt;
    kf_F(1, 7) = kf_dt * kf_dt / 2;
    kf_F(2, 5) = kf_dt;
    kf_F(2, 8) = kf_dt * kf_dt / 2;

    kf_F(3, 6) = kf_dt;
    kf_F(4, 7) = kf_dt;
    kf_F(5, 8) = kf_dt;

    kf_F(6, 3) = -beta * (vx * vx / u + u);
    kf_F(7, 4) = -beta * (vy * vy / u + u);
    kf_F(8, 5) = -beta * (vz * vz / u + u);

    kf_F(9, 10) = kf_dt;
}

void Vtl_Engine::obsv_calc(arma::mat &rho_pri, arma::mat &rhoDot_pri, arma::mat &rhoDot2_pri, arma::colvec &ax, arma::colvec &ay, arma::colvec &az, int sat_number, arma::mat sat_p, arma::mat sat_v, arma::mat kf_x)
{
    for (int32_t i = 0; i < sat_number; i++)  // neccesary quantities
        {
            // d(i) is the distance sat(i) to receiver
            d(i) = (sat_p(i, 0) - kf_x(0)) * (sat_p(i, 0) - kf_x(0));
            d(i) = d(i) + (sat_p(i, 1) - kf_x(1)) * (sat_p(i, 1) - kf_x(1));
            d(i) = d(i) + (sat_p(i, 2) - kf_x(2)) * (sat_p(i, 2) - kf_x(2));
            d(i) = sqrt(d(i));

            // compute pseudorange estimation OUTPUT
            rho_pri(i) = d(i) + kf_x(9);
            // compute LOS sat-receiver vector componentsx
            ax(i) = -(sat_p(i, 0) - kf_x(0)) / d(i);
            ay(i) = -(sat_p(i, 1) - kf_x(1)) / d(i);
            az(i) = -(sat_p(i, 2) - kf_x(2)) / d(i);
            // compute pseudorange rate estimation OUTPUT
            rhoDot_pri(i) = (sat_v(i, 0) - kf_x(3)) * a_x(i) + (sat_v(i, 1) - kf_x(4)) * a_y(i) + (sat_v(i, 2) - kf_x(5)) * a_z(i);
            rhoDot2_pri(i) = (kf_x(6)) * a_x(i) + (kf_x(7)) * a_y(i) + (kf_x(8)) * a_z(i);
        }
}

void Vtl_Engine::kf_measurements(arma::mat &kf_yerr, int sat_number, arma::mat rho_pri, arma::mat rhoDot_pri, arma::mat rhoDot2_pri, arma::colvec pr_m, arma::colvec doppler_hz, arma::mat kf_x)
{
    for (int32_t i = 0; i < sat_number; i++)  // Measurement vector OUTPUT
        {
            kf_yerr(i) = rho_pri(i) - pr_m(i);
            kf_yerr(i + sat_number) = (doppler_hz(i) * Lambda_GPS_L1 + kf_x(10)) - rhoDot_pri(i);
            kf_yerr(i + 2 * sat_number) = -rhoDot2_pri(i);
        }
}

std::vector<double> Vtl_Engine::get_position_ecef_m()
{
    std::vector<double> temp = {42, 42, 42};
    temp[0] = kf_x[0];
    temp[1] = kf_x[1];
    temp[2] = kf_x[2];

    return temp;
}

std::vector<double> Vtl_Engine::get_velocity_ecef_m_s()
{
    std::vector<double> temp = {42, 42, 42};
    temp[0] = kf_x[3];
    temp[1] = kf_x[4];
    temp[2] = kf_x[5];

    return temp;
}

std::vector<double> Vtl_Engine::get_accel_ecef_m_s2()
{
    std::vector<double> temp = {42, 42, 42};
    temp[0] = kf_x[6];
    temp[1] = kf_x[7];
    temp[2] = kf_x[8];

    return temp;
}
std::vector<double> Vtl_Engine::get_position_var_ecef_m()
{
    std::vector<double> temp = {42, 42, 42};
    temp[0] = kf_P_x(0, 0);
    temp[1] = kf_P_x(1, 1);
    temp[2] = kf_P_x(2, 2);

    return temp;
}

std::vector<double> Vtl_Engine::get_velocity_var_ecef_m_s()
{
    std::vector<double> temp = {42, 42, 42};
    temp[0] = kf_P_x(3, 3);
    temp[1] = kf_P_x(4, 4);
    temp[2] = kf_P_x(5, 5);

    return temp;
}

std::vector<double> Vtl_Engine::get_accel_var_ecef_m_s2()
{
    std::vector<double> temp = {42, 42, 42};
    temp[0] = kf_P_x(6, 6);
    temp[1] = kf_P_x(7, 7);
    temp[2] = kf_P_x(8, 8);

    return temp;
}

double Vtl_Engine::get_latitude()
{
    return -1.0;
}

double Vtl_Engine::get_longitude()
{
    return -1.0;
}

double Vtl_Engine::get_height()
{
    return -1.0;
}

double Vtl_Engine::get_user_clock_offset_s()
{
    double temp = 0;
    temp = kf_x[9];

    return temp;
}

double Vtl_Engine::get_user_clock_offset_drift_s_s()
{
    double temp = 0;
    temp = kf_x[10];

    return temp;
}