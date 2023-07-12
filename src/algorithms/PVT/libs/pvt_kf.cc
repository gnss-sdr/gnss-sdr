/*!
 * \file pvt_kf.cc
 * \brief Kalman Filter for Position and Velocity
 * \author Javier Arribas, 2023. jarribas(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "pvt_kf.h"
#include <glog/logging.h>


void Pvt_Kf::init_Kf(const arma::vec& p,
    const arma::vec& v,
    const arma::vec& res_p,
    double solver_interval_s,
    bool estatic_measures_sd,
    double measures_ecef_pos_sd_m,
    double measures_ecef_vel_sd_ms,
    double system_ecef_pos_sd_m,
    double system_ecef_vel_sd_ms)
{
    // Kalman Filter class variables
    const double Ti = solver_interval_s;

    d_F = {{1.0, 0.0, 0.0, Ti, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0, Ti, 0.0},
        {0.0, 0.0, 1.0, 0.0, 0.0, Ti},
        {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};

    d_H = arma::eye(6, 6);

    // measurement matrix static covariances
    if(estatic_measures_sd){
        d_R = {{pow(measures_ecef_pos_sd_m, 2.0), 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, pow(measures_ecef_pos_sd_m, 2.0), 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, pow(measures_ecef_pos_sd_m, 2.0), 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, pow(measures_ecef_vel_sd_ms, 2.0), 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, pow(measures_ecef_vel_sd_ms, 2.0), 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, pow(measures_ecef_vel_sd_ms, 2.0)}};

        d_static = true;

        }else{
            d_R = {{res_p[0], res_p[3], res_p[5], 0.0, 0.0, 0.0},
        {res_p[3], res_p[1], res_p[4], 0.0, 0.0, 0.0},
        {res_p[4], res_p[5], res_p[2], 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, pow(measures_ecef_vel_sd_ms, 2.0), 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, pow(measures_ecef_vel_sd_ms, 2.0), 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, pow(measures_ecef_vel_sd_ms, 2.0)}};

        d_static = false;
        }
    // system covariance matrix (static)
    d_Q = {{pow(system_ecef_pos_sd_m, 2.0), 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, pow(system_ecef_pos_sd_m, 2.0), 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, pow(system_ecef_pos_sd_m, 2.0), 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, pow(system_ecef_vel_sd_ms, 2.0), 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, pow(system_ecef_vel_sd_ms, 2.0), 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, pow(system_ecef_vel_sd_ms, 2.0)}};

    // initial Kalman covariance matrix
    d_P_old_old = {{pow(system_ecef_pos_sd_m, 2.0), 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, pow(system_ecef_pos_sd_m, 2.0), 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, pow(system_ecef_pos_sd_m, 2.0), 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, pow(system_ecef_vel_sd_ms, 2.0), 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, pow(system_ecef_vel_sd_ms, 2.0), 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, pow(system_ecef_vel_sd_ms, 2.0)}};

    // states: position ecef [m], velocity ecef [m/s]
    d_x_old_old = arma::zeros(6);
    d_x_old_old.subvec(0, 2) = p;
    d_x_old_old.subvec(3, 5) = v;

    d_initialized = true;

    DLOG(INFO) << "Ti: " << Ti;
    DLOG(INFO) << "F: " << d_F;
    DLOG(INFO) << "H: " << d_H;
    DLOG(INFO) << "R: " << d_R;
    DLOG(INFO) << "Q: " << d_Q;
    DLOG(INFO) << "P: " << d_P_old_old;
    DLOG(INFO) << "x: " << d_x_old_old;
}


bool Pvt_Kf::is_initialized() const
{
    return d_initialized;
}


void Pvt_Kf::reset_Kf()
{
    d_initialized = false;
}


void Pvt_Kf::run_Kf(const arma::vec& p, const arma::vec& v, const arma::vec& res_p)
{
    if (d_initialized)
        {
            // Kalman loop
            // Prediction
            d_x_new_old = d_F * d_x_old_old;
            d_P_new_old = d_F * d_P_old_old * d_F.t() + d_Q;

            // Measurement update
            try
                {
                    if(!d_static){
                        // Measurement residuals update
                        d_R(0, 0) = res_p[0];
                        d_R(0, 1) = res_p[3];
                        d_R(0, 2) = res_p[5];
                        d_R(1, 0) = res_p[3];
                        d_R(1, 1) = res_p[1];
                        d_R(1, 2) = res_p[4];
                        d_R(2, 0) = res_p[5];
                        d_R(2, 1) = res_p[4];
                        d_R(2, 2) = res_p[2];
                    }
                    // Measurement update
                    arma::vec z = arma::join_cols(p, v);
                    arma::mat K = d_P_new_old * d_H.t() * arma::inv(d_H * d_P_new_old * d_H.t() + d_R);  // Kalman gain

                    d_x_new_new = d_x_new_old + K * (z - d_H * d_x_new_old);
                    arma::mat A = (arma::eye(6, 6) - K * d_H);
                    d_P_new_new = A * d_P_new_old * A.t() + K * d_R * K.t();

                    // prepare data for next KF epoch
                    d_x_old_old = d_x_new_new;
                    d_P_old_old = d_P_new_new;
                }
            catch (...)
                {
                    d_x_new_new = d_x_new_old;
                    this->reset_Kf();
                }
        }
}


void Pvt_Kf::get_pv_Kf(arma::vec& p, arma::vec& v) const
{
    if (d_initialized)
        {
            p = d_x_new_new.subvec(0, 2);
            v = d_x_new_new.subvec(3, 5);
        }
}
