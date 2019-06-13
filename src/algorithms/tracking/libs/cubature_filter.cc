/*!
 * \file cubature_filter.cc
 * \brief Interface of a library with Bayesian noise statistic estimation
 *
 * Cubature_Filter implements the functionality of the Cubature Kalman
 * Filter, which uses multidimensional cubature rules to estimate the
 * time evolution of a nonlinear system.
 *
 * [1] I Arasaratnam and S Haykin. Cubature kalman filters. IEEE 
 * Transactions on Automatic Control, 54(6):1254–1269,2009.
 *
 * \authors <ul>
 *          <li> Gerald LaMountain, 2019. gerald(at)ece.neu.edu
 *          <li> Jordi Vila-Valls 2019. jvila(at)cttc.es
 *          </ul>
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "cubature_filter.h"


Cubature_filter::Cubature_filter()
{
    int nx = 1;
    x_pred_out = arma::zeros(nx, 1);
    P_x_pred_out = arma::eye(nx, nx) * (nx + 1);

    x_est = x_pred_out;
    P_x_est = P_x_pred_out;
}

Cubature_filter::Cubature_filter(int nx)
{
    x_pred_out = arma::zeros(nx, 1);
    P_x_pred_out = arma::eye(nx, nx) * (nx + 1);

    x_est = x_pred_out;
    P_x_est = P_x_pred_out;
}

Cubature_filter::Cubature_filter(const arma::vec& x_pred_0, const arma::mat& P_x_pred_0)
{
    x_pred_out = x_pred_0;
    P_x_pred_out = P_x_pred_0;

    x_est = x_pred_out;
    P_x_est = P_x_pred_out;
}

Cubature_filter::~Cubature_filter() = default;

void Cubature_filter::initialize(const arma::mat& x_pred_0, const arma::mat& P_x_pred_0)
{
    x_pred_out = x_pred_0;
    P_x_pred_out = P_x_pred_0;

    x_est = x_pred_out;
    P_x_est = P_x_pred_out;
}


/*
 * Perform the prediction step of the cubature Kalman filter
 */
void Cubature_filter::predict_sequential(const arma::vec& x_post, const arma::mat& P_x_post, Model_Function* transition_fcn, const arma::mat& noise_covariance)
{
    // Compute number of cubature points
    int nx = x_post.n_elem;
    int np = 2 * nx;

    // Generator Matrix
    arma::mat gen_one = arma::join_horiz(arma::eye(nx,nx),-1.0*arma::eye(nx,nx));

    // Initialize predicted mean and covariance
    arma::vec x_pred = arma::zeros(nx,1);
    arma::mat P_x_pred = arma::zeros(nx,nx);

    // Factorize posterior covariance
    arma::mat Sm_post = arma::chol(P_x_post, "lower");
    
    // Propagate and evaluate cubature points
    arma::vec Xi_post;
    arma::vec Xi_pred;

    for (uint8_t i = 0; i < np; i++)
    {
        Xi_post = Sm_post * (std::sqrt(((float) np) / 2.0) * gen_one.col(i)) + x_post;
        Xi_pred = (*transition_fcn)(Xi_post);
       
        x_pred = x_pred + Xi_pred;
        P_x_pred = P_x_pred + Xi_pred*Xi_pred.t();
    }
    
    // Estimate predicted state and error covariance
    x_pred = x_pred / ((float) np);
    P_x_pred = P_x_pred / ((float) np) - x_pred*x_pred.t() + noise_covariance;

    // Store predicted state and error covariance
    x_pred_out = x_pred;
    P_x_pred_out = P_x_pred;
}

/*
 * Perform the update step of the cubature Kalman filter
 */
void Cubature_filter::update_sequential(const arma::vec& z_upd, const arma::vec& x_pred, const arma::mat& P_x_pred, Model_Function* measurement_fcn, const arma::mat& noise_covariance)
{
    // Compute number of cubature points
    int nx = x_pred.n_elem;
    int nz = z_upd.n_elem;
    int np = 2 * nx;

    // Generator Matrix
    arma::mat gen_one = arma::join_horiz(arma::eye(nx,nx),-1.0*arma::eye(nx,nx));

    // Evaluate predicted measurement and covariances
    arma::mat z_pred = arma::zeros(nz,1);
    arma::mat P_zz_pred = arma::zeros(nz,nz);
    arma::mat P_xz_pred = arma::zeros(nx,nz);

    // Factorize predicted covariance
    arma::mat Sm_pred = arma::chol(P_x_pred, "lower");

    // Propagate and evaluate cubature points
    arma::vec Xi_pred;
    arma::vec Zi_pred;
    for (uint8_t i = 0; i < np; i++)
    {
        Xi_pred = Sm_pred * (std::sqrt(((float) np) / 2.0) * gen_one.col(i)) + x_pred;
        Zi_pred = (*measurement_fcn)(Xi_pred);
        
        z_pred = z_pred + Zi_pred;
        P_zz_pred = P_zz_pred + Zi_pred*Zi_pred.t();
        P_xz_pred = P_xz_pred + Xi_pred*Zi_pred.t();
    }

    // Estimate measurement covariance and cross covariances
    z_pred = z_pred / ((float) np);
    P_zz_pred = P_zz_pred / ((float) np) - z_pred*z_pred.t() + noise_covariance;
    P_xz_pred = P_xz_pred / ((float) np) - x_pred*z_pred.t();

    // Estimate cubature Kalman gain
    arma::mat W_k = P_xz_pred*arma::inv(P_zz_pred);

    // Estimate and store the updated state and error covariance
    x_est = x_pred + W_k*(z_upd - z_pred);
    P_x_est = P_x_pred - W_k*P_zz_pred*W_k.t();
}

arma::mat Cubature_filter::get_x_pred() const
{
    return x_pred_out;
}

arma::mat Cubature_filter::get_P_x_pred() const
{
    return P_x_pred_out;
}

arma::mat Cubature_filter::get_x_est() const
{
    return x_est;
}

arma::mat Cubature_filter::get_P_x_est() const
{
    return P_x_est;
}
