/*!
 * \file cubature_filter.cc
 * \brief Interface of a library for nonlinear tracking algorithms
 *
 * Cubature_Filter implements the functionality of the Cubature Kalman
 * Filter, which uses multidimensional cubature rules to estimate the
 * time evolution of a nonlinear system. UnscentedFilter implements
 * an Unscented Kalman Filter which uses Unscented Transform rules to
 * perform a similar estimation.
 *
 * [1] I Arasaratnam and S Haykin. Cubature kalman filters. IEEE
 * Transactions on Automatic Control, 54(6):1254–1269,2009.
 *
 * \authors <ul>
 *          <li> Gerald LaMountain, 2019. gerald(at)ece.neu.edu
 *          <li> Jordi Vila-Valls 2019. jvila(at)cttc.es
 *          </ul>
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "nonlinear_tracking.h"
#include <utility>

/***************** CUBATURE KALMAN FILTER *****************/

CubatureFilter::CubatureFilter()
    : x_pred_out(arma::zeros(1, 1)),
      P_x_pred_out(arma::eye(1, 1) * (1 + 1)),
      x_est(x_pred_out),
      P_x_est(P_x_pred_out)
{
}


CubatureFilter::CubatureFilter(int nx)
    : x_pred_out(arma::zeros(nx, 1)),
      P_x_pred_out(arma::eye(nx, nx) * (nx + 1)),
      x_est(x_pred_out),
      P_x_est(P_x_pred_out)
{
}


CubatureFilter::CubatureFilter(const arma::vec& x_pred_0, const arma::mat& P_x_pred_0)
    : x_pred_out(x_pred_0),
      P_x_pred_out(P_x_pred_0),
      x_est(x_pred_out),
      P_x_est(P_x_pred_out)
{
}


void CubatureFilter::initialize(const arma::mat& x_pred_0, const arma::mat& P_x_pred_0)
{
    x_pred_out = x_pred_0;
    P_x_pred_out = P_x_pred_0;

    x_est = x_pred_out;
    P_x_est = P_x_pred_out;
}


/*
 * Perform the prediction step of the cubature Kalman filter
 */
void CubatureFilter::predict_sequential(const arma::vec& x_post, const arma::mat& P_x_post, ModelFunction* transition_fcn, const arma::mat& noise_covariance)
{
    // Compute number of cubature points
    int nx = x_post.n_elem;
    int np = 2 * nx;

    // Generator Matrix
    arma::mat gen_one = arma::join_horiz(arma::eye(nx, nx), -1.0 * arma::eye(nx, nx));

    // Initialize predicted mean and covariance
    arma::vec x_pred = arma::zeros(nx, 1);
    arma::mat P_x_pred = arma::zeros(nx, nx);

    // Factorize posterior covariance
    arma::mat Sm_post = arma::chol(P_x_post, "lower");

    // Propagate and evaluate cubature points
    arma::vec Xi_post;
    arma::vec Xi_pred;

    for (int i = 0; i < np; i++)
        {
            Xi_post = Sm_post * (std::sqrt(static_cast<float>(np) / 2.0) * gen_one.col(i)) + x_post;
            Xi_pred = (*transition_fcn)(Xi_post);

            x_pred = x_pred + Xi_pred;
            P_x_pred = P_x_pred + Xi_pred * Xi_pred.t();
        }

    // Compute predicted mean and error covariance
    x_pred = x_pred / static_cast<float>(np);
    P_x_pred = P_x_pred / static_cast<float>(np) - x_pred * x_pred.t() + noise_covariance;

    // Store predicted mean and error covariance
    x_pred_out = std::move(x_pred);
    P_x_pred_out = std::move(P_x_pred);
}


/*
 * Perform the update step of the cubature Kalman filter
 */
void CubatureFilter::update_sequential(const arma::vec& z_upd, const arma::vec& x_pred, const arma::mat& P_x_pred, ModelFunction* measurement_fcn, const arma::mat& noise_covariance)
{
    // Compute number of cubature points
    int nx = x_pred.n_elem;
    int nz = z_upd.n_elem;
    int np = 2 * nx;

    // Generator Matrix
    arma::mat gen_one = arma::join_horiz(arma::eye(nx, nx), -1.0 * arma::eye(nx, nx));

    // Initialize estimated predicted measurement and covariances
    arma::mat z_pred = arma::zeros(nz, 1);
    arma::mat P_zz_pred = arma::zeros(nz, nz);
    arma::mat P_xz_pred = arma::zeros(nx, nz);

    // Factorize predicted covariance
    arma::mat Sm_pred = arma::chol(P_x_pred, "lower");

    // Propagate and evaluate cubature points
    arma::vec Xi_pred;
    arma::vec Zi_pred;
    for (int i = 0; i < np; i++)
        {
            Xi_pred = Sm_pred * (std::sqrt(static_cast<float>(np) / 2.0) * gen_one.col(i)) + x_pred;
            Zi_pred = (*measurement_fcn)(Xi_pred);

            z_pred = z_pred + Zi_pred;
            P_zz_pred = P_zz_pred + Zi_pred * Zi_pred.t();
            P_xz_pred = P_xz_pred + Xi_pred * Zi_pred.t();
        }

    // Compute measurement mean, covariance and cross covariance
    z_pred = z_pred / static_cast<float>(np);
    P_zz_pred = P_zz_pred / static_cast<float>(np) - z_pred * z_pred.t() + noise_covariance;
    P_xz_pred = P_xz_pred / static_cast<float>(np) - x_pred * z_pred.t();

    // Compute cubature Kalman gain
    arma::mat W_k = P_xz_pred * arma::inv(P_zz_pred);

    // Compute and store the updated mean and error covariance
    x_est = x_pred + W_k * (z_upd - z_pred);
    P_x_est = P_x_pred - W_k * P_zz_pred * W_k.t();
}


arma::mat CubatureFilter::get_x_pred() const
{
    return x_pred_out;
}


arma::mat CubatureFilter::get_P_x_pred() const
{
    return P_x_pred_out;
}


arma::mat CubatureFilter::get_x_est() const
{
    return x_est;
}


arma::mat CubatureFilter::get_P_x_est() const
{
    return P_x_est;
}
/***************** END CUBATURE KALMAN FILTER *****************/


/***************** UNSCENTED KALMAN FILTER *****************/

UnscentedFilter::UnscentedFilter()
    : x_pred_out(arma::zeros(1, 1)),
      P_x_pred_out(arma::eye(1, 1) * (1 + 1)),
      x_est(x_pred_out),
      P_x_est(P_x_pred_out)
{
}


UnscentedFilter::UnscentedFilter(int nx)
    : x_pred_out(arma::zeros(nx, 1)),
      P_x_pred_out(arma::eye(nx, nx) * (nx + 1)),
      x_est(x_pred_out),
      P_x_est(P_x_pred_out)
{
}


UnscentedFilter::UnscentedFilter(const arma::vec& x_pred_0, const arma::mat& P_x_pred_0)
    : x_pred_out(x_pred_0),
      P_x_pred_out(P_x_pred_0),
      x_est(x_pred_out),
      P_x_est(P_x_pred_out)
{
}


void UnscentedFilter::initialize(const arma::mat& x_pred_0, const arma::mat& P_x_pred_0)
{
    x_pred_out = x_pred_0;
    P_x_pred_out = P_x_pred_0;

    x_est = x_pred_out;
    P_x_est = P_x_pred_out;
}


/*
 * Perform the prediction step of the Unscented Kalman filter
 */
void UnscentedFilter::predict_sequential(const arma::vec& x_post, const arma::mat& P_x_post, ModelFunction* transition_fcn, const arma::mat& noise_covariance)
{
    // Compute number of sigma points
    int nx = x_post.n_elem;
    int np = 2 * nx + 1;

    float alpha = 0.001;
    float kappa = 0.0;
    float beta = 2.0;

    float lambda = std::pow(alpha, 2.0F) * (static_cast<float>(nx) + kappa) - static_cast<float>(nx);

    // Compute UT Weights
    float W0_m = lambda / (static_cast<float>(nx) + lambda);
    float W0_c = lambda / (static_cast<float>(nx) + lambda) + (1 - std::pow(alpha, 2.0F) + beta);
    float Wi_m = 1.0F / (2.0F * (static_cast<float>(nx) + lambda));

    // Propagate and evaluate sigma points
    arma::mat Xi_fact = arma::zeros(nx, nx);
    arma::mat Xi_post = arma::zeros(nx, np);
    arma::mat Xi_pred = arma::zeros(nx, np);

    Xi_post.col(0) = x_post;
    Xi_pred.col(0) = (*transition_fcn)(Xi_post.col(0));
    for (int i = 1; i <= nx; i++)
        {
            Xi_fact = std::sqrt(static_cast<float>(nx) + lambda) * arma::sqrtmat_sympd(P_x_post);
            Xi_post.col(i) = x_post + Xi_fact.col(i - 1);
            Xi_post.col(i + nx) = x_post - Xi_fact.col(i - 1);

            Xi_pred.col(i) = (*transition_fcn)(Xi_post.col(i));
            Xi_pred.col(i + nx) = (*transition_fcn)(Xi_post.col(i + nx));
        }

    // Compute predicted mean
    arma::vec x_pred = W0_m * Xi_pred.col(0) + Wi_m * arma::sum(Xi_pred.cols(1, np - 1), 1);

    // Compute predicted error covariance
    arma::mat P_x_pred = W0_c * ((Xi_pred.col(0) - x_pred) * (Xi_pred.col(0).t() - x_pred.t()));
    for (int i = 1; i < np; i++)
        {
            P_x_pred = P_x_pred + Wi_m * ((Xi_pred.col(i) - x_pred) * (Xi_pred.col(i).t() - x_pred.t()));
        }
    P_x_pred = P_x_pred + noise_covariance;

    // Store predicted mean and error covariance
    x_pred_out = std::move(x_pred);
    P_x_pred_out = std::move(P_x_pred);
}


/*
 * Perform the update step of the Unscented Kalman filter
 */
void UnscentedFilter::update_sequential(const arma::vec& z_upd, const arma::vec& x_pred, const arma::mat& P_x_pred, ModelFunction* measurement_fcn, const arma::mat& noise_covariance)
{
    // Compute number of sigma points
    int nx = x_pred.n_elem;
    int nz = z_upd.n_elem;
    int np = 2 * nx + 1;

    float alpha = 0.001;
    float kappa = 0.0;
    float beta = 2.0;

    float lambda = std::pow(alpha, 2.0F) * (static_cast<float>(nx) + kappa) - static_cast<float>(nx);

    // Compute UT Weights
    float W0_m = lambda / (static_cast<float>(nx) + lambda);
    float W0_c = lambda / (static_cast<float>(nx) + lambda) + (1.0F - std::pow(alpha, 2.0F) + beta);
    float Wi_m = 1.0F / (2.0F * (static_cast<float>(nx) + lambda));

    // Propagate and evaluate sigma points
    arma::mat Xi_fact = arma::zeros(nx, nx);
    arma::mat Xi_pred = arma::zeros(nx, np);
    arma::mat Zi_pred = arma::zeros(nz, np);

    Xi_pred.col(0) = x_pred;
    Zi_pred.col(0) = (*measurement_fcn)(Xi_pred.col(0));
    for (int i = 1; i <= nx; i++)
        {
            Xi_fact = std::sqrt(static_cast<float>(nx) + lambda) * arma::sqrtmat_sympd(P_x_pred);
            Xi_pred.col(i) = x_pred + Xi_fact.col(i - 1);
            Xi_pred.col(i + nx) = x_pred - Xi_fact.col(i - 1);

            Zi_pred.col(i) = (*measurement_fcn)(Xi_pred.col(i));
            Zi_pred.col(i + nx) = (*measurement_fcn)(Xi_pred.col(i + nx));
        }

    // Compute measurement mean
    arma::mat z_pred = W0_m * Zi_pred.col(0) + Wi_m * arma::sum(Zi_pred.cols(1, np - 1), 1);

    // Compute measurement covariance and cross covariance
    arma::mat P_zz_pred = W0_c * ((Zi_pred.col(0) - z_pred) * (Zi_pred.col(0).t() - z_pred.t()));
    arma::mat P_xz_pred = W0_c * ((Xi_pred.col(0) - x_pred) * (Zi_pred.col(0).t() - z_pred.t()));
    for (int i = 0; i < np; i++)
        {
            P_zz_pred = P_zz_pred + Wi_m * ((Zi_pred.col(i) - z_pred) * (Zi_pred.col(i).t() - z_pred.t()));
            P_xz_pred = P_xz_pred + Wi_m * ((Xi_pred.col(i) - x_pred) * (Zi_pred.col(i).t() - z_pred.t()));
        }
    P_zz_pred = P_zz_pred + noise_covariance;

    // Estimate cubature Kalman gain
    arma::mat W_k = P_xz_pred * arma::inv(P_zz_pred);

    // Estimate and store the updated mean and error covariance
    x_est = x_pred + W_k * (z_upd - z_pred);
    P_x_est = P_x_pred - W_k * P_zz_pred * W_k.t();
}


arma::mat UnscentedFilter::get_x_pred() const
{
    return x_pred_out;
}


arma::mat UnscentedFilter::get_P_x_pred() const
{
    return P_x_pred_out;
}


arma::mat UnscentedFilter::get_x_est() const
{
    return x_est;
}


arma::mat UnscentedFilter::get_P_x_est() const
{
    return P_x_est;
}

/***************** END UNSCENTED KALMAN FILTER *****************/
