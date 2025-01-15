/*!
 * \file bayesian_estimation.cc
 * \brief Interface of a library with Bayesian noise statistic estimation
 *
 * Bayesian_estimator is a Bayesian estimator which attempts to estimate
 * the properties of a stochastic process based on a sequence of
 * discrete samples of the sequence.
 *
 * [1]: LaMountain, Gerald, Vilà-Valls, Jordi, Closas, Pau, "Bayesian
 * Covariance Estimation for Kalman Filter based Digital Carrier
 * Synchronization," Proceedings of the 31st International Technical Meeting
 * of the Satellite Division of The Institute of Navigation
 * (ION GNSS+ 2018), Miami, Florida, September 2018, pp. 3575-3586.
 * https://doi.org/10.33012/2018.15911
 *
 * \authors <ul>
 *          <li> Gerald LaMountain, 2018. gerald(at)ece.neu.edu
 *          <li> Jordi Vila-Valls 2018. jvila(at)cttc.es
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

#include "bayesian_estimation.h"
#include <utility>

Bayesian_estimator::Bayesian_estimator()
    : kappa_prior(0),
      nu_prior(0)
{
    mu_prior = arma::zeros(1, 1);
    mu_est = mu_prior;
    Psi_prior = arma::eye(1, 1) * (nu_prior + 1 + 1);
    Psi_est = Psi_prior;
}


Bayesian_estimator::Bayesian_estimator(int ny)
    : kappa_prior(0),
      nu_prior(0)
{
    mu_prior = arma::zeros(ny, 1);
    mu_est = mu_prior;

    Psi_prior = arma::eye(ny, ny) * (nu_prior + ny + 1);
    Psi_est = Psi_prior;
}


Bayesian_estimator::Bayesian_estimator(const arma::vec& mu_prior_0,
    int kappa_prior_0,
    int nu_prior_0,
    const arma::mat& Psi_prior_0)
    : mu_prior(mu_prior_0),
      Psi_prior(Psi_prior_0),
      kappa_prior(kappa_prior_0),
      nu_prior(nu_prior_0)
{
    mu_est = mu_prior;
    Psi_est = Psi_prior;
}


void Bayesian_estimator::init(const arma::mat& mu_prior_0, int kappa_prior_0, int nu_prior_0, const arma::mat& Psi_prior_0)
{
    mu_prior = mu_prior_0;
    kappa_prior = kappa_prior_0;
    nu_prior = nu_prior_0;
    Psi_prior = Psi_prior_0;

    mu_est = mu_prior;
    Psi_est = Psi_prior;
}


/*
 * Perform Bayesian noise estimation using the normal-inverse-Wishart priors stored in
 * the class structure, and update the priors according to the computed posteriors
 */
void Bayesian_estimator::update_sequential(const arma::vec& data)
{
    int K = data.n_cols;
    int ny = data.n_rows;

    if (mu_prior.is_empty())
        {
            mu_prior = arma::zeros(ny, 1);
        }

    if (Psi_prior.is_empty())
        {
            Psi_prior = arma::zeros(ny, ny);
        }

    arma::vec y_mean = arma::mean(data, 1);
    arma::mat Psi_N = arma::zeros(ny, ny);

    for (int kk = 0; kk < K; kk++)
        {
            Psi_N = Psi_N + (data.col(kk) - y_mean) * ((data.col(kk) - y_mean).t());
        }

    arma::vec mu_posterior = (kappa_prior * mu_prior + K * y_mean) / (kappa_prior + K);
    int kappa_posterior = kappa_prior + K;
    int nu_posterior = nu_prior + K;
    arma::mat Psi_posterior = Psi_prior + Psi_N + (static_cast<float>(kappa_prior) * static_cast<float>(K)) / (static_cast<float>(kappa_prior) + static_cast<float>(K)) * (y_mean - mu_prior) * ((y_mean - mu_prior).t());

    mu_est = mu_posterior;
    if ((nu_posterior - ny - 1) > 0)
        {
            Psi_est = Psi_posterior / (nu_posterior - ny - 1);
        }
    else
        {
            Psi_est = Psi_posterior / (nu_posterior + ny + 1);
        }

    mu_prior = std::move(mu_posterior);
    kappa_prior = kappa_posterior;
    nu_prior = nu_posterior;
    Psi_prior = std::move(Psi_posterior);
}


/*
 * Perform Bayesian noise estimation using a new set of normal-inverse-Wishart priors
 * and update the priors according to the computed posteriors
 */
void Bayesian_estimator::update_sequential(const arma::vec& data, const arma::vec& mu_prior_0, int kappa_prior_0, int nu_prior_0, const arma::mat& Psi_prior_0)
{
    int K = data.n_cols;
    int ny = data.n_rows;

    arma::vec y_mean = arma::mean(data, 1);
    arma::mat Psi_N = arma::zeros(ny, ny);

    for (int kk = 0; kk < K; kk++)
        {
            Psi_N = Psi_N + (data.col(kk) - y_mean) * ((data.col(kk) - y_mean).t());
        }

    arma::vec mu_posterior = (kappa_prior_0 * mu_prior_0 + K * y_mean) / (kappa_prior_0 + K);
    int kappa_posterior = kappa_prior_0 + K;
    int nu_posterior = nu_prior_0 + K;
    arma::mat Psi_posterior = Psi_prior_0 + Psi_N + (Psi_prior_0 * static_cast<double>(K)) / (static_cast<double>(kappa_prior_0) + static_cast<double>(K)) * (y_mean - mu_prior_0) * ((y_mean - mu_prior_0).t());

    mu_est = mu_posterior;
    if ((nu_posterior - ny - 1) > 0)
        {
            Psi_est = Psi_posterior / (nu_posterior - ny - 1);
        }
    else
        {
            Psi_est = Psi_posterior / (nu_posterior + ny + 1);
        }

    mu_prior = std::move(mu_posterior);
    kappa_prior = kappa_posterior;
    nu_prior = nu_posterior;
    Psi_prior = std::move(Psi_posterior);
}


arma::mat Bayesian_estimator::get_mu_est() const
{
    return mu_est;
}


arma::mat Bayesian_estimator::get_Psi_est() const
{
    return Psi_est;
}
