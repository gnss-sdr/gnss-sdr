/*!
 * \file bayesian_estimation.h
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

#ifndef GNSS_SDR_BAYESIAN_ESTIMATION_H_
#define GNSS_SDR_BAYESIAN_ESTIMATION_H_

#if ARMA_NO_BOUND_CHECKING
#define ARMA_NO_DEBUG 1
#endif

#include <armadillo>
#include <gnuradio/gr_complex.h>

/*! \brief Bayesian_estimator is an estimator of noise characteristics (i.e. mean, covariance)
 *
 * Bayesian_estimator is an estimator which performs estimation of noise characteristics from
 * a sequence of identically and independently distributed (IID) samples of a stationary
 * stochastic process by way of Bayesian inference using conjugate priors. The posterior
 * distribution is assumed to be Gaussian with mean \mathbf{\mu} and covariance \hat{\mathbf{C}},
 * which has a conjugate prior given by a normal-inverse-Wishart distribution with paramemters
 * \mathbf{\mu}_{0}, \kappa_{0}, \nu_{0}, and \mathbf{\Psi}.
 *
 * [1] TODO: Ref1
 *
 */

class Bayesian_estimator
{
public:
    Bayesian_estimator();
    Bayesian_estimator(int ny);
    Bayesian_estimator(const arma::vec& mu_prior_0, int kappa_prior_0, int nu_prior_0, const arma::mat& Psi_prior_0);
    ~Bayesian_estimator() = default;

    void init(const arma::mat& mu_prior_0, int kappa_prior_0, int nu_prior_0, const arma::mat& Psi_prior_0);

    void update_sequential(const arma::vec& data);
    void update_sequential(const arma::vec& data, const arma::vec& mu_prior_0, int kappa_prior_0, int nu_prior_0, const arma::mat& Psi_prior_0);

    arma::mat get_mu_est() const;
    arma::mat get_Psi_est() const;

private:
    arma::vec mu_est;
    arma::mat Psi_est;

    arma::vec mu_prior;
    int kappa_prior;
    int nu_prior;
    arma::mat Psi_prior;
};

#endif
