/*!
 * \file bayesian_estimation.h
 * \brief Interface of a library with Bayesian noise statistic estimation
 *
 * Bayesian_estimator is a Bayesian estimator which attempts to estimate
 * the properties of a stochastic process based on a sequence of
 * discrete samples of the sequence.
 *
 * [1] TODO: Refs
 *
 * \authors <ul>
 *          <li> Gerald LaMountain, 2018. gerald(at)ece.neu.edu
 *          <li> Jordi Vila-Valls 2018. jvila(at)cttc.es
 *          </ul>
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#include <gnuradio/gr_complex.h>
#include <armadillo>

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
    Bayesian_estimator(arma::vec mu_prior_0, int kappa_prior_0, int nu_prior_0, arma::mat Psi_prior_0);
    ~Bayesian_estimator();

    void update_sequential(arma::vec data);
    void update_sequential(arma::vec data, arma::vec mu_prior_0, int kappa_prior_0, int nu_prior_0, arma::mat Psi_prior_0);

    arma::vec get_mu_est();
    arma::mat get_Psi_est();

private:

    arma::vec mu_est;
    arma::mat Psi_est;
    
    arma::vec mu_prior;
    int kappa_prior;
    int nu_prior;
    arma::mat Psi_prior;
    
};

#endif
