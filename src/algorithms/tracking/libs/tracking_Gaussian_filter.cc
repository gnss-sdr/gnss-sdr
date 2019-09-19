/*!
 * \file tracking_Gaussian_filter.cc
 * \brief Implementation of a nonlinear Gaussian filter for tracking carrier loop.
 *
 * Class that implements a nonlinear Gaussian for tracking carrier loop using
 * CKF or UKF implementations.
 *
 * [1] I Arasaratnam and S Haykin. Cubature kalman filters. IEEE
 * Transactions on Automatic Control, 54(6):1254–1269,2009.
 *
 * \authors <ul>
 *          <li>  Gerald LaMountain, 2018. gerald(at)ece.neu.edu
 *          <li> Jordi Vila-Valls 2019. jvila(at)cttc.es
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

#include "tracking_Gaussian_filter.h"
#include <utility>

void TrackingGaussianFilter::set_ncov_measurement(arma::mat ncov)
{
    d_ncov_measurement = std::move(ncov);
}

void TrackingGaussianFilter::set_ncov_process(arma::mat ncov)
{
    d_ncov_process = std::move(ncov);
}

void TrackingGaussianFilter::set_state(arma::vec state)
{
    d_state = std::move(state);
}

void TrackingGaussianFilter::set_state_cov(arma::mat state_cov)
{
    d_state_cov = std::move(state_cov);
}

void TrackingGaussianFilter::set_params(arma::vec state, arma::mat state_cov, arma::mat p_ncov, arma::mat m_ncov)
{
    set_ncov_process(std::move(p_ncov));
    set_ncov_measurement(std::move(m_ncov));
    set_state(std::move(state));
    set_state_cov(std::move(state_cov));
}
