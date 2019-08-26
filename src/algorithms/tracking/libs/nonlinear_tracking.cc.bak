/*!
 * \file nonlinear_tracking.cc
 * \brief Interface of a library for nonlinear tracking algorithms
 *
 * CubatureFilter implements the functionality of the Cubature Kalman
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

#include "nonlinear_tracking.h"

GaussianFilter::GaussianFilter()
{
    int nx = 1;
    x_pred_out = arma::zeros(nx, 1);
    P_x_pred_out = arma::eye(nx, nx) * (nx + 1);

    x_est = x_pred_out;
    P_x_est = P_x_pred_out;
}


GaussianFilter::GaussianFilter(int nx)
{
    x_pred_out = arma::zeros(nx, 1);
    P_x_pred_out = arma::eye(nx, nx) * (nx + 1);

    x_est = x_pred_out;
    P_x_est = P_x_pred_out;
}


GaussianFilter::GaussianFilter(const arma::vec& x_pred_0, const arma::mat& P_x_pred_0)
{
    x_pred_out = x_pred_0;
    P_x_pred_out = P_x_pred_0;

    x_est = x_pred_out;
    P_x_est = P_x_pred_out;
}


GaussianFilter::~GaussianFilter() = default;


void GaussianFilter::initialize(const arma::mat& x_pred_0, const arma::mat& P_x_pred_0)
{
    x_pred_out = x_pred_0;
    P_x_pred_out = P_x_pred_0;

    x_est = x_pred_out;
    P_x_est = P_x_pred_out;
}

arma::mat GaussianFilter::get_x_pred() const
{
    return x_pred_out;
}


arma::mat GaussianFilter::get_P_x_pred() const
{
    return P_x_pred_out;
}


arma::mat GaussianFilter::get_x_est() const
{
    return x_est;
}


arma::mat GaussianFilter::get_P_x_est() const
{
    return P_x_est;
}
