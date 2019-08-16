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

void TrackingGaussianFilter::set_ncov_measurement(float ev_1, float ev_2)
{
    ncov_measurement = arma::zeros(2, 2);
    ncov_measurement(0, 0) = ev_1;
    ncov_measurement(1, 1) = ev_2;
}

void TrackingGaussianFilter::set_ncov_process(float ev_1, float ev_2, float ev_3)
{
    ncov_process = arma::zeros(3, 3);
    ncov_process(0, 0) = ev_1;
    ncov_process(1, 1) = ev_2;
    ncov_process(2, 2) = ev_3;
}

void TrackingGaussianFilter::set_params(float ev_1, float ev_2, float pdi_carr)
{
    set_ncov_measurement(ev_1, ev_2);
    set_ncov_process(std::pow(pdi_carr, 6), std::pow(pdi_carr, 4), std::pow(pdi_carr, 2));
}

void TrackingGaussianFilter::set_state(float x_1, float x_2, float x_3)
{
    state = arma::zeros(3,1);
    state(0) = x_1;
    state(1) = x_2;
    state(2) = x_3;
}

void TrackingGaussianFilter::set_state_cov(float P_x_1, float P_x_2, float P_x_3)
{
    state_cov = arma::zeros(3,3);
    state_cov(0, 0) = P_x_1;
    state_cov(1, 1) = P_x_2;
    state_cov(2, 2) = P_x_3;
}

void TrackingGaussianFilter::set_state(float x_1, float x_2, float x_3, float x_4)
{
    state = arma::zeros(4,1);
    state(0) = x_1;
    state(1) = x_2;
    state(2) = x_3;
    state(4) = x_4;
}

void TrackingGaussianFilter::set_state_cov(float P_x_1, float P_x_2, float P_x_3, float P_x_4)
{
    state_cov = arma::zeros(4,4);
    state_cov(0, 0) = P_x_1;
    state_cov(1, 1) = P_x_2;
    state_cov(2, 2) = P_x_3;
    state_cov(3, 3) = P_x_4;
}
