/*!
 * \file tracking_Gaussian_filter.h
 * \brief Implementation of a Gaussian filter for tracking carrier loop.
 *
 * Class that implements a Gaussian for tracking carrier loop using
 * CKF or UKF implementations.
 *
 * [1] I Arasaratnam and S Haykin. Cubature kalman filters. IEEE
 * Transactions on Automatic Control, 54(6):1254–1269,2009.
 *
 * \authors <ul>
 *          <li> Gerald LaMountain, 2018. gerald(at)ece.neu.edu
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

#ifndef GNSS_SDR_TRACKING_GAUSSIAN_FILTER_H_
#define GNSS_SDR_TRACKING_GAUSSIAN_FILTER_H_

#include <armadillo>
#include <gnuradio/gr_complex.h>
// #include "tracking_models.h"
#include "nonlinear_tracking.h"
#include "MATH_CONSTANTS.h"

/*!
 * \brief This class implements a standard Gaussian filter for carrier tracking.
 *
 * The algorithm is described in:
 * [1] I Arasaratnam and S Haykin. Cubature kalman filters. IEEE
 * Transactions on Automatic Control, 54(6):1254–1269,2009.
 */
class TrackingGaussianFilter
{
public:
    // TrackingGaussianFilter();
    // TrackingGaussianFilter(float pdi_carr, uint32_t kf_order, bool kf_extended);
    // ~TrackingGaussianFilter();

    // void initialize();
    //void initialize(float pdi_carar, arma::colvec x_ini, arma::mat P_x_ini, arma::mat Q_ini, arma::mat R_ini, ModelFunction* f_t, ModelFunction* f_m);

    //void set_ncov_process(float err_variance);
    void set_ncov_process(float ev_1, float ev_2, float ev_3);

   // void set_ncov_measurement(float err_variance);
    void set_ncov_measurement(float ev_1, float ev_2);

    void set_params(float ev_1, float ev_2, float pdi_carr);

    // void set_KF_x(float x_1, float x_2);
    void set_state(float x_1, float x_2, float x_3);

    //void set_KF_P_x(float P_x_1, float P_x_2);
    void set_state_cov(float P_x_1, float P_x_2, float P_x_3);

    // void setup_filter(float pdi_carr, uint32_t kf_order, bool kf_extended);

protected:

    //float d_pdi_carr = 0.0;
    // uint32_t d_kf_order = 2U;
    // bool d_kf_extended = false;

    // Kalman filter parameters
    // arma::colvec kf_x_ini; /* initial state vector */
    // arma::mat kf_P_x_ini;  /* initial state error covariance matrix */

    arma::colvec state;     /* state vector */
    arma::mat state_cov;    /* state error covariance matrix */

    // arma::colvec kf_x_pre; /* predicted state vector */
    // arma::mat kf_P_x_pre;  /* Predicted state error covariance matrix */

    // arma::mat kf_P_y;      /* innovation covariance matrix */

    // arma::mat kf_F;        /* state transition matrix */
    // arma::mat kf_H;        /* system matrix */

    arma::mat ncov_process;     /* model error covariance matrix */
    arma::mat ncov_measurement; /* measurement error covariance matrix */

    // arma::colvec kf_z;     /* measurement vector */
    // arma::colvec kf_y;     /* innovation vector */
    // arma::mat kf_K;        /* Kalman gain matrix */

};

template <class NonlinearFilter>
class TrackingNonlinearFilter : public TrackingGaussianFilter
{
public:
    //carr_nco get_carrier_nco(float PLL_discriminator, float pll_bw_hz);
    std::pair<float, float> get_carrier_nco(const gr_complex Prompt);

    void set_transition_model(ModelFunction* ft) { func_transition = ft; };
    void set_measurement_model(ModelFunction* fm) { func_measurement = fm; };
    void set_model(ModelFunction* ft, ModelFunction* fm) {
        set_transition_model(ft);
        set_measurement_model(fm);
    };

private:
    ModelFunction* func_transition;
    ModelFunction* func_measurement;

    NonlinearFilter GaussFilt;
    // CubatureFilter GaussFilt;

};
#include "tracking_Gaussian_filter.tcc"

#endif
