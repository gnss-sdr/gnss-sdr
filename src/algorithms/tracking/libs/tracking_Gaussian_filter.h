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
#include "tracking_models.h"
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
    void set_ncov_process(float ev_1, float ev_2, float ev_3);
    void set_ncov_measurement(float ev_1, float ev_2);
    void set_params(float ev_1, float ev_2, float pdi_carr);
    void set_state(float x_1, float x_2, float x_3);
    void set_state(float x_1, float x_2, float x_3, float x_4);
    void set_state_cov(float P_x_1, float P_x_2, float P_x_3);
    void set_state_cov(float P_x_1, float P_x_2, float P_x_3, float P_x_4);

protected:
    arma::vec state;     /* state vector */
    arma::mat state_cov;    /* state error covariance matrix */
    arma::mat ncov_process;     /* model error covariance matrix */
    arma::mat ncov_measurement; /* measurement error covariance matrix */
};

template <class NonlinearFilter, class OutputType1, class OutputType2>
class TrackingNonlinearFilter : public TrackingGaussianFilter
{
public:
    arma::vec get_carrier_nco(const OutputType2 meas_in);
    
    void set_transition_model(ModelFunction<OutputType1>* ft) { func_transition = ft; };
    void set_measurement_model(ModelFunction<OutputType2>* fm) { func_measurement = fm; };
    void set_model(ModelFunction<OutputType1>* ft, ModelFunction<OutputType2>* fm) {
        set_transition_model(ft);
        set_measurement_model(fm);
    };

private:
    ModelFunction<OutputType1>* func_transition;
    ModelFunction<OutputType2>* func_measurement;

    NonlinearFilter GaussFilt;

};

/* Template definitions */

/*
 * Gaussian filter operating on correlator outputs
 * Req Input in [Hz/Ti]
 * The output is in [Hz/s].
 */
template <class NonlinearFilter, class OutputType1, class OutputType2>
arma::vec TrackingNonlinearFilter<NonlinearFilter,OutputType1,OutputType2>::get_carrier_nco(const OutputType2 meas_in)
{
    arma::vec state_pred;
    arma::mat state_cov_pred;

    // State Update
# if 0
    std::cout << "state:";
    state.print();
    std::cout << std::endl;
    std::cout << "state_cov:";
    state_cov.print();
    std::cout << std::endl;
    std::cout << "ncov_process:";
    ncov_process.print();
    std::cout << std::endl;

    std::cout << "BEGIN PREDICT" << std::endl;
#endif
    GaussFilt.predict_sequential(state, state_cov, func_transition, ncov_process);
    state_pred = GaussFilt.get_x_pred();
    state_cov_pred = GaussFilt.get_P_x_pred();

    // Measurement Update
    OutputType2 measurement = meas_in;
    /*
    measurement(0) = Prompt.real();
    measurement(1) = Prompt.imag();

    innovation(0) = measurement(0) - std::sqrt(d_carrier_power_snv) * cos(state_pred(0));
    innovation(1) = measurement(1) - std::sqrt(d_carrier_power_snv) * sin(state_pred(0));
    */

    GaussFilt.update_sequential(measurement, state_pred, state_cov_pred, func_measurement, ncov_measurement);
    state = GaussFilt.get_x_pred();
    state_cov = GaussFilt.get_P_x_pred();

    // Return the new carrier estimation
    return state;
}

#endif
