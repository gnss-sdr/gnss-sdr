
#ifndef GNSS_SDR_TRACKING_GAUSSIAN_FILTER_TCC_
#define GNSS_SDR_TRACKING_GAUSSIAN_FILTER_TCC_

#include <armadillo>
#include <gnuradio/gr_complex.h>
#include "MATH_CONSTANTS.h"

/*
 * Gaussian filter operating on correlator outputs
 * Req Input in [Hz/Ti]
 * The output is in [Hz/s].
 */
template <class NonlinearFilter>
// carr_nco TrackingNonlinearFilter<NonlinearFilter>::get_carrier_nco(gr_complex Prompt)
std::pair<float, float> TrackingNonlinearFilter<NonlinearFilter>::get_carrier_nco(const gr_complex Prompt)
{
    arma::colvec state_pred;
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
    arma::colvec measurement = arma::zeros(2, 1);
    measurement(0) = Prompt.real();
    measurement(1) = Prompt.imag();

    /*
    arma::colvec innovation = arma::zeros(2, 1);
    innovation(0) = measurement(0) - std::sqrt(d_carrier_power_snv) * cos(state_pred(0));
    innovation(1) = measurement(1) - std::sqrt(d_carrier_power_snv) * sin(state_pred(0));
    */

    GaussFilt.update_sequential(measurement, state_pred, state_cov_pred, func_measurement, ncov_measurement);
    state = GaussFilt.get_x_pred();
    state_cov = GaussFilt.get_P_x_pred();

    // Set a new carrier estimation to the NCO
    return std::make_pair(static_cast<float>(state(0)), static_cast<float>(state(1)));
}

#endif
