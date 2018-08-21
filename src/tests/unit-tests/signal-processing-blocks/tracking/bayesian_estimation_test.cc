/*!
 * \file bayesian_estimation_positivity_test.cc
 * \brief  This file implements timing tests for the Bayesian covariance estimator
 * \author Gerald LaMountain, 20168. gerald(at)ece.neu.edu
 *
 *
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

#include <chrono>
#include <complex>
#include <random>
#include <thread>
#include <armadillo>
#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <gnuradio/gr_complex.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include "bayesian_estimation.h"

#define BAYESIAN_TEST_N_TRIALS 100
#define BAYESIAN_TEST_ITER 10000

TEST(BayesianEstimationPositivityTest, BayesianPositivityTest)
{
    Bayesian_estimator bayes;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0);

    arma::vec bayes_mu = arma::zeros(1, 1);
    int bayes_nu = 0;
    int bayes_kappa = 0;
    arma::mat bayes_Psi = arma::ones(1, 1);

    arma::vec input  = arma::zeros(1, 1);
    arma::mat output = arma::ones(1, 1);

    //--- Perform initializations ------------------------------

    std::random_device r;
    std::default_random_engine e1(r());
    std::normal_distribution<float> normal_dist(0, 5);

    for (int k = 0; k < BAYESIAN_TEST_N_TRIALS; k++)
        {
            bayes.init(bayes_mu, bayes_kappa, bayes_nu, bayes_Psi);


            for (int n = 0; n < BAYESIAN_TEST_ITER; n++)
                {
                    input(0) = (double)(normal_dist(e1));
                    bayes.update_sequential(input);

                    output = bayes.get_Psi_est();
                    ASSERT_EQ(output(0) > 0, true);
                }
        }
}
