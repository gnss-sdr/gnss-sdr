/*!
 * \file bayesian_estimation_test.cc
 * \brief  This file implements feasibility test for the BCE library.
 * \author Gerald LaMountain, 2018. gerald(at)ece.neu.edu
 *
 *
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

#include "bayesian_estimation.h"
#include <armadillo>
#include <gtest/gtest.h>
#include <random>

#define BAYESIAN_TEST_N_TRIALS 100
#define BAYESIAN_TEST_ITER 10000

TEST(BayesianEstimationPositivityTest, BayesianPositivityTest)
{
    Bayesian_estimator bayes;
    arma::vec bayes_mu = arma::zeros(1, 1);
    int bayes_nu = 0;
    int bayes_kappa = 0;
    arma::mat bayes_Psi = arma::ones(1, 1);
    arma::vec input = arma::zeros(1, 1);

    //--- Perform initializations ------------------------------

    std::random_device r;
    std::default_random_engine e1(r());
    std::normal_distribution<float> normal_dist(0, 5);

    for (int k = 0; k < BAYESIAN_TEST_N_TRIALS; k++)
        {
            bayes.init(bayes_mu, bayes_kappa, bayes_nu, bayes_Psi);

            for (int n = 0; n < BAYESIAN_TEST_ITER; n++)
                {
                    input(0) = static_cast<double>(normal_dist(e1));
                    bayes.update_sequential(input);

                    arma::mat output = bayes.get_Psi_est();
                    double output0 = output(0, 0);
                    ASSERT_EQ(output0 > 0.0, true);
                }
        }
}
