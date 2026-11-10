/*!
 * \file rtklib_bancroft_test.cc
 * \brief Unit tests for the Bancroft initial position estimate
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include <armadillo>
#include <gtest/gtest.h>

// Internal helper in rtklib_pntpos.cc.
arma::vec rough_bancroft(const arma::mat &B_pass);


TEST(RtklibBancroftTest, UsesAllObservationsRegardlessOfTheirOrder)
{
    // Synthetic visible MEO satellites and pseudoranges in meters, generated
    // for ECEF (6378137, 0, 0), clock bias 30000 m, and noise below 3.1 m.
    // The nearly planar geometry admits two positive-range candidates. The
    // third observation alone favors the distant candidate; the full set does not.
    const arma::mat observations = {
        {19999879.402, 14917688.026, 10164696.121, 22644383.721},
        {20000063.265, 9184607.893, 16839508.814, 23556216.976},
        {20001024.178, -20139219.042, 6468834.238, 25189827.597},
        {20000723.908, -5232363.712, -19439789.660, 24337569.934},
        {19999817.812, 13327821.031, -10720272.516, 21895620.942},
        {19999565.912, 15820714.945, -679896.445, 20917811.449}};
    const arma::vec receiver = {6378137.0, 0.0, 0.0};
    const arma::vec initial_solution = rough_bancroft(observations);

    for (arma::uword last = 0; last < observations.n_rows; ++last)
        {
            SCOPED_TRACE(last);
            arma::mat reordered = observations;
            reordered.swap_rows(last, reordered.n_rows - 1);
            const arma::vec solution = rough_bancroft(reordered);
            ASSERT_TRUE(solution.is_finite());
            EXPECT_LT(arma::norm(solution.head(3) - receiver), 25.0);
            EXPECT_NEAR(solution(3), 30000.0, 15.0);
            EXPECT_LT(arma::norm(solution - initial_solution), 1e-3);
        }
}


TEST(RtklibBancroftTest, RecoversPositionFromFourNoiselessObservations)
{
    arma::mat observations = {
        {26560000.0, 0.0, 0.0, 0.0},
        {17560000.0, 18000000.0, 0.0, 0.0},
        {17560000.0, -9000000.0, 15588457.268, 0.0},
        {17560000.0, -9000000.0, -15588457.268, 0.0}};
    const arma::vec receiver = {6378137.0, 0.0, 0.0};
    constexpr double CLOCK_BIAS_M = 30000.0;

    for (arma::uword row = 0; row < observations.n_rows; ++row)
        {
            observations(row, 3) = arma::norm(observations.row(row).head(3).t() - receiver) + CLOCK_BIAS_M;
        }

    const arma::vec solution = rough_bancroft(observations);
    ASSERT_TRUE(solution.is_finite());
    EXPECT_LT(arma::norm(solution.head(3) - receiver), 1e-3);
    EXPECT_NEAR(solution(3), CLOCK_BIAS_M, 1e-3);
}
