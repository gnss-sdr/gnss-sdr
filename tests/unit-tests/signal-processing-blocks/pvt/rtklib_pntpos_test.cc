/*!
 * \file rtklib_pntpos_test.cc
 * \brief Unit tests for the RTKLIB single point positioning with many observations
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

#include "rtklib.h"
#include "rtklib_pntpos.h"
#include <gtest/gtest.h>
#include <cmath>
#include <vector>


TEST(RtklibPntposTest, ValsolChiSquareThresholdCoversManyDegreesOfFreedom)
{
    // The chi-square table has 300 entries, the threshold is approximated
    // beyond it. The exact thresholds (alpha=0.001) for 120, 200 and 400
    // degrees of freedom are 173.6, 267.5 and 493.1.
    constexpr int NS = 10;
    constexpr int NX = 8;
    std::vector<double> azel(2 * NS);
    std::vector<int> vsat(NS, 1);
    for (int i = 0; i < NS; i++)
        {
            azel[2 * i] = 2.0 * M_PI * i / NS;
            azel[2 * i + 1] = (i % 2 == 0 ? 0.6 : 1.2);
        }
    prcopt_t opt{};
    opt.elmin = 5.0 * M_PI / 180.0;
    opt.maxgdop = 30.0;

    const int dofs[] = {120, 200, 400};
    const double thresholds[] = {173.6, 267.5, 493.1};
    for (int k = 0; k < 3; k++)
        {
            SCOPED_TRACE(dofs[k]);
            const int nv = dofs[k] + NX;
            std::vector<double> v(nv, 0.0);
            char msg[128] = {0};

            v[0] = std::sqrt(0.97 * thresholds[k]);
            EXPECT_EQ(valsol(azel.data(), vsat.data(), NS, &opt, v.data(), nv, NX, msg), 1) << msg;
            v[0] = std::sqrt(1.03 * thresholds[k]);
            EXPECT_EQ(valsol(azel.data(), vsat.data(), NS, &opt, v.data(), nv, NX, msg), 0);
        }
}
