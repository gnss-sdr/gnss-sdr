/*!
 * \file beidou_bdgim_test.cc
 * \brief Smoke tests for BDGIM ionospheric model
 * \author Wenhao Ou, 2026. ouwh(at)mail2.sysu.edu.cn
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
#include "MATH_CONSTANTS.h"
#include "beidou_bdgim.h"
#include "gnss_frequencies.h"
#include <gtest/gtest.h>
#include <cmath>

TEST(BeidouBdgimTest, NonZeroDelayAtMidLatitude)
{
    double alpha[9] = {10.0, 1.0, 0.5, -0.5, 0.2, 0.1, -0.1, 0.05, -0.05};
    const double lat = 40.0 * D2R;
    const double lon = 116.0 * D2R;
    const double az = 45.0 * D2R;
    const double el = 40.0 * D2R;
    const double mjd = 58000.0;
    const double delay = beidou_bdgim_delay_m(mjd, lat, lon, az, el, alpha, FREQ1);
    EXPECT_TRUE(std::isfinite(delay));
    EXPECT_NE(delay, 0.0);
}

TEST(BeidouBdgimTest, ScalesWithFrequencySquared)
{
    double alpha[9] = {8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const double lat = 30.0 * D2R;
    const double lon = 120.0 * D2R;
    const double az = 0.0;
    const double el = 60.0 * D2R;
    const double mjd = 59000.0;
    const double d1 = beidou_bdgim_delay_m(mjd, lat, lon, az, el, alpha, FREQ1);
    const double d_b1i = beidou_bdgim_delay_m(mjd, lat, lon, az, el, alpha, FREQ1_BDS);
    ASSERT_NE(d1, 0.0);
    const double ratio = d_b1i / d1;
    const double expected = (FREQ1 * FREQ1) / (FREQ1_BDS * FREQ1_BDS);
    EXPECT_NEAR(ratio, expected, 1.0e-6);
}

TEST(BeidouBdgimTest, ZeroElevationReturnsZero)
{
    double alpha[9] = {5.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    const double delay = beidou_bdgim_delay_m(58000.0, 0.0, 0.0, 0.0, 0.0, alpha, FREQ1);
    EXPECT_DOUBLE_EQ(delay, 0.0);
}
