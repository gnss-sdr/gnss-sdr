/*!
 * \file tracking_loop_filter_test.cc
 * \brief  This file implements tests for the general loop filter
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "tracking_loop_filter.h"
#include <gtest/gtest.h>
#include <cstddef>

TEST(TrackingLoopFilterTest, FirstOrderLoop)
{
    int loop_order = 1;
    float noise_bandwidth = 5.0;
    float update_interval = 0.001;
    bool include_last_integrator = false;

    Tracking_loop_filter theFilter(update_interval,
        noise_bandwidth,
        loop_order,
        include_last_integrator);

    EXPECT_EQ(theFilter.get_noise_bandwidth(), noise_bandwidth);
    EXPECT_EQ(theFilter.get_update_interval(), update_interval);
    EXPECT_EQ(theFilter.get_include_last_integrator(), include_last_integrator);
    EXPECT_EQ(theFilter.get_order(), loop_order);

    std::vector<float> sample_data = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    theFilter.initialize(0.0);

    float g1 = noise_bandwidth * 4.0;

    float result = 0.0;
    for (float i : sample_data)
        {
            result = theFilter.apply(i);
            EXPECT_FLOAT_EQ(result, i * g1);
        }
}


TEST(TrackingLoopFilterTest, FirstOrderLoopWithLastIntegrator)
{
    int loop_order = 1;
    float noise_bandwidth = 5.0;
    float update_interval = 0.001;
    bool include_last_integrator = true;

    Tracking_loop_filter theFilter(update_interval,
        noise_bandwidth,
        loop_order,
        include_last_integrator);

    EXPECT_EQ(theFilter.get_noise_bandwidth(), noise_bandwidth);
    EXPECT_EQ(theFilter.get_update_interval(), update_interval);
    EXPECT_EQ(theFilter.get_include_last_integrator(), include_last_integrator);
    EXPECT_EQ(theFilter.get_order(), loop_order);

    std::vector<float> sample_data = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    std::vector<float> expected_out = {0.0, 0.0, 0.01, 0.02, 0.02, 0.02};

    theFilter.initialize(0.0);

    float result = 0.0;
    for (size_t i = 0; i < sample_data.size(); ++i)
        {
            result = theFilter.apply(sample_data[i]);
            EXPECT_NEAR(result, expected_out[i], 1e-4);
        }
}


TEST(TrackingLoopFilterTest, SecondOrderLoop)
{
    int loop_order = 2;
    float noise_bandwidth = 5.0;
    float update_interval = 0.001;
    bool include_last_integrator = false;

    Tracking_loop_filter theFilter(update_interval,
        noise_bandwidth,
        loop_order,
        include_last_integrator);

    EXPECT_EQ(theFilter.get_noise_bandwidth(), noise_bandwidth);
    EXPECT_EQ(theFilter.get_update_interval(), update_interval);
    EXPECT_EQ(theFilter.get_include_last_integrator(), include_last_integrator);
    EXPECT_EQ(theFilter.get_order(), loop_order);

    std::vector<float> sample_data = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    std::vector<float> expected_out = {0.0, 0.0, 13.37778, 0.0889, 0.0889, 0.0889};

    theFilter.initialize(0.0);

    float result = 0.0;
    for (size_t i = 0; i < sample_data.size(); ++i)
        {
            result = theFilter.apply(sample_data[i]);
            EXPECT_NEAR(result, expected_out[i], 1e-4);
        }
}


TEST(TrackingLoopFilterTest, SecondOrderLoopWithLastIntegrator)
{
    int loop_order = 2;
    float noise_bandwidth = 5.0;
    float update_interval = 0.001;
    bool include_last_integrator = true;

    Tracking_loop_filter theFilter(update_interval,
        noise_bandwidth,
        loop_order,
        include_last_integrator);

    EXPECT_EQ(theFilter.get_noise_bandwidth(), noise_bandwidth);
    EXPECT_EQ(theFilter.get_update_interval(), update_interval);
    EXPECT_EQ(theFilter.get_include_last_integrator(), include_last_integrator);
    EXPECT_EQ(theFilter.get_order(), loop_order);

    std::vector<float> sample_data = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    std::vector<float> expected_out = {0.0, 0.0, 0.006689, 0.013422, 0.013511, 0.013600};

    theFilter.initialize(0.0);

    float result = 0.0;
    for (size_t i = 0; i < sample_data.size(); ++i)
        {
            result = theFilter.apply(sample_data[i]);
            EXPECT_NEAR(result, expected_out[i], 1e-4);
        }
}


TEST(TrackingLoopFilterTest, ThirdOrderLoop)
{
    int loop_order = 3;
    float noise_bandwidth = 5.0;
    float update_interval = 0.001;
    bool include_last_integrator = false;

    Tracking_loop_filter theFilter(update_interval,
        noise_bandwidth,
        loop_order,
        include_last_integrator);

    EXPECT_EQ(theFilter.get_noise_bandwidth(), noise_bandwidth);
    EXPECT_EQ(theFilter.get_update_interval(), update_interval);
    EXPECT_EQ(theFilter.get_include_last_integrator(), include_last_integrator);
    EXPECT_EQ(theFilter.get_order(), loop_order);

    std::vector<float> sample_data = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    std::vector<float> expected_out = {0.0, 0.0, 15.31877, 0.04494, 0.04520, 0.04546};

    theFilter.initialize(0.0);

    float result = 0.0;
    for (size_t i = 0; i < sample_data.size(); ++i)
        {
            result = theFilter.apply(sample_data[i]);
            EXPECT_NEAR(result, expected_out[i], 1e-4);
        }
}


TEST(TrackingLoopFilterTest, ThirdOrderLoopWithLastIntegrator)
{
    int loop_order = 3;
    float noise_bandwidth = 5.0;
    float update_interval = 0.001;
    bool include_last_integrator = true;

    Tracking_loop_filter theFilter(update_interval,
        noise_bandwidth,
        loop_order,
        include_last_integrator);

    EXPECT_EQ(theFilter.get_noise_bandwidth(), noise_bandwidth);
    EXPECT_EQ(theFilter.get_update_interval(), update_interval);
    EXPECT_EQ(theFilter.get_include_last_integrator(), include_last_integrator);
    EXPECT_EQ(theFilter.get_order(), loop_order);

    std::vector<float> sample_data = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    std::vector<float> expected_out = {0.0, 0.0, 0.007659, 0.015341, 0.015386, 0.015432};

    theFilter.initialize(0.0);

    float result = 0.0;
    for (size_t i = 0; i < sample_data.size(); ++i)
        {
            result = theFilter.apply(sample_data[i]);
            EXPECT_NEAR(result, expected_out[i], 1e-4);
        }
}
