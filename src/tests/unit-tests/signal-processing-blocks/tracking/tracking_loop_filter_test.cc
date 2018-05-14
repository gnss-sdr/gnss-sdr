/*!
 * \file tracking_loop_filter_test.cc
 * \brief  This file implements tests for the general loop filter
 * \author Cillian O'Driscoll, 2015. cillian.odriscoll(at)gmail.com
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

#include "tracking_loop_filter.h"
#include <gtest/gtest.h>

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
    for (unsigned int i = 0; i < sample_data.size(); ++i)
        {
            result = theFilter.apply(sample_data[i]);
            EXPECT_FLOAT_EQ(result, sample_data[i] * g1);
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
    for (unsigned int i = 0; i < sample_data.size(); ++i)
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
    for (unsigned int i = 0; i < sample_data.size(); ++i)
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
    for (unsigned int i = 0; i < sample_data.size(); ++i)
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
    for (unsigned int i = 0; i < sample_data.size(); ++i)
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
    for (unsigned int i = 0; i < sample_data.size(); ++i)
        {
            result = theFilter.apply(sample_data[i]);
            EXPECT_NEAR(result, expected_out[i], 1e-4);
        }
}
