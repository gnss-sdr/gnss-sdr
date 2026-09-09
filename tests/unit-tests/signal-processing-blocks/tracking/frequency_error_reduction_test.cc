/*!
 * \file frequency_error_reduction_test.cc
 * \brief  This file implements tests for the tracking frequency-error
 * reduction mechanism (Dll_Pll_Conf::f_error_step_num parsing and
 * dll_pll_veml_tracking::f_error_bin_multiplier()).
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

#include "dll_pll_conf.h"
#include "dll_pll_veml_tracking.h"
#include "in_memory_configuration.h"
#include <gtest/gtest.h>
#include <memory>
#include <vector>

TEST(FrequencyErrorReductionConfTest, DefaultIsDisabled)
{
    auto config = std::make_shared<InMemoryConfiguration>();
    Dll_Pll_Conf trk_params;
    trk_params.SetFromConfiguration(config.get(), "Tracking_1C");

    EXPECT_EQ(0U, trk_params.f_error_step_num);
}


TEST(FrequencyErrorReductionConfTest, OddValuePassesThroughUnchanged)
{
    auto config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Tracking_1C.f_error_step_num", "5");
    Dll_Pll_Conf trk_params;
    trk_params.SetFromConfiguration(config.get(), "Tracking_1C");

    EXPECT_EQ(5U, trk_params.f_error_step_num);
}


TEST(FrequencyErrorReductionConfTest, EvenValueGetsRoundedUpToOdd)
{
    auto config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Tracking_1C.f_error_step_num", "4");
    Dll_Pll_Conf trk_params;
    trk_params.SetFromConfiguration(config.get(), "Tracking_1C");

    EXPECT_EQ(5U, trk_params.f_error_step_num);
}


TEST(FrequencyErrorReductionConfTest, ZeroStaysZero)
{
    auto config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Tracking_1C.f_error_step_num", "0");
    Dll_Pll_Conf trk_params;
    trk_params.SetFromConfiguration(config.get(), "Tracking_1C");

    EXPECT_EQ(0U, trk_params.f_error_step_num);
}


TEST(FrequencyErrorReductionConfTest, AccumulationAndDopplerStepPassThrough)
{
    auto config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Tracking_1C.f_error_accumulation", "10");
    config->set_property("Tracking_1C.f_error_doppler_step", "125.0");
    Dll_Pll_Conf trk_params;
    trk_params.SetFromConfiguration(config.get(), "Tracking_1C");

    EXPECT_EQ(10U, trk_params.f_error_accumulation);
    EXPECT_DOUBLE_EQ(125.0, trk_params.f_error_doppler_step);
}


TEST(FrequencyErrorReductionBinMultiplierTest, CenterAndAlternatingOutward)
{
    EXPECT_DOUBLE_EQ(0.0, dll_pll_veml_tracking::f_error_bin_multiplier(0));
    EXPECT_DOUBLE_EQ(1.0, dll_pll_veml_tracking::f_error_bin_multiplier(1));
    EXPECT_DOUBLE_EQ(-1.0, dll_pll_veml_tracking::f_error_bin_multiplier(2));
    EXPECT_DOUBLE_EQ(2.0, dll_pll_veml_tracking::f_error_bin_multiplier(3));
    EXPECT_DOUBLE_EQ(-2.0, dll_pll_veml_tracking::f_error_bin_multiplier(4));
    EXPECT_DOUBLE_EQ(3.0, dll_pll_veml_tracking::f_error_bin_multiplier(5));
    EXPECT_DOUBLE_EQ(-3.0, dll_pll_veml_tracking::f_error_bin_multiplier(6));
}


TEST(FrequencyErrorReductionBinMultiplierTest, MatchesFiveBinLegacyPattern)
{
    // f_error_step_num = 5 must reproduce the original fixed 5-bin scan order:
    // +0, +step, -step, +2*step, -2*step
    const std::vector<double> expected = {0.0, 1.0, -1.0, 2.0, -2.0};
    for (uint32_t i = 0; i < expected.size(); i++)
        {
            EXPECT_DOUBLE_EQ(expected[i], dll_pll_veml_tracking::f_error_bin_multiplier(i));
        }
}
