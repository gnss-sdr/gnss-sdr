/*!
 * \file beidou_b1c_dll_pll_veml_tracking_test.cc
 * \brief Unit tests for BeiDou B1C DLL+PLL VEML tracking via DllPllTrackingAdapter
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
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "tracking_interface.h"
#include <gtest/gtest.h>
#include <memory>

TEST(BeidouB1cDllPllVemlTrackingTest, Instantiate)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Tracking_1D.implementation", "BEIDOU_B1C_DLL_PLL_VEML_Tracking");
    configuration->set_property("Tracking_1D.item_type", "gr_complex");
    configuration->set_property("GNSS-SDR.internal_fs_sps", "2046000");
    auto factory = std::make_unique<GNSSBlockFactory>();
    std::shared_ptr<GNSSBlockInterface> trk_ = factory->GetBlock(configuration.get(), "Tracking_1D", 1, 1);
    auto tracking = std::dynamic_pointer_cast<TrackingInterface>(trk_);
    ASSERT_NE(tracking, nullptr);
    EXPECT_STREQ("BEIDOU_B1C_DLL_PLL_VEML_Tracking", tracking->implementation().c_str());
}
