/*!
 * \file beidou_b1c_dll_pll_veml_tracking_test.cc
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "beidou_b1c_dll_pll_veml_tracking.h"
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
