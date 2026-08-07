/*!
 * \file beidou_b1c_pcps_acquisition_test.cc
 * \brief Unit tests for BeiDou B1C PCPS acquisition via PcpsAcquisitionAdapter
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
#include "acquisition_interface.h"
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include <gtest/gtest.h>
#include <memory>

TEST(BeidouB1cPcpsAcquisitionTest, Instantiate)
{
    auto configuration = std::make_shared<InMemoryConfiguration>();
    configuration->set_property("Acquisition_1D.implementation", "BEIDOU_B1C_PCPS_Ambiguous_Acquisition");
    configuration->set_property("Acquisition_1D.item_type", "gr_complex");
    auto factory = std::make_unique<GNSSBlockFactory>();
    std::shared_ptr<GNSSBlockInterface> acq_ = factory->GetBlock(configuration.get(), "Acquisition_1D", 1, 0);
    auto acquisition = std::dynamic_pointer_cast<AcquisitionInterface>(acq_);
    ASSERT_NE(acquisition, nullptr);
    EXPECT_STREQ("BEIDOU_B1C_PCPS_Ambiguous_Acquisition", acquisition->implementation().c_str());
}
