/*!
 * \file pass_through_test.cc
 * \brief  This file implements tests for the Pass_Through block
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
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


#include "in_memory_configuration.h"
#include "pass_through.h"
#include <gtest/gtest.h>


TEST(PassThroughTest, Instantiate)
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Test.item_type", "gr_complex");
    std::shared_ptr<Pass_Through> signal_conditioner = std::make_shared<Pass_Through>(config.get(), "Test", 1, 1);
    EXPECT_STREQ("gr_complex", signal_conditioner->item_type().c_str());
}
