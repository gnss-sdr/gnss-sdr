/*!
 * \file pass_through_test.cc
 * \brief  This file implements tests for the Pass_Through block
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "pass_through.h"
#include <gtest/gtest.h>
#include "in_memory_configuration.h"


TEST(PassThroughTest, Instantiate)
{
    std::shared_ptr<ConfigurationInterface> config = std::make_shared<InMemoryConfiguration>();
    config->set_property("Test.item_type", "gr_complex");
    std::shared_ptr<Pass_Through> signal_conditioner = std::make_shared<Pass_Through>(config.get(), "Test", 1, 1);
    EXPECT_STREQ("gr_complex", signal_conditioner->item_type().c_str());
}
