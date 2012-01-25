/*!
 * \file pass_through_test.cc
 * \brief  This file implements tests for the Pass_Through block
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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


#include <gtest/gtest.h>
#include "pass_through.h"
#include "in_memory_configuration.h"



TEST(Pass_Through_Test, Instantiate)
{
    InMemoryConfiguration* config = new InMemoryConfiguration();
    config->set_property("Test.item_type", "gr_complex");
    config->set_property("Test.vector_size", "2");
    Pass_Through *signal_conditioner = new Pass_Through(config, "Test", 1, 1);

    EXPECT_STREQ("gr_complex", signal_conditioner->item_type().c_str());
    unsigned int expected2 = 2;
    EXPECT_EQ(expected2, signal_conditioner->vector_size());

    delete signal_conditioner;
}
