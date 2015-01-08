/*!
 * \file file_output_filter_test.cc
 * \brief  This class implements a Unit Test for the class FileOutputFilter.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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

#include <gtest/gtest.h>
#include "file_output_filter.h"
#include "in_memory_configuration.h"

TEST(FileOutputFilter, Instantiate)
{
    std::shared_ptr<InMemoryConfiguration> config = std::make_shared<InMemoryConfiguration>();
    std::string path = std::string(TEST_PATH);
    std::string file = path + "data/output.dat";
    config->set_property("Test.filename", file);
    config->set_property("Test.item_type", "float");
    std::unique_ptr<FileOutputFilter> output_filter(new FileOutputFilter(config.get(), "Test", 1, 0));
    int res = 0;
    if (output_filter) res = 1;
    ASSERT_EQ(1, res);
}
