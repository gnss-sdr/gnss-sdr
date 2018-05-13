/*!
 * \file file_configuration_test.cc
 * \brief  This file implements tests for the file_configuration.
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
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


#include <string>
#include "file_configuration.h"


TEST(FileConfigurationTest, OverridedProperties)
{
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "data/config_file_sample.txt";
    //std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<FileConfiguration>(filename);
    std::unique_ptr<ConfigurationInterface> configuration(new FileConfiguration(filename));
    std::string default_value = "default_value";
    std::string value = configuration->property("NotThere", default_value);
    EXPECT_STREQ("default_value", value.c_str());
    configuration->set_property("NotThere", "Yes!");
    value = configuration->property("NotThere", default_value);
    EXPECT_STREQ("Yes!", value.c_str());
}


TEST(FileConfigurationTest, LoadFromNonExistentFile)
{
    std::unique_ptr<ConfigurationInterface> configuration(new FileConfiguration("./i_dont_exist.conf"));
    std::string default_value = "default_value";
    std::string value = configuration->property("whatever.whatever", default_value);
    EXPECT_STREQ("default_value", value.c_str());
}


TEST(FileConfigurationTest, PropertyDoesNotExist)
{
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "data/config_file_sample.txt";
    std::unique_ptr<ConfigurationInterface> configuration(new FileConfiguration(filename));
    std::string default_value = "default_value";
    std::string value = configuration->property("whatever.whatever", default_value);
    EXPECT_STREQ("default_value", value.c_str());
}
