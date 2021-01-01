/*!
 * \file file_configuration_test.cc
 * \brief  This file implements tests for the file_configuration.
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
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


#include "file_configuration.h"
#include "gnss_sdr_make_unique.h"
#include <string>


TEST(FileConfigurationTest, OverridedProperties)
{
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "data/config_file_sample.txt";
    // std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<FileConfiguration>(filename);
    std::unique_ptr<ConfigurationInterface> configuration = std::make_unique<FileConfiguration>(filename);
    std::string default_value = "default_value";
    std::string value = configuration->property("NotThere", default_value);
    EXPECT_STREQ("default_value", value.c_str());
    configuration->set_property("NotThere", "Yes!");
    value = configuration->property("NotThere", default_value);
    EXPECT_STREQ("Yes!", value.c_str());
}


TEST(FileConfigurationTest, LoadFromNonExistentFile)
{
    std::unique_ptr<ConfigurationInterface> configuration = std::make_unique<FileConfiguration>("./i_dont_exist.conf");
    std::string default_value = "default_value";
    std::string value = configuration->property("whatever.whatever", default_value);
    EXPECT_STREQ("default_value", value.c_str());
}


TEST(FileConfigurationTest, PropertyDoesNotExist)
{
    std::string path = std::string(TEST_PATH);
    std::string filename = path + "data/config_file_sample.txt";
    std::unique_ptr<ConfigurationInterface> configuration = std::make_unique<FileConfiguration>(filename);
    std::string default_value = "default_value";
    std::string value = configuration->property("whatever.whatever", default_value);
    EXPECT_STREQ("default_value", value.c_str());
}
