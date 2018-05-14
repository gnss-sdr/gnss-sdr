/*!
 * \file in_memory_configuration_test.cc
 * \brief  This file implements tests for the in_memory_configuration.
 * \author Carles Fernandez-Prades, 2013. cfernandez(at)cttc.es
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

#include "configuration_interface.h"
#include "in_memory_configuration.h"

TEST(InMemoryConfiguration, IsPresent)
{
    //std::shared_ptr<InMemoryConfiguration> configuration = std::make_shared<InMemoryConfiguration>();
    std::unique_ptr<InMemoryConfiguration> configuration(new InMemoryConfiguration);
    EXPECT_FALSE(configuration->is_present("NotThere"));
    configuration->set_property("NotThere", "Yes!");
    EXPECT_TRUE(configuration->is_present("NotThere"));
}

TEST(InMemoryConfiguration, StoreAndRetrieve)
{
    //std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    std::unique_ptr<ConfigurationInterface> configuration(new InMemoryConfiguration);
    configuration->set_property("Foo.property1", "value");
    std::string default_value = "default_value";
    std::string value = configuration->property("Foo.property1", default_value);
    EXPECT_STREQ("value", value.c_str());
}

TEST(InMemoryConfiguration, NoStoringAndRetrieve)
{
    //std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    std::unique_ptr<ConfigurationInterface> configuration(new InMemoryConfiguration);
    std::string default_value = "default_value";
    std::string value = configuration->property("Foo.property1", default_value);
    EXPECT_STREQ("default_value", value.c_str());
}

TEST(InMemoryConfiguration, RetrieveBool)
{
    //std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    std::unique_ptr<ConfigurationInterface> configuration(new InMemoryConfiguration);
    configuration->set_property("Foo.property1", "true");
    bool value = configuration->property("Foo.property1", false);
    bool expectedtrue = true;
    EXPECT_EQ(expectedtrue, value);
}

TEST(InMemoryConfiguration, RetrieveBoolFail)
{
    //std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    std::unique_ptr<ConfigurationInterface> configuration(new InMemoryConfiguration);
    configuration->set_property("Foo.property1", "tru");
    bool value = configuration->property("Foo.property1", false);
    bool expectedfalse = false;
    EXPECT_EQ(expectedfalse, value);
}

TEST(InMemoryConfiguration, RetrieveBoolNoDefine)
{
    //std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    std::unique_ptr<ConfigurationInterface> configuration(new InMemoryConfiguration);
    bool value = configuration->property("Foo.property1", false);
    bool expectedfalse = false;
    EXPECT_EQ(expectedfalse, value);
}

TEST(InMemoryConfiguration, RetrieveSizeT)
{
    //std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    std::unique_ptr<ConfigurationInterface> configuration(new InMemoryConfiguration);
    configuration->set_property("Foo.property1", "8");
    unsigned int value = configuration->property("Foo.property1", 4);
    unsigned int expected8 = 8;
    EXPECT_EQ(expected8, value);
}

TEST(InMemoryConfiguration, RetrieveSizeTFail)
{
    //std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    std::unique_ptr<ConfigurationInterface> configuration(new InMemoryConfiguration);
    configuration->set_property("Foo.property1", "true");
    unsigned int value = configuration->property("Foo.property1", 4);
    unsigned int expected4 = 4;
    EXPECT_EQ(expected4, value);
}

TEST(InMemoryConfiguration, RetrieveSizeTNoDefine)
{
    //std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    std::unique_ptr<ConfigurationInterface> configuration(new InMemoryConfiguration);
    unsigned int value = configuration->property("Foo.property1", 4);
    unsigned int expected4 = 4;
    EXPECT_EQ(expected4, value);
}
