/*!
 * \file in_memory_configuration_test.cc
 * \brief  This file implements tests for the in_memory_configuration.
 * \author Carles Fernandez-Prades, 2013. cfernandez(at)cttc.es
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

#include "configuration_interface.h"
#include "gnss_sdr_make_unique.h"
#include "in_memory_configuration.h"
#include <utility>

TEST(InMemoryConfiguration, IsPresent)
{
    // std::shared_ptr<InMemoryConfiguration> configuration = std::make_shared<InMemoryConfiguration>();
    auto configuration = std::make_unique<InMemoryConfiguration>();
    EXPECT_FALSE(configuration->is_present("NotThere"));
    configuration->set_property("NotThere", "Yes!");
    EXPECT_TRUE(configuration->is_present("NotThere"));
}


TEST(InMemoryConfiguration, StoreAndRetrieve)
{
    // std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    auto configuration = std::make_unique<InMemoryConfiguration>();
    configuration->set_property("Foo.property1", "value");
    std::string default_value = "default_value";
    std::string value = configuration->property("Foo.property1", std::move(default_value));
    EXPECT_STREQ("value", value.c_str());
}


TEST(InMemoryConfiguration, NoStoringAndRetrieve)
{
    // std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    auto configuration = std::make_unique<InMemoryConfiguration>();
    std::string default_value = "default_value";
    std::string value = configuration->property("Foo.property1", std::move(default_value));
    EXPECT_STREQ("default_value", value.c_str());
}


TEST(InMemoryConfiguration, RetrieveBool)
{
    // std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    auto configuration = std::make_unique<InMemoryConfiguration>();
    configuration->set_property("Foo.property1", "true");
    bool value = configuration->property("Foo.property1", false);
    bool expectedtrue = true;
    EXPECT_EQ(expectedtrue, value);
}


TEST(InMemoryConfiguration, RetrieveBoolFail)
{
    // std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    auto configuration = std::make_unique<InMemoryConfiguration>();
    configuration->set_property("Foo.property1", "tru");
    bool value = configuration->property("Foo.property1", false);
    bool expectedfalse = false;
    EXPECT_EQ(expectedfalse, value);
}


TEST(InMemoryConfiguration, RetrieveBoolNoDefine)
{
    // std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    auto configuration = std::make_unique<InMemoryConfiguration>();
    bool value = configuration->property("Foo.property1", false);
    bool expectedfalse = false;
    EXPECT_EQ(expectedfalse, value);
}


TEST(InMemoryConfiguration, RetrieveSizeT)
{
    // std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    auto configuration = std::make_unique<InMemoryConfiguration>();
    configuration->set_property("Foo.property1", "8");
    unsigned int value = configuration->property("Foo.property1", 4);
    unsigned int expected8 = 8;
    EXPECT_EQ(expected8, value);
}


TEST(InMemoryConfiguration, RetrieveSizeTFail)
{
    // std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    auto configuration = std::make_unique<InMemoryConfiguration>();
    configuration->set_property("Foo.property1", "true");
    unsigned int value = configuration->property("Foo.property1", 4);
    unsigned int expected4 = 4;
    EXPECT_EQ(expected4, value);
}


TEST(InMemoryConfiguration, RetrieveSizeTNoDefine)
{
    // std::shared_ptr<ConfigurationInterface> configuration = std::make_shared<InMemoryConfiguration>();
    auto configuration = std::make_unique<InMemoryConfiguration>();
    unsigned int value = configuration->property("Foo.property1", 4);
    unsigned int expected4 = 4;
    EXPECT_EQ(expected4, value);
}
