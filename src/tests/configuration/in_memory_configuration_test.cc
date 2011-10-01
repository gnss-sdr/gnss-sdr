
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class implements a Unit Test for the class InMemoryConfiguration.
 *
 */

#include <gtest/gtest.h>

#include "configuration_interface.h"
#include "in_memory_configuration.h"

TEST(InMemoryConfiguration, IsPresent) {
	InMemoryConfiguration *configuration = new InMemoryConfiguration();

	EXPECT_FALSE(configuration->is_present("NotThere"));

	configuration->set_property("NotThere", "Yes!");
	EXPECT_TRUE(configuration->is_present("NotThere"));

	delete configuration;
}

TEST(InMemoryConfiguration, StoreAndRetrieve) {
	ConfigurationInterface *configuration = new InMemoryConfiguration();

	((InMemoryConfiguration*)configuration)->set_property("Foo.property1", "value");
	std::string default_value = "default_value";
	std::string value = configuration->property("Foo.property1", default_value);

	EXPECT_STREQ("value", value.c_str());

	delete configuration;
}

TEST(InMemoryConfiguration, NoStoringAndRetrieve) {
	ConfigurationInterface *configuration = new InMemoryConfiguration();
	std::string default_value = "default_value";
	std::string value = configuration->property("Foo.property1", default_value);

	EXPECT_STREQ("default_value", value.c_str());

	delete configuration;
}

TEST(InMemoryConfiguration, RetrieveBool) {
	ConfigurationInterface *configuration = new InMemoryConfiguration();

	((InMemoryConfiguration*)configuration)->set_property("Foo.property1", "true");
	bool value = configuration->property("Foo.property1", false);

	EXPECT_EQ(true, value);

	delete configuration;
}

TEST(InMemoryConfiguration, RetrieveBoolFail) {
	ConfigurationInterface *configuration = new InMemoryConfiguration();

	((InMemoryConfiguration*)configuration)->set_property("Foo.property1", "tru");
	bool value = configuration->property("Foo.property1", false);

	EXPECT_EQ(false, value);

	delete configuration;
}

TEST(InMemoryConfiguration, RetrieveBoolNoDefine) {
	ConfigurationInterface *configuration = new InMemoryConfiguration();

	bool value = configuration->property("Foo.property1", false);

	EXPECT_EQ(false, value);

	delete configuration;
}

TEST(InMemoryConfiguration, RetrieveSizeT) {
	ConfigurationInterface *configuration = new InMemoryConfiguration();

	((InMemoryConfiguration*)configuration)->set_property("Foo.property1", "8");
	unsigned int value = configuration->property("Foo.property1", 4);

	EXPECT_EQ(8, value);

	delete configuration;
}

TEST(InMemoryConfiguration, RetrieveSizeTFail) {
	ConfigurationInterface *configuration = new InMemoryConfiguration();

	((InMemoryConfiguration*)configuration)->set_property("Foo.property1", "true");
	unsigned int value = configuration->property("Foo.property1", 4);

	EXPECT_EQ(4, value);

	delete configuration;
}

TEST(InMemoryConfiguration, RetrieveSizeTNoDefine) {
	ConfigurationInterface *configuration = new InMemoryConfiguration();

	unsigned int value = configuration->property("Foo.property1", 4);

	EXPECT_EQ(4, value);

	delete configuration;
}
