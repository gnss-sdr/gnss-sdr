
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class implements a Unit Test for the class FileConfiguration.
 *
 */

#include <string>
#include <iostream>

#include <gtest/gtest.h>
#include "file_configuration.h"

TEST(FileConfiguration, OverridedProperties) {
	ConfigurationInterface *configuration = new FileConfiguration("./src/tests/data/config_file_sample.txt");
	std::string default_value = "default_value";
	std::string value = configuration->property("NotThere", default_value);

	EXPECT_STREQ("default_value", value.c_str());

	configuration->set_property("NotThere", "Yes!");
	value = configuration->property("NotThere", default_value);

	EXPECT_STREQ("Yes!", value.c_str());

	delete configuration;
}

TEST(FileConfiguration, LoadFromNonExistentFile) {

	ConfigurationInterface *configuration = new FileConfiguration("./i_dont_exist.conf");
	std::string default_value = "default_value";
	std::string value = configuration->property("whatever.whatever", default_value);

	EXPECT_STREQ("default_value", value.c_str());

	delete configuration;
}

TEST(FileConfiguration, PropertyDoesNotExist) {
	ConfigurationInterface *configuration = new FileConfiguration("./src/tests/data/config_file_sample.txt");
	std::string default_value = "default_value";
	std::string value = configuration->property("whatever.whatever", default_value);

	EXPECT_STREQ("default_value", value.c_str());

	delete configuration;
}
