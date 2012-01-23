/*!
 * \file file_configuration_test.cc
 * \brief  This file implements tests for the ControlMessageFactory.
 * \author Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
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


#include <string>
#include <iostream>
#include <gtest/gtest.h>
#include "file_configuration.h"




TEST(File_Configuration_Test, OverridedProperties)
{
	ConfigurationInterface *configuration = new FileConfiguration("./data/config_file_sample.txt");
	std::string default_value = "default_value";
	std::string value = configuration->property("NotThere", default_value);

	EXPECT_STREQ("default_value", value.c_str());

	configuration->set_property("NotThere", "Yes!");
	value = configuration->property("NotThere", default_value);

	EXPECT_STREQ("Yes!", value.c_str());

	delete configuration;
}



TEST(File_Configuration_Test, LoadFromNonExistentFile)
{

	ConfigurationInterface *configuration = new FileConfiguration("./i_dont_exist.conf");
	std::string default_value = "default_value";
	std::string value = configuration->property("whatever.whatever", default_value);

	EXPECT_STREQ("default_value", value.c_str());

	delete configuration;
}




TEST(File_Configuration_Test, PropertyDoesNotExist)
{
	ConfigurationInterface *configuration = new FileConfiguration("./data/config_file_sample.txt");
	std::string default_value = "default_value";
	std::string value = configuration->property("whatever.whatever", default_value);

	EXPECT_STREQ("default_value", value.c_str());

	delete configuration;
}
