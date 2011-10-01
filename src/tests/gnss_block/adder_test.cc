
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class implements a Unit Test for the class Adder
 *
 */

#include <gtest/gtest.h>

#include "adder.h"
#include "in_memory_configuration.h"

TEST(Adder, Instantiate) {

	InMemoryConfiguration* config = new InMemoryConfiguration();

	config->set_property("Test.item_type", "gr_complex");

	Adder *adder = new Adder(config, "PVT", 4, 1);

	delete adder;
}