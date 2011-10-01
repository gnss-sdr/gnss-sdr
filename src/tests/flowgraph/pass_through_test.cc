
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class implements a Unit Test for the class NoConditioningSignalConditioner.
 *
 */

#include <gtest/gtest.h>
#include <gr_block.h>
#include <stdexcept>

#include "pass_through.h"
#include "in_memory_configuration.h"

TEST(PassThrough, Instantiate) {

	InMemoryConfiguration* config = new InMemoryConfiguration();

	config->set_property("Test.item_type", "gr_complex");
	config->set_property("Test.vector_size", "2");

	PassThrough *signal_conditioner = new PassThrough(config, "Test", 1, 1);

	EXPECT_STREQ("gr_complex", signal_conditioner->item_type().c_str());
	EXPECT_EQ(2, signal_conditioner->vector_size());

	delete signal_conditioner;
}