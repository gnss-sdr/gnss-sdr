
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class implements a Unit Test for the class FileOutputFilter
 *
 */

#include <gtest/gtest.h>

#include "file_output_filter.h"
#include "in_memory_configuration.h"

TEST(FileOutoutFilter, Instantiate) {

	InMemoryConfiguration* config = new InMemoryConfiguration();

	config->set_property("Test.filename", "./data/output.dat");
	config->set_property("Test.item_type", "float");

	FileOutputFilter *output_filter = new FileOutputFilter(config, "Test", 1, 0);

	delete output_filter;
}