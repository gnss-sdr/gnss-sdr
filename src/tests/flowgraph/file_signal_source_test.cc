
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

#include <gr_block.h>
#include <gr_msg_queue.h>
#include <gr_top_block.h>
#include <gr_null_sink.h>

#include <stdexcept>

#include "file_signal_source.h"
#include "in_memory_configuration.h"

TEST(FileSignalSource, Instantiate) {

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	InMemoryConfiguration* config = new InMemoryConfiguration();

	config->set_property("Test.samples", "0");
	config->set_property("Test.sampling_frequency", "0");
	config->set_property("Test.filename", "./signal_samples/signal.dat");
	config->set_property("Test.item_type", "gr_complex");
	config->set_property("Test.repeat", "false");

	FileSignalSource *signal_source = new FileSignalSource(config, "Test", 1, 1, queue);

	EXPECT_STREQ("./signal_samples/signal.dat", signal_source->filename().c_str());
	EXPECT_STREQ("gr_complex", signal_source->item_type().c_str());
	EXPECT_TRUE(signal_source->repeat() == false);

	delete signal_source;
}

TEST(FileSignalSource, InstantiateFileNotExists) {

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	InMemoryConfiguration* config = new InMemoryConfiguration();

	config->set_property("Test.samples", "0");
	config->set_property("Test.sampling_frequency", "0");
	config->set_property("Test.filename", "./signal_samples/i_dont_exist.dat");
	config->set_property("Test.item_type", "gr_complex");
	config->set_property("Test.repeat", "false");

	EXPECT_THROW(new FileSignalSource(config, "Test", 1, 1, queue), std::runtime_error);
}
