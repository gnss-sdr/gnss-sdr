
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * Integration test for file signal source.
 *
 */

#include <gtest/gtest.h>

#include <iostream>
#include <fstream>

#include "in_memory_configuration.h"
#include "control_thread.h"

TEST(SignalSource, CorrectFileSignalSource) {

	InMemoryConfiguration* config = new InMemoryConfiguration();

	config->set_property("ControlThread.wait_for_flowgraph", "true");

	config->set_property("SignalSource.implementation", "FileSignalSource");
	config->set_property("SignalSource.samples", "0");
	config->set_property("SignalSource.filename", "./signal_samples/signal_1ms.dat");
	config->set_property("SignalSource.dump", "true");
	config->set_property("SignalSource.dump_filename", "./data/test1_dump.dat");

	config->set_property("SignalConditioner.implementation", "PassThrough");

	config->set_property("Channels.count", "1");

	ControlThread* control_thread = new ControlThread(config);
	control_thread->run();

	delete control_thread;
	delete config;

	std::ifstream signal_expected;
	signal_expected.open("./signal_samples/signal_1ms.dat", std::ios::in|std::ios::binary|std::ios::ate);
	EXPECT_FALSE(signal_expected.fail());

	std::ifstream signal_result;
	signal_result.open("./data/test_dump.dat", std::ios::in|std::ios::binary|std::ios::ate);
	EXPECT_FALSE(signal_result.fail());

	EXPECT_EQ(signal_expected.tellg(), signal_result.tellg());
	std::cout << signal_expected.tellg() << ":" << signal_result.tellg() << std::endl;

	signal_expected.close();
	signal_result.close();
}
