
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class implements a Unit Test for the class GNSSBlockFactory.
 *
 */
#include <gtest/gtest.h>
#include <gnuradio/gr_msg_queue.h>
#include <vector>
#include "in_memory_configuration.h"
#include "gnss_block_interface.h"
#include "gnss_block_factory.h"

TEST(GNSSBlockFactory, InstantiateChannels)
{
	InMemoryConfiguration *configuration = new InMemoryConfiguration();

	configuration->set_property("Channels.count", "2");
	configuration->set_property("Channel1.implementation", "Pass_Through");
	configuration->set_property("Channel1.item_type", "gr_complex");
	configuration->set_property("Channel1.vector_size", "1");
	configuration->set_property("Channel2.implementation", "Pass_Through");
	configuration->set_property("Channel2.item_type", "gr_complex");
	configuration->set_property("Channel2.vector_size", "1");

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	GNSSBlockFactory *factory = new GNSSBlockFactory();
	std::vector<GNSSBlockInterface*>* channels = factory->GetChannels(configuration, queue);

	EXPECT_EQ(2, channels->size());

	delete configuration;
	delete factory;
	for(unsigned int i=0 ; i<channels->size() ; i++)
        delete channels->at(i);
	channels->clear();
	delete channels;
}

TEST(GNSSBlockFactory, InstantiateSignalSource)
{
	InMemoryConfiguration *configuration = new InMemoryConfiguration();

	configuration->set_property("SignalSource.implementation", "FileSignalSource");

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	GNSSBlockFactory *factory = new GNSSBlockFactory();
	GNSSBlockInterface *signal_source = factory->GetSignalSource(configuration, queue);

	EXPECT_STREQ("SignalSource", signal_source->role().c_str());

	delete configuration;
	delete factory;
	delete signal_source;
}

TEST(GNSSBlockFactory, InstantiateWrongSignalSource) {
	InMemoryConfiguration *configuration = new InMemoryConfiguration();

	configuration->set_property("SignalSource.implementation", "Pepito");

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	GNSSBlockFactory *factory = new GNSSBlockFactory();
	GNSSBlockInterface *signal_source = factory->GetSignalSource(configuration, queue);

	EXPECT_EQ(NULL, signal_source);

	delete configuration;
	delete factory;
}

TEST(GNSSBlockFactory, InstantiateSignalConditioner) {
	InMemoryConfiguration *configuration = new InMemoryConfiguration();

	configuration->set_property("SignalConditioner.implementation", "Pass_Through");

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	GNSSBlockFactory *factory = new GNSSBlockFactory();
	GNSSBlockInterface *signal_conditioner = factory->GetSignalConditioner(configuration, queue);

	EXPECT_STREQ("SignalConditioner", signal_conditioner->role().c_str());

	delete configuration;
	delete factory;
	delete signal_conditioner;
}

TEST(GNSSBlockFactory, InstantiateWrongSignalConditioner) {
	InMemoryConfiguration *configuration = new InMemoryConfiguration();

	configuration->set_property("SignalConditioner.implementation", "Pepito");

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	GNSSBlockFactory *factory = new GNSSBlockFactory();
	GNSSBlockInterface *signal_conditioner = factory->GetSignalConditioner(configuration, queue);

	EXPECT_EQ(NULL, signal_conditioner);

	delete configuration;
	delete factory;
}

TEST(GNSSBlockFactory, InstantiatePVT) {
	InMemoryConfiguration *configuration = new InMemoryConfiguration();

	configuration->set_property("PVT.implementation", "Pass_Through");

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	GNSSBlockFactory *factory = new GNSSBlockFactory();
	GNSSBlockInterface *pvt = factory->GetPVT(configuration, queue);

	EXPECT_STREQ("PVT", pvt->role().c_str());

	delete configuration;
	delete factory;
	delete pvt;
}

TEST(GNSSBlockFactory, InstantiateWrongPVT) {
	InMemoryConfiguration *configuration = new InMemoryConfiguration();

	configuration->set_property("PVT.implementation", "Pepito");

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	GNSSBlockFactory *factory = new GNSSBlockFactory();
	GNSSBlockInterface *pvt = factory->GetPVT(configuration, queue);

	EXPECT_EQ(NULL, pvt);

	delete configuration;
	delete factory;
}

TEST(GNSSBlockFactory, InstantiateOutputFilter) {
	InMemoryConfiguration *configuration = new InMemoryConfiguration();

	configuration->set_property("OutputFilter.implementation", "NullSinkOutputFilter");

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	GNSSBlockFactory *factory = new GNSSBlockFactory();
	GNSSBlockInterface *output_filter = factory->GetOutputFilter(configuration, queue);

	EXPECT_STREQ("OutputFilter", output_filter->role().c_str());

	delete configuration;
	delete factory;
	delete output_filter;
}

TEST(GNSSBlockFactory, InstantiateWrongOutputFilter) {
	InMemoryConfiguration *configuration = new InMemoryConfiguration();

	configuration->set_property("OutputFilter.implementation", "Pepito");

	gr_msg_queue_sptr queue = gr_make_msg_queue(0);

	GNSSBlockFactory *factory = new GNSSBlockFactory();
	GNSSBlockInterface *output_filter = factory->GetOutputFilter(configuration, queue);

	EXPECT_EQ(NULL, output_filter);

	delete configuration;
	delete factory;
}
