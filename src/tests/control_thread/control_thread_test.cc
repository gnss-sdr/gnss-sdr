
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class implements a Unit Tests for the class ControlThread.
 *
 */

#include <gtest/gtest.h>
#include <gr_msg_queue.h>
#include "control_thread.h"
#include "in_memory_configuration.h"


TEST(Control_Thread_Test, InstantiateRunControlMessages) {

	InMemoryConfiguration *config = new InMemoryConfiguration();
        config->set_property("SignalSource.implementation", "File_Signal_Source");
        config->set_property("SignalSource.filename", "/Users/carlesfernandez/Documents/workspace/gnss-sdr/trunk/data/sc2_d16.dat");
        config->set_property("SignalSource.item_type", "gr_complex");
        config->set_property("SignalConditioner.implementation", "Pass_Through");
        config->set_property("SignalConditioner.item_type", "gr_complex");
        config->set_property("Channels.count", "12");
        config->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Acquisition");
        config->set_property("Acquisition.item_type", "gr_complex");
        config->set_property("Tracking.implementation", "GPS_L1_CA_DLL_FLL_PLL_Tracking");
        config->set_property("Tracking.item_type", "gr_complex");
        config->set_property("Navigation.implementation", "GPS_L1_CA_Telemetry_Decoder");
        config->set_property("Navigation.item_type", "gr_complex");
        config->set_property("Pseudorange.implementation", "GPS_L1_CA_Observables");
        config->set_property("Pseudorange.item_type", "gr_complex");
        config->set_property("PVT.implementation", "GPS_L1_CA_PVT");
        config->set_property("PVT.item_type", "gr_complex");
        config->set_property("OutputFilter.implementation", "Null_Sink_Output_Filter");
        config->set_property("OutputFilter.item_type", "gr_complex");

	ControlThread *control_thread = new ControlThread(config);

	gr_msg_queue_sptr control_queue = gr_make_msg_queue(0);
	ControlMessageFactory *control_msg_factory = new ControlMessageFactory();

	control_queue->handle(control_msg_factory->GetQueueMessage(0,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(1,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(2,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(3,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(4,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(5,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(6,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(7,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(8,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(9,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(10,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(11,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(12,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(13,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(14,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(15,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(16,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(200,0));

	control_thread->set_control_queue(control_queue);

	control_thread->run();

	EXPECT_EQ(18, control_thread->processed_control_messages());
	EXPECT_EQ(1, control_thread->applied_actions());

	delete config;
	delete control_thread;
	delete control_msg_factory;
}





TEST(Control_Thread_Test, InstantiateRunControlMessages2) {

	InMemoryConfiguration *config = new InMemoryConfiguration();
	config->set_property("SignalSource.implementation", "File_Signal_Source");
	config->set_property("SignalSource.filename", "/Users/carlesfernandez/Documents/workspace/gnss-sdr/trunk/data/sc2_d16.dat");
	config->set_property("SignalSource.item_type", "gr_complex");
	config->set_property("SignalConditioner.implementation", "Pass_Through");
	config->set_property("SignalConditioner.item_type", "gr_complex");
	config->set_property("Channels.count", "12");
	config->set_property("Acquisition.implementation", "GPS_L1_CA_PCPS_Acquisition");
	config->set_property("Acquisition.item_type", "gr_complex");
	config->set_property("Tracking.implementation", "GPS_L1_CA_DLL_FLL_PLL_Tracking");
	config->set_property("Tracking.item_type", "gr_complex");
	config->set_property("Navigation.implementation", "GPS_L1_CA_Telemetry_Decoder");
	config->set_property("Navigation.item_type", "gr_complex");
	config->set_property("Pseudorange.implementation", "GPS_L1_CA_Observables");
	config->set_property("Pseudorange.item_type", "gr_complex");
	config->set_property("PVT.implementation", "GPS_L1_CA_PVT");
	config->set_property("PVT.item_type", "gr_complex");
	config->set_property("OutputFilter.implementation", "Null_Sink_Output_Filter");
	config->set_property("OutputFilter.item_type", "gr_complex");

	ControlThread *control_thread = new ControlThread(config);

	gr_msg_queue_sptr control_queue = gr_make_msg_queue(0);
	ControlMessageFactory *control_msg_factory = new ControlMessageFactory();

	control_queue->handle(control_msg_factory->GetQueueMessage(0,0));
	control_queue->handle(control_msg_factory->GetQueueMessage(0,2));
	control_queue->handle(control_msg_factory->GetQueueMessage(0,1));
	control_queue->handle(control_msg_factory->GetQueueMessage(0,3));
	control_queue->handle(control_msg_factory->GetQueueMessage(200,0));

	control_thread->set_control_queue(control_queue);

	control_thread->run();

	EXPECT_EQ(5, control_thread->processed_control_messages());
	EXPECT_EQ(1, control_thread->applied_actions());

	delete config;
	delete control_thread;
	delete control_msg_factory;
}
