
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class implements a Unit Tests for the class ControlMessageFactory.
 *
 */

#include <string>

#include <gtest/gtest.h>

#include "control_message_factory.h"

TEST(ControlMessageFactory, GetQueueMessage) {

	ControlMessageFactory *factory = new ControlMessageFactory();

	gr_message_sptr queue_message = factory->GetQueueMessage(0, 0);
	ControlMessage *control_message = (ControlMessage*)queue_message->msg();

	EXPECT_EQ(0, control_message->who);
	EXPECT_EQ(0, control_message->what);
	EXPECT_EQ(sizeof(ControlMessage), queue_message->length());

	delete factory;
}

TEST(ControlMessageFactory, GetControlMessages) {

	ControlMessageFactory *factory = new ControlMessageFactory();
	ControlMessage *control_message = new ControlMessage;

	control_message->who = 1;
	control_message->what = 4;

	gr_message_sptr queue_message = gr_make_message(0, 0, 0, sizeof(ControlMessage));
	memcpy(queue_message->msg(), control_message, sizeof(ControlMessage));
	std::vector<ControlMessage*> *control_messages = factory->GetControlMessages(queue_message);

	EXPECT_EQ(1, control_messages->size());
	EXPECT_EQ(1, control_messages->at(0)->who);
	EXPECT_EQ(4, control_messages->at(0)->what);

	delete control_message;
	delete control_messages;
	delete factory;
}

TEST(ControlMessageFactory, GetControlMessagesWrongSize) {

	ControlMessageFactory *factory = new ControlMessageFactory();
	ControlMessage *control_message = new ControlMessage;

	control_message->who = 1;
	control_message->what = 4;
	int another_int = 10;

	gr_message_sptr queue_message = gr_make_message(0, 0, 0, sizeof(ControlMessage) + sizeof(int));
	memcpy(queue_message->msg(), control_message, sizeof(ControlMessage));
	memcpy(queue_message->msg() + sizeof(ControlMessage), &another_int, sizeof(int));
	std::vector<ControlMessage*> *control_messages = factory->GetControlMessages(queue_message);

	EXPECT_EQ(0, control_messages->size());

	delete control_message;
	delete control_messages;
	delete factory;
}
