/*!
 * \file control message_factory_test.cc
 * \brief  This file implements tests for the ControlMessageFactory.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
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
#include <gtest/gtest.h>
#include "control_message_factory.h"




TEST(Control_Message_Factory_Test, GetQueueMessage)
{
    ControlMessageFactory *factory = new ControlMessageFactory();

    gr::message::sptr queue_message = factory->GetQueueMessage(0, 0);
    ControlMessage *control_message = (ControlMessage*)queue_message->msg();

    unsigned int expected0 = 0;
    EXPECT_EQ(expected0, control_message->who);
    EXPECT_EQ(expected0, control_message->what);
    EXPECT_EQ(sizeof(ControlMessage), queue_message->length());

    delete factory;
}




TEST(Control_Message_Factory_Test, GetControlMessages)
{
    ControlMessageFactory *factory = new ControlMessageFactory();
    ControlMessage *control_message = new ControlMessage;

    control_message->who = 1;
    control_message->what = 4;

    gr::message::sptr queue_message = gr::message::make(0, 0, 0, sizeof(ControlMessage));
    memcpy(queue_message->msg(), control_message, sizeof(ControlMessage));
    std::vector<ControlMessage*> *control_messages = factory->GetControlMessages(queue_message);

    unsigned int expected1 = 1;
    unsigned int expected4 = 4;
    EXPECT_EQ(expected1, control_messages->size());
    EXPECT_EQ(expected1, control_messages->at(0)->who);
    EXPECT_EQ(expected4, control_messages->at(0)->what);

    delete control_message;
    delete control_messages;
    delete factory;
}



TEST(Control_Message_Factory_Test, GetControlMessagesWrongSize)
{

    ControlMessageFactory *factory = new ControlMessageFactory();
    ControlMessage *control_message = new ControlMessage;

    control_message->who = 1;
    control_message->what = 4;
    int another_int = 10;

    gr::message::sptr queue_message = gr::message::make(0, 0, 0, sizeof(ControlMessage) + sizeof(int));
    memcpy(queue_message->msg(), control_message, sizeof(ControlMessage));
    memcpy(queue_message->msg() + sizeof(ControlMessage), &another_int, sizeof(int));
    std::vector<ControlMessage*> *control_messages = factory->GetControlMessages(queue_message);

    unsigned int expected0 = 0;
    EXPECT_EQ(expected0, control_messages->size());

    delete control_message;
    delete control_messages;
    delete factory;
}
