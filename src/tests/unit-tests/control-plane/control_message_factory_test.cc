/*!
 * \file control message_factory_test.cc
 * \brief  This file implements tests for the ControlMessageFactory.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include <string>
#include <gtest/gtest.h>
#include "control_message_factory.h"


TEST(ControlMessageFactoryTest, GetQueueMessage)
{
    std::shared_ptr<ControlMessageFactory> factory = std::make_shared<ControlMessageFactory>();
    gr::message::sptr queue_message = factory->GetQueueMessage(0, 2);
    auto control_messages = factory->GetControlMessages(queue_message);
    unsigned int expected0 = 0;
    unsigned int expected2 = 2;
    EXPECT_EQ(expected0, control_messages->at(0)->who);
    EXPECT_EQ(expected2, control_messages->at(0)->what);
    EXPECT_EQ(sizeof(ControlMessage), queue_message->length());
}


TEST(ControlMessageFactoryTest, GetControlMessages)
{
    std::shared_ptr<ControlMessageFactory> factory = std::make_shared<ControlMessageFactory>();
    gr::message::sptr queue_message = gr::message::make(0, 0, 0, sizeof(ControlMessage));
    std::shared_ptr<ControlMessage> control_message = std::make_shared<ControlMessage>();

    control_message->who = 1;
    control_message->what = 4;

    memcpy(queue_message->msg(), control_message.get(), sizeof(ControlMessage));
    std::shared_ptr<std::vector<std::shared_ptr<ControlMessage>>> control_messages = factory->GetControlMessages(queue_message);

    unsigned int expected1 = 1;
    unsigned int expected4 = 4;
    EXPECT_EQ(expected1, control_messages->size());
    EXPECT_EQ(expected1, control_messages->at(0)->who);
    EXPECT_EQ(expected4, control_messages->at(0)->what);
}

/*

TEST(ControlMessageFactoryTest, GetControlMessagesWrongSize)
{

    std::shared_ptr<ControlMessageFactory> factory = std::make_shared<ControlMessageFactory>();
    std::shared_ptr<ControlMessage> control_message = std::make_shared<ControlMessage>();

    control_message->who = 1;
    control_message->what = 4;
    int another_int = 10;

    gr::message::sptr queue_message = gr::message::make(0, 0, 0, sizeof(ControlMessage) + sizeof(int));
    memcpy(queue_message->msg(), control_message.get(), sizeof(ControlMessage));
    memcpy(queue_message->msg() + sizeof(ControlMessage), &another_int, sizeof(int));
    std::shared_ptr<std::vector<std::shared_ptr<ControlMessage>>> control_messages = factory->GetControlMessages(queue_message);

    unsigned int expected0 = 0;
    EXPECT_EQ(expected0, control_messages->size());
} */
