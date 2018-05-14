/*!
 * \file control_message_factory.cc
 * \brief Implementation of a Control Message Factory
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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

#include "control_message_factory.h"
#include <glog/logging.h>


using google::LogMessage;

// Constructor
ControlMessageFactory::ControlMessageFactory() {}


// Destructor
ControlMessageFactory::~ControlMessageFactory() {}


gr::message::sptr ControlMessageFactory::GetQueueMessage(unsigned int who, unsigned int what)
{
    std::shared_ptr<ControlMessage> control_message = std::make_shared<ControlMessage>();
    control_message->who = who;
    control_message->what = what;
    gr::message::sptr queue_message = gr::message::make(0, 0, 0, sizeof(ControlMessage));
    memcpy(queue_message->msg(), control_message.get(), sizeof(ControlMessage));
    return queue_message;
}


std::shared_ptr<std::vector<std::shared_ptr<ControlMessage>>> ControlMessageFactory::GetControlMessages(gr::message::sptr queue_message)
{
    std::shared_ptr<std::vector<std::shared_ptr<ControlMessage>>> control_messages = std::make_shared<std::vector<std::shared_ptr<ControlMessage>>>();
    unsigned int control_messages_count = queue_message->length() / sizeof(ControlMessage);
    if (queue_message->length() % sizeof(ControlMessage) != 0)
        {
            LOG(WARNING) << "Queue message has size " << queue_message->length() << ", which is not"
                         << " multiple of control message size " << sizeof(ControlMessage);
            LOG(WARNING) << "Ignoring this queue message to prevent unexpected results.";
            return control_messages;
        }
    for (unsigned int i = 0; i < control_messages_count; i++)
        {
            control_messages->push_back(std::make_shared<ControlMessage>());
            memcpy(control_messages->at(i).get(), queue_message->msg() + (i * sizeof(ControlMessage)), sizeof(ControlMessage));
        }
    return control_messages;
}
