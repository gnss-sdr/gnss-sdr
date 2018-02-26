/*!
 * \file control_message_factory.h
 * \brief Interface of a factory for control messages.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CONTROL_MESSAGE_FACTORY_H_
#define GNSS_SDR_CONTROL_MESSAGE_FACTORY_H_

#include <gnuradio/message.h>
#include <memory>
#include <vector>

//! Message described by who sent it and what it says
typedef struct ControlMessage_
{
    unsigned int who;
    unsigned int what;
} ControlMessage ;


/*!
 * \brief This class implements a factory for Control Messages.
 *
 * It encapsulates the complexity behind getting Queue Messages and associated Control Messages
 */
class ControlMessageFactory
{

public:
    //! Constructor
    ControlMessageFactory();

    //! Virtual destructor
    virtual ~ControlMessageFactory();

    gr::message::sptr GetQueueMessage(unsigned int who, unsigned int what);
    std::shared_ptr<std::vector<std::shared_ptr<ControlMessage>>> GetControlMessages(gr::message::sptr queue_message);
};

#endif /*GNSS_SDR_CONTROL_MESSAGE_FACTORY_H_*/
