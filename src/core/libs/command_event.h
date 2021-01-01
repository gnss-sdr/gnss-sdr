/*!
 * \file command_event.h
 * \brief Class that defines a receiver command event
 * \author Javier Arribas, 2019. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_COMMAND_EVENT_H
#define GNSS_SDR_COMMAND_EVENT_H

#include <memory>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */


class Command_Event;

using command_event_sptr = std::shared_ptr<Command_Event>;

command_event_sptr command_event_make(int command_id, int event_type);

class Command_Event
{
public:
    int command_id;
    int event_type;

private:
    friend command_event_sptr command_event_make(int command_id, int event_type);
    Command_Event(int command_id_, int event_type_);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_COMMAND_EVENT_H
