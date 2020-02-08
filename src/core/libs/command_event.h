/*!
 * \file command_event.h
 * \brief Class that defines a receiver command event
 * \author Javier Arribas, 2019. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_COMMAND_EVENT_H_
#define GNSS_SDR_COMMAND_EVENT_H_

#include <memory>

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

#endif  // GNSS_SDR_COMMAND_EVENT_H_
