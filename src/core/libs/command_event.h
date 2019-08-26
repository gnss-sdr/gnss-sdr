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

#ifndef GNSS_SDR_COMMAND_EVENT_H
#define GNSS_SDR_COMMAND_EVENT_H

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

#endif
