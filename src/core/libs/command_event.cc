/*!
 * \file command_event.cc
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

#include "command_event.h"

command_event_sptr command_event_make(int command_id, int event_type)
{
    return command_event_sptr(new Command_Event(command_id, event_type));
}

Command_Event::Command_Event(int command_id_, int event_type_)
{
    command_id = command_id_;
    event_type = event_type_;
}
