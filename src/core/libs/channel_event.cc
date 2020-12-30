/*!
 * \file channel_event.cc
 * \brief Class that defines a channel event
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

#include "channel_event.h"

channel_event_sptr channel_event_make(int channel_id, int event_type)
{
    return channel_event_sptr(new Channel_Event(channel_id, event_type));
}

Channel_Event::Channel_Event(int channel_id_, int event_type_)
{
    channel_id = channel_id_;
    event_type = event_type_;
}
