/*!
 * \file channel_event.h
 * \brief Class that defines a channel event
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

#ifndef GNSS_SDR_CHANNEL_EVENT_H
#define GNSS_SDR_CHANNEL_EVENT_H

#include <memory>

class Channel_Event;

using channel_event_sptr = std::shared_ptr<Channel_Event>;

channel_event_sptr channel_event_make(int channel_id, int event_type);

class Channel_Event
{
public:
    int channel_id;
    int event_type;

private:
    friend channel_event_sptr channel_event_make(int channel_id, int event_type);
    Channel_Event(int channel_id_, int event_type_);
};

#endif  // GNSS_SDR_CHANNEL_EVENT_H
