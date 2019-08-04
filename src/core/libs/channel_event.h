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

#endif
