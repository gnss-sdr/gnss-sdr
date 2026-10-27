/*!
 * \file channel_event.h
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

#ifndef GNSS_SDR_CHANNEL_EVENT_H
#define GNSS_SDR_CHANNEL_EVENT_H

#include <cstdint>
#include <limits>
#include <memory>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library core_libs
 * \{ */


class Channel_Event;

using channel_event_sptr = std::shared_ptr<Channel_Event>;

/*!
 * \brief Makes a channel event. \p sample_stamp is the input sample at which
 * the event happened, used to order events in deterministic processing.
 */
channel_event_sptr channel_event_make(int channel_id, int event_type,
    uint64_t sample_stamp = std::numeric_limits<uint64_t>::max());

class Channel_Event
{
public:
    int channel_id;
    int event_type;
    uint64_t sample_stamp;

private:
    friend channel_event_sptr channel_event_make(int channel_id, int event_type, uint64_t sample_stamp);
    Channel_Event(int channel_id_, int event_type_, uint64_t sample_stamp_);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CHANNEL_EVENT_H
