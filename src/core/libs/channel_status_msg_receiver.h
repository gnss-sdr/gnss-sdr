/*!
 * \file channel_status_msg_receiver.h
 * \brief GNU Radio block that receives asynchronous channel messages from acquisition and tracking blocks
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

#ifndef GNSS_SDR_CHANNEL_STATUS_MSG_RECEIVER_CC_H
#define GNSS_SDR_CHANNEL_STATUS_MSG_RECEIVER_CC_H

#include "gnss_synchro.h"
#include "monitor_pvt.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <map>
#include <memory>

class channel_status_msg_receiver;

using channel_status_msg_receiver_sptr = boost::shared_ptr<channel_status_msg_receiver>;

channel_status_msg_receiver_sptr channel_status_msg_receiver_make();

/*!
 * \brief GNU Radio block that receives asynchronous channel messages from tlm blocks
 */
class channel_status_msg_receiver : public gr::block
{
public:
    ~channel_status_msg_receiver() = default;  //!< Default destructor

    /*!
     * \brief return the current status map of all channels with valid telemetry
     */
    std::map<int, std::shared_ptr<Gnss_Synchro>> get_current_status_map();

    /*!
     * \brief return the current receiver PVT
     */
    Monitor_Pvt get_current_status_pvt();

private:
    friend channel_status_msg_receiver_sptr channel_status_msg_receiver_make();
    channel_status_msg_receiver();
    std::map<int, std::shared_ptr<Gnss_Synchro>> d_channel_status_map;
    Monitor_Pvt d_pvt_status{};
    void msg_handler_events(const pmt::pmt_t& msg);
};

#endif  // GNSS_SDR_CHANNEL_STATUS_MSG_RECEIVER_CC_H
