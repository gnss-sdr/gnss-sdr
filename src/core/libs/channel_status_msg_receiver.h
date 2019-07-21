/*!
 * \file channel_msg_receiver_cc.h
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

#ifndef GNSS_SDR_CHANNEL_STATUS_MSG_RECEIVER_CC_H
#define GNSS_SDR_CHANNEL_STATUS_MSG_RECEIVER_CC_H

#include "gnss_synchro.h"
#include "monitor_pvt.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
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

#endif
