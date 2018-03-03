/*!
 * \file channel_msg_receiver_cc.h
 * \brief GNU Radio block that receives asynchronous channel messages from acquisition and tracking blocks
 * \author Javier Arribas, 2016. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2016  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CHANNEL_MSG_RECEIVER_CC_H
#define GNSS_SDR_CHANNEL_MSG_RECEIVER_CC_H

#include "channel_fsm.h"
#include <gnuradio/block.h>

class channel_msg_receiver_cc;

typedef boost::shared_ptr<channel_msg_receiver_cc> channel_msg_receiver_cc_sptr;

channel_msg_receiver_cc_sptr channel_msg_receiver_make_cc(std::shared_ptr<ChannelFsm> channel_fsm, bool repeat);

/*!
 * \brief GNU Radio block that receives asynchronous channel messages from acquisition and tracking blocks
 */
class channel_msg_receiver_cc : public gr::block
{
private:
    std::shared_ptr<ChannelFsm> d_channel_fsm;
    bool d_repeat;  // todo: change FSM to include repeat value
    friend channel_msg_receiver_cc_sptr channel_msg_receiver_make_cc(std::shared_ptr<ChannelFsm> channel_fsm, bool repeat);
    void msg_handler_events(pmt::pmt_t msg);
    channel_msg_receiver_cc(std::shared_ptr<ChannelFsm> channel_fsm, bool repeat);

public:
    ~channel_msg_receiver_cc();  //!< Default destructor
};

#endif
