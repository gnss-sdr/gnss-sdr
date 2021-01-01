/*!
 * \file channel_msg_receiver_cc.h
 * \brief GNU Radio block that receives asynchronous channel messages from acquisition and tracking blocks
 * \author Javier Arribas, 2016. jarribas(at)cttc.es
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

#ifndef GNSS_SDR_CHANNEL_MSG_RECEIVER_CC_H
#define GNSS_SDR_CHANNEL_MSG_RECEIVER_CC_H

#include "channel_fsm.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <memory>

/** \addtogroup Channel
 * \{ */
/** \addtogroup Channel_libs
 * \{ */


class channel_msg_receiver_cc;

using channel_msg_receiver_cc_sptr = gnss_shared_ptr<channel_msg_receiver_cc>;

channel_msg_receiver_cc_sptr channel_msg_receiver_make_cc(std::shared_ptr<ChannelFsm> channel_fsm, bool repeat);

/*!
 * \brief GNU Radio block that receives asynchronous channel messages from acquisition and tracking blocks
 */
class channel_msg_receiver_cc : public gr::block
{
public:
    ~channel_msg_receiver_cc() = default;  //!< Default destructor

private:
    friend channel_msg_receiver_cc_sptr channel_msg_receiver_make_cc(std::shared_ptr<ChannelFsm> channel_fsm, bool repeat);
    channel_msg_receiver_cc(std::shared_ptr<ChannelFsm> channel_fsm, bool repeat);
    void msg_handler_channel_events(const pmt::pmt_t& msg);
    std::shared_ptr<ChannelFsm> d_channel_fsm;
    bool d_repeat;  // todo: change FSM to include repeat value
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CHANNEL_MSG_RECEIVER_CC_H
