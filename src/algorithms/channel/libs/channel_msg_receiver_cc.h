/*!
 * \file channel_msg_receiver_cc.h
 * \brief GNU Radio block that receives asynchronous channel messages from acquisition and tracking blocks
 * \author Javier Arribas, 2016. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
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
#if GNURADIO_USES_STD_POINTERS
#else
#include <boost/shared_ptr.hpp>
#endif

class channel_msg_receiver_cc;

#if GNURADIO_USES_STD_POINTERS
using channel_msg_receiver_cc_sptr = std::shared_ptr<channel_msg_receiver_cc>;
#else
using channel_msg_receiver_cc_sptr = boost::shared_ptr<channel_msg_receiver_cc>;
#endif

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

#endif  // GNSS_SDR_CHANNEL_MSG_RECEIVER_CC_H
