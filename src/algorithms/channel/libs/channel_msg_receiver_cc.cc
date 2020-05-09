/*!
 * \file channel_msg_receiver_cc.cc
 * \brief GNU Radio block that receives asynchronous channel messages from acquisition and tracking blocks
 * \author Javier Arribas, 2016. jarribas(at)cttc.es
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


#include "channel_msg_receiver_cc.h"
#include <boost/any.hpp>
#include <glog/logging.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <cstdint>
#include <utility>

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind.hpp>
#endif

channel_msg_receiver_cc_sptr channel_msg_receiver_make_cc(std::shared_ptr<ChannelFsm> channel_fsm, bool repeat)
{
    return channel_msg_receiver_cc_sptr(new channel_msg_receiver_cc(std::move(channel_fsm), repeat));
}


channel_msg_receiver_cc::channel_msg_receiver_cc(std::shared_ptr<ChannelFsm> channel_fsm, bool repeat) : gr::block("channel_msg_receiver_cc", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](pmt::pmt_t&& PH1) { msg_handler_events(PH1); });
#else
        boost::bind(&channel_msg_receiver_cc::msg_handler_events, this, _1));
#endif

    d_channel_fsm = std::move(channel_fsm);
    d_repeat = repeat;
}


void channel_msg_receiver_cc::msg_handler_events(pmt::pmt_t msg)
{
    bool result = false;
    try
        {
            int64_t message = pmt::to_long(std::move(msg));
            switch (message)
                {
                case 1:  // positive acquisition
                         // Now the acquisition block can optionally trigger the event valid acquisition internally,
                         // in order to reduce acquisition to tracking delay.
                    result = d_channel_fsm->Event_valid_acquisition();
                    break;
                case 2:  // negative acquisition
                    if (d_repeat == true)
                        {
                            result = d_channel_fsm->Event_failed_acquisition_repeat();
                        }
                    else
                        {
                            result = d_channel_fsm->Event_failed_acquisition_no_repeat();
                        }
                    break;
                case 3:  // tracking loss of lock event
                    result = d_channel_fsm->Event_failed_tracking_standby();
                    break;
                default:
                    LOG(WARNING) << "Default case, invalid message.";
                    break;
                }
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
        }
    if (!result)
        {
            LOG(WARNING) << "msg_handler_telemetry invalid event";
        }
}
