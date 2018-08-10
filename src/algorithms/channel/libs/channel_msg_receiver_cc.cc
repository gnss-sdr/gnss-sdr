/*!
 * \file channel_msg_receiver_cc.cc
 * \brief GNU Radio block that receives asynchronous channel messages from acquisition and tracking blocks
 * \author Javier Arribas, 2016. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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


#include "channel_msg_receiver_cc.h"
#include <gnuradio/gr_complex.h>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <cstdint>

using google::LogMessage;


channel_msg_receiver_cc_sptr channel_msg_receiver_make_cc(std::shared_ptr<ChannelFsm> channel_fsm, bool repeat)
{
    return channel_msg_receiver_cc_sptr(new channel_msg_receiver_cc(channel_fsm, repeat));
}


void channel_msg_receiver_cc::msg_handler_events(pmt::pmt_t msg)
{
    bool result = false;
    try
        {
            int64_t message = pmt::to_long(msg);
            switch (message)
                {
                case 1:  // positive acquisition
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


channel_msg_receiver_cc::channel_msg_receiver_cc(std::shared_ptr<ChannelFsm> channel_fsm, bool repeat) : gr::block("channel_msg_receiver_cc", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&channel_msg_receiver_cc::msg_handler_events, this, _1));

    d_channel_fsm = channel_fsm;
    d_repeat = repeat;
}


channel_msg_receiver_cc::~channel_msg_receiver_cc() {}
