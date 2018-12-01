/*!
 * \file acquisition_msg_rx.cc
 * \brief  This is a helper class to catch the asynchronous messages
 * emitted by an acquisition block.
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.cat
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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

#include "acquisition_msg_rx.h"
#include <cstdint>
#include <boost/bind.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>


Acquisition_msg_rx_sptr Acquisition_msg_rx_make()
{
    return Acquisition_msg_rx_sptr(new Acquisition_msg_rx());
}


void Acquisition_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            int64_t message = pmt::to_long(msg);
            rx_message = message;
            top_block->stop();  // stop the flowgraph
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_acquisition Bad cast!\n";
            rx_message = 0;
        }
}


Acquisition_msg_rx::Acquisition_msg_rx() : gr::block("Acquisition_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&Acquisition_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


Acquisition_msg_rx::~Acquisition_msg_rx() {}
