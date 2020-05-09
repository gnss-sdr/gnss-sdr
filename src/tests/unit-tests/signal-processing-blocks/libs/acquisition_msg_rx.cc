/*!
 * \file acquisition_msg_rx.cc
 * \brief  This is a helper class to catch the asynchronous messages
 * emitted by an acquisition block.
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.cat
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
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

#include "acquisition_msg_rx.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <cstdint>
#include <utility>
#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind.hpp>
#endif

Acquisition_msg_rx_sptr Acquisition_msg_rx_make()
{
    return Acquisition_msg_rx_sptr(new Acquisition_msg_rx());
}


void Acquisition_msg_rx::msg_handler_events(const pmt::pmt_t& msg)
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
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](pmt::pmt_t&& PH1) { msg_handler_events(PH1); });
#else
        boost::bind(&Acquisition_msg_rx::msg_handler_events, this, _1));
#endif
    rx_message = 0;
}


Acquisition_msg_rx::~Acquisition_msg_rx() = default;
