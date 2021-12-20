/*!
 * \file acquisition_msg_rx.cc
 * \brief  This is a helper class to catch the asynchronous messages
 * emitted by an acquisition block.
 * \author Carles Fernandez-Prades, 2018. cfernandez(at)cttc.cat
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "acquisition_msg_rx.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <cstdint>
#include <utility>
#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif

#if PMT_USES_BOOST_ANY
#include <boost/any.hpp>
namespace wht = boost;
#else
#include <any>
namespace wht = std;
#endif

Acquisition_msg_rx_sptr Acquisition_msg_rx_make()
{
    return Acquisition_msg_rx_sptr(new Acquisition_msg_rx());
}


void Acquisition_msg_rx::msg_handler_channel_events(const pmt::pmt_t& msg)
{
    try
        {
            int64_t message = pmt::to_long(msg);
            rx_message = message;
            top_block->stop();  // stop the flowgraph
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_acquisition Bad cast!\n";
            rx_message = 0;
        }
}


Acquisition_msg_rx::Acquisition_msg_rx()
    : gr::block("Acquisition_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0)),
      rx_message(0)
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_channel_events(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&Acquisition_msg_rx::msg_handler_channel_events, this, boost::placeholders::_1));
#else
        boost::bind(&Acquisition_msg_rx::msg_handler_channel_events, this, _1));
#endif
#endif
}


Acquisition_msg_rx::~Acquisition_msg_rx() = default;
