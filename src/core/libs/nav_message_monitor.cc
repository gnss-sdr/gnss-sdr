/*!
 * \file nav_message_monitor.cc
 * \brief GNU Radio block that processes Nav_Message_Packet received from
 * telemetry blocks and sends them via UDP.
 * \author Carles Fernandez-Prades, 2021. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "nav_message_monitor.h"
#include "gnss_sdr_make_unique.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <cstddef>   // size_t
#include <typeinfo>  // typeid

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

nav_message_monitor_sptr nav_message_monitor_make(const std::vector<std::string>& addresses, uint16_t port)
{
    return nav_message_monitor_sptr(new nav_message_monitor(addresses, port));
}


nav_message_monitor::nav_message_monitor(const std::vector<std::string>& addresses, uint16_t port) : gr::block("nav_message_monitor", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    // register Nav_msg_from_TLM input message port from telemetry blocks
    this->message_port_register_in(pmt::mp("Nav_msg_from_TLM"));
    this->set_msg_handler(pmt::mp("Nav_msg_from_TLM"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_nav_message(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&nav_message_monitor::msg_handler_nav_message, this, boost::placeholders::_1));
#else
        boost::bind(&nav_message_monitor::msg_handler_nav_message, this, _1));
#endif
#endif
    nav_message_udp_sink_ = std::make_unique<Nav_Message_Udp_Sink>(addresses, port);
}


void nav_message_monitor::msg_handler_nav_message(const pmt::pmt_t& msg)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with msg_handler_galileo_e6_has function called by the scheduler

    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            if (msg_type_hash_code == typeid(std::shared_ptr<Nav_Message_Packet>).hash_code())
                {
                    const auto nav_message_packet = wht::any_cast<std::shared_ptr<Nav_Message_Packet>>(pmt::any_ref(msg));
                    nav_message_udp_sink_->write_nav_message(nav_message_packet);
                }
            else
                {
                    LOG(WARNING) << "nav_message_monitor received an unknown object type!";
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "nav_message_monitor Bad any_cast: " << e.what();
        }
}
