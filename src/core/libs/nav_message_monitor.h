/*!
 * \file nav_message_monitor.h
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

#ifndef GNSS_SDR_NAV_MESSAGE_MONITOR_H
#define GNSS_SDR_NAV_MESSAGE_MONITOR_H

#include "gnss_block_interface.h"
#include "nav_message_udp_sink.h"
#include <gnuradio/block.h>
#include <pmt/pmt.h>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

class nav_message_monitor;

using nav_message_monitor_sptr = gnss_shared_ptr<nav_message_monitor>;

nav_message_monitor_sptr nav_message_monitor_make(const std::vector<std::string>& addresses, uint16_t port);

/*!
 * \brief GNU Radio block that receives asynchronous Nav_Message_Packet obkects
 * from the telemetry blocks and sends them via UDP
 */
class nav_message_monitor : public gr::block
{
public:
    ~nav_message_monitor() = default;  //!< Default destructor

private:
    friend nav_message_monitor_sptr nav_message_monitor_make(const std::vector<std::string>& addresses, uint16_t port);
    nav_message_monitor(const std::vector<std::string>& addresses, uint16_t port);
    void msg_handler_nav_message(const pmt::pmt_t& msg);
    std::unique_ptr<Nav_Message_Udp_Sink> nav_message_udp_sink_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NAV_MESSAGE_MONITOR_H
