/*!
 * \file nav_msg_udp_listener.h
 * \author Carles Fernandez-Prades, 2021. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_NAV_MSG_UDP_LISTENER_H
#define GNSS_SDR_NAV_MSG_UDP_LISTENER_H

#include "nav_message.pb.h"
#include <boost/asio.hpp>

class Nav_Msg_Udp_Listener
{
public:
    explicit Nav_Msg_Udp_Listener(unsigned short port);
    void print_message(gnss_sdr::navMsg &message) const;
    bool receive_and_parse_nav_message(gnss_sdr::navMsg &message);

private:
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;
    boost::system::error_code error;
    boost::asio::ip::udp::endpoint endpoint;
};

#endif
