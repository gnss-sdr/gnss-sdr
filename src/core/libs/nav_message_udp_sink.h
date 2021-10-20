/*!
 * \file nav_message_udp_sink.h
 * \brief Interface of a class that sends serialized Nav_Message_Packet objects
 * over UDP to one or multiple endpoints.
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

#ifndef GNSS_SDR_NAV_MESSAGE_UDP_SINK_H
#define GNSS_SDR_NAV_MESSAGE_UDP_SINK_H

#include "nav_message_packet.h"
#include "serdes_nav_message.h"
#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

#if USE_BOOST_ASIO_IO_CONTEXT
using b_io_context = boost::asio::io_context;
#else
using b_io_context = boost::asio::io_service;
#endif

class Nav_Message_Udp_Sink
{
public:
    Nav_Message_Udp_Sink(const std::vector<std::string>& addresses, const uint16_t& port);
    bool write_nav_message(const std::shared_ptr<Nav_Message_Packet>& nav_meg_packet);

private:
    Serdes_Nav_Message serdes_nav;
    b_io_context io_context;
    boost::asio::ip::udp::socket socket;
    std::vector<boost::asio::ip::udp::endpoint> endpoints;
    boost::system::error_code error;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_NAV_MESSAGE_UDP_SINK_H
