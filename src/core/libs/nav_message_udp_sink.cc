/*!
 * \file nav_message_udp_sink.cc
 * \brief Implementation of a class that sends serialized Nav_Message_Packet
 * objects over UDP to one or multiple endpoints.
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

#include "nav_message_udp_sink.h"
#include <iostream>
#include <sstream>


Nav_Message_Udp_Sink::Nav_Message_Udp_Sink(const std::vector<std::string>& addresses, const uint16_t& port) : socket{io_context}
{
    for (const auto& address : addresses)
        {
            boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(address, error), port);
            endpoints.push_back(endpoint);
        }
    serdes_nav = Serdes_Nav_Message();
}


bool Nav_Message_Udp_Sink::write_nav_message(const std::shared_ptr<Nav_Message_Packet>& nav_meg_packet)
{
    std::string outbound_data = serdes_nav.createProtobuffer(nav_meg_packet);

    for (const auto& endpoint : endpoints)
        {
            socket.open(endpoint.protocol(), error);
            socket.connect(endpoint, error);

            try
                {
                    if (socket.send(boost::asio::buffer(outbound_data)) == 0)
                        {
                            return false;
                        }
                }
            catch (boost::system::system_error const& e)
                {
                    return false;
                }
        }
    return true;
}
