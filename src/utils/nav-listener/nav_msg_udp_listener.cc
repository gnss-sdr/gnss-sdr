/*!
 * \file nav_msg_udp_listener.cc
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

#include "nav_msg_udp_listener.h"
#include <sstream>

Nav_Msg_Udp_Listener::Nav_Msg_Udp_Listener(unsigned short port)
    : socket{io_service}, endpoint{boost::asio::ip::udp::v4(), port}
{
    socket.open(endpoint.protocol(), error);  // Open socket.
    socket.bind(endpoint, error);             // Bind the socket to the given local endpoint.
}


bool Nav_Msg_Udp_Listener::read_nav_message(gnss_sdr::navMsg &message)
{
    char buff[1500];  // Buffer for storing the received data.

    message_ = message;
    // This call will block until one or more bytes of data has been received.
    int bytes = socket.receive(boost::asio::buffer(buff));

    std::string data(&buff[0], bytes);
    // Deserialize a stock of Nav_Msg objects from the binary string.
    return message_.ParseFromString(data);
}


bool Nav_Msg_Udp_Listener::print_content()
{
    if (read_nav_message(message_))
        {
            std::cout << "\nNew Data received:\n";
            std::cout << "System: " << message_.system() << '\n';
            std::cout << "Signal: " << message_.signal() << '\n';
            std::cout << "PRN: " << message_.prn() << '\n';
            std::cout << "TOW of last symbol [ms]: "
                      << message_.tow_at_current_symbol_ms() << '\n';
            std::cout << "Nav message: " << message_.nav_message() << "\n\n";
        }
    else
        {
            std::cout << "Error: the message cannot be parsed.\n";
            return false;
        }

    return true;
}
