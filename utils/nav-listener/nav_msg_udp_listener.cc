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
#include <iostream>
#include <sstream>
#include <string>

Nav_Msg_Udp_Listener::Nav_Msg_Udp_Listener(unsigned short port)
    : socket{io_service}, endpoint{boost::asio::ip::udp::v4(), port}, connected_socket(true)
{
    socket.open(endpoint.protocol(), error);  // Open socket
    if (error)
        {
            std::cerr << "Error opening socket: " << error.message() << '\n';
            connected_socket = false;
        }
    if (connected_socket)
        {
            socket.bind(endpoint, error);  // Bind the socket to the given local endpoint.
            if (error)
                {
                    std::cerr << "Error binding socket: " << error.message() << '\n';
                    connected_socket = false;
                }
        }
}


/*
 * Blocking call to read nav_message from UDP port
 * return true if message parsed succesfully, false ow
 */
bool Nav_Msg_Udp_Listener::receive_and_parse_nav_message(gnss_sdr::navMsg &message)
{
    if (connected_socket)
        {
            char buff[8192];  // Buffer for storing the received data.

            // This call will block until one or more bytes of data has been received.
            int bytes = socket.receive(boost::asio::buffer(buff));

            std::string data(&buff[0], bytes);
            // Deserialize a stock of Nav_Msg objects from the binary string.
            return message.ParseFromString(data);
        }
    return false;
}


/*
 * Prints navigation message content
 * param[in] message nav message to be printed
 */
void Nav_Msg_Udp_Listener::print_message(gnss_sdr::navMsg &message) const
{
    if (connected_socket)
        {
            std::string system = message.system();
            std::string signal = message.signal();
            int prn = message.prn();
            int tow_at_current_symbol_ms = message.tow_at_current_symbol_ms();
            std::string nav_message = message.nav_message();

            std::cout << "\nNew data received:\n";
            std::cout << "System: " << system << '\n';
            std::cout << "Signal: " << signal << '\n';
            std::cout << "PRN: " << prn << '\n';
            std::cout << "TOW of last symbol [ms]: "
                      << tow_at_current_symbol_ms << '\n';
            std::cout << "Nav message: " << nav_message << "\n\n";
        }
}
