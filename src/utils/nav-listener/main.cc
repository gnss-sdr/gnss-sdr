/*!
 * \file main.cc
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
#include <boost/lexical_cast.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
    try
        {
            // Check command line arguments.
            if (argc != 2)
                {
                    // Print help.
                    std::cerr << "Usage: nav_msg_listener <port>\n";
                    return 1;
                }

            unsigned short port = boost::lexical_cast<unsigned short>(argv[1]);
            Nav_Msg_Udp_Listener udp_listener(port);

            while (true)
                {
                    gnss_sdr::navMsg message;
                    if (udp_listener.receive_and_parse_nav_message(message))
                        {
                            udp_listener.print_message(message);
                        }
                    else
                        {
                            std::cout << "Error: the message cannot be parsed." << std::endl;
                        }
                }
        }
    catch (std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

    return 0;
}
