/*!
 * \file rtl_tcp_commands.h
 * \brief Defines structures and constants for communicating with rtl_tcp
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
 *
 * This file contains information taken from librtlsdr:
 *  http://git.osmocom.org/rtl-sdr/
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */
#ifndef GNSS_SDR_RTL_TCP_COMMANDS_H
#define GNSS_SDR_RTL_TCP_COMMANDS_H

#include <boost/asio/ip/tcp.hpp>

/// Command IDs for configuration rtl_tcp
enum RTL_TCP_COMMAND
{
    RTL_TCP_SET_FREQUENCY = 1,
    RTL_TCP_SET_SAMPLE_RATE = 2,
    RTL_TCP_SET_GAIN_MODE = 3,
    RTL_TCP_SET_GAIN = 4,
    RTL_TCP_SET_IF_GAIN = 6,
    RTL_TCP_SET_AGC_MODE = 8
};


/*!
 * \brief Send a command to rtl_tcp over the given socket.
 */
boost::system::error_code rtl_tcp_command(RTL_TCP_COMMAND id, unsigned param,
    boost::asio::ip::tcp::socket &socket);

#endif  // GNSS_SDR_RTL_TCP_COMMANDS_H
