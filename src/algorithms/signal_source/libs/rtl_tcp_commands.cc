/*!
 * \file rtl_tcp_commands.cc
 * \brief Defines methods for communicating with rtl_tcp
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
 *
 * This file contains information taken from librtlsdr:
 *  http://git.osmocom.org/rtl-sdr/
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "rtl_tcp_commands.h"
#include <string>

boost::system::error_code rtl_tcp_command(RTL_TCP_COMMAND id, unsigned param, boost::asio::ip::tcp::socket &socket)
{
    // Data payload
    unsigned char data[sizeof(unsigned char) + sizeof(unsigned)];

    data[0] = static_cast<unsigned char>(id);

    unsigned nparam = boost::asio::detail::socket_ops::host_to_network_long(param);
    std::memcpy(&data[1], &nparam, sizeof(nparam));

    boost::system::error_code ec;
    socket.send(boost::asio::buffer(data), 0, ec);
    return ec;
}
