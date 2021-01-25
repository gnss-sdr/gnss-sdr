/*!
 * \file rtl_tcp_commands.cc
 * \brief Defines methods for communicating with rtl_tcp
 * \author Anthony Arnold, 2015. anthony.arnold(at)uqconnect.edu.au
 *
 * This file contains information taken from librtlsdr:
 *  https://git.osmocom.org/rtl-sdr
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "rtl_tcp_commands.h"
#include <boost/asio/buffer.hpp>                  // for buffer
#include <boost/asio/detail/impl/socket_ops.ipp>  // for host_to_network_long
#include <cstring>                                // for memcpy

boost::system::error_code rtl_tcp_command(RTL_TCP_COMMAND id, unsigned param, boost::asio::ip::tcp::socket &socket)
{
    // Data payload
    unsigned char data[sizeof(unsigned char) + sizeof(unsigned)];

    data[0] = static_cast<unsigned char>(id);

    unsigned nparam = boost::asio::detail::socket_ops::host_to_network_long(param);
    std::memcpy(&data[1], &nparam, sizeof(nparam));

    boost::system::error_code ec;
    if (socket.send(boost::asio::buffer(data), 0, ec) == 0)
        {
            // 0 bytes sent
        }
    return ec;
}
