/*!
 * \file rtl_tcp_commands.h
 * \brief Defines structures and constants for communicating with rtl_tcp
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

#ifndef GNSS_SDR_RTL_TCP_COMMANDS_H
#define GNSS_SDR_RTL_TCP_COMMANDS_H

#include <boost/asio/ip/tcp.hpp>        // for tcp, tcp::socket
#include <boost/system/error_code.hpp>  // for error_code

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


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


/** \} */
/** \} */
#endif  // GNSS_SDR_RTL_TCP_COMMANDS_H
