/*!
 * \file tcp_communication.h
 * \brief Interface of the TCP communication class
 * \author David Pubill, 2011. dpubill(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_TCP_COMMUNICATION_H_
#define GNSS_SDR_TCP_COMMUNICATION_H_

#include "tcp_packet_data.h"
#include <boost/array.hpp>
#include <boost/asio.hpp>

#if BOOST_GREATER_1_65
using b_io_context = boost::asio::io_context;
#else
using b_io_context = boost::asio::io_service;
#endif

#define NUM_TX_VARIABLES_GALILEO_E1 13
#define NUM_TX_VARIABLES_GPS_L1_CA 9
#define NUM_RX_VARIABLES 4

/*!
 * \brief TCP communication class
 */
class Tcp_Communication
{
public:
    Tcp_Communication();
    ~Tcp_Communication() = default;

    int listen_tcp_connection(size_t d_port_, size_t d_port_ch0_);
    void send_receive_tcp_packet_galileo_e1(boost::array<float, NUM_TX_VARIABLES_GALILEO_E1> buf, Tcp_Packet_Data *tcp_data_);
    void send_receive_tcp_packet_gps_l1_ca(boost::array<float, NUM_TX_VARIABLES_GPS_L1_CA> buf, Tcp_Packet_Data *tcp_data_);
    void close_tcp_connection(size_t d_port_);

private:
    b_io_context io_context_;
    boost::asio::ip::tcp::socket tcp_socket_;
};

#endif
