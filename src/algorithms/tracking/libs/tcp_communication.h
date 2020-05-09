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
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_TCP_COMMUNICATION_H
#define GNSS_SDR_TCP_COMMUNICATION_H

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
