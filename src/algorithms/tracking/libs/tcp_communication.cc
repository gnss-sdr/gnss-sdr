/*!
 * \file tcp_communication.cc
 * \brief Implementation of the TCP communication class
 * \author David Pubill, 2011. dpubill(at)cttc.es
 *
 *
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

#include "tcp_packet_data.h"
#include "tcp_communication.h"
#include <iostream>
#include <string>


tcp_communication::tcp_communication() : tcp_socket_(io_service_)
{
}


tcp_communication::~tcp_communication()
{
}


int tcp_communication::listen_tcp_connection(size_t d_port_, size_t d_port_ch0_)
{
    try
        {
            // Specify IP type and port
            boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), d_port_);
            boost::asio::ip::tcp::acceptor acceptor(io_service_, endpoint);

            if (d_port_ == d_port_ch0_)
                {
                    std::cout << "Server ready. Listening for TCP connections..." << std::endl;
                }

            // Reuse the IP address for each connection
            acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));

            // Listen for a connection and accept it
            acceptor.listen(12);
            acceptor.accept(tcp_socket_);

            std::cout << "Socket accepted on port " << d_port_ << std::endl;
        }

    catch (const std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
        }

    return false;
}


void tcp_communication::send_receive_tcp_packet_galileo_e1(boost::array<float, NUM_TX_VARIABLES_GALILEO_E1> buf, tcp_packet_data* tcp_data_)
{
    int controlc = 0;
    boost::array<float, NUM_RX_VARIABLES> readbuf;
    float d_control_id_ = buf.data()[0];

    try
        {
            // Send a TCP packet
            tcp_socket_.write_some(boost::asio::buffer(buf));

            // Read the received TCP packet
            tcp_socket_.read_some(boost::asio::buffer(readbuf));

            //! Control. The GNSS-SDR program ends if an error in a TCP packet is detected.
            if (d_control_id_ != readbuf.data()[0])
                {
                    throw "Packet error!";
                }

            // Recover the variables received
            tcp_data_->proc_pack_code_error = readbuf.data()[1];
            tcp_data_->proc_pack_carr_error = readbuf.data()[2];
            tcp_data_->proc_pack_carrier_doppler_hz = readbuf.data()[3];
        }

    catch (const std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << ". Please press Ctrl+C to end the program." << std::endl;
            std::cin >> controlc;
        }
    return;
}


void tcp_communication::send_receive_tcp_packet_gps_l1_ca(boost::array<float, NUM_TX_VARIABLES_GPS_L1_CA> buf, tcp_packet_data* tcp_data_)
{
    int controlc = 0;
    boost::array<float, NUM_RX_VARIABLES> readbuf;
    float d_control_id_ = buf.data()[0];

    try
        {
            // Send a TCP packet
            tcp_socket_.write_some(boost::asio::buffer(buf));

            // Read the received TCP packet
            tcp_socket_.read_some(boost::asio::buffer(readbuf));

            //! Control. The GNSS-SDR program ends if an error in a TCP packet is detected.
            if (d_control_id_ != readbuf.data()[0])
                {
                    throw "Packet error!";
                }

            // Recover the variables received
            tcp_data_->proc_pack_code_error = readbuf.data()[1];
            tcp_data_->proc_pack_carr_error = readbuf.data()[2];
            tcp_data_->proc_pack_carrier_doppler_hz = readbuf.data()[3];
        }

    catch (const std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << ". Please press Ctrl+C to end the program." << std::endl;
            std::cin >> controlc;
        }
    return;
}


void tcp_communication::close_tcp_connection(size_t d_port_)
{
    // Close the TCP connection
    tcp_socket_.close();
    std::cout << "Socket closed on port " << d_port_ << std::endl;
    return;
}
