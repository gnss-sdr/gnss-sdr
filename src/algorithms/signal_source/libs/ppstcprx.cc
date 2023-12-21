/*!
 * \file ppstcprx.cc
 * \brief TCP client class for front-end PPS samplestamp information reception
 * \author Javier Arribas, jarribas(at)cttc.es
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "ppstcprx.h"
#include <cstring>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

pps_tcp_rx::pps_tcp_rx()
{
    // TODO Auto-generated constructor stub
    is_connected = false;
    clientSd = -1;
}


pps_tcp_rx::~pps_tcp_rx()
{
    // TODO Auto-generated destructor stub
}


void pps_tcp_rx::set_pps_samplestamp_queue(std::shared_ptr<Concurrent_Queue<PpsSamplestamp>> queue)
{
    Pps_queue = std::move(queue);
}


bool pps_tcp_rx::send_cmd(std::string cmd)
{
    if (is_connected == true)
        {
            // send call sends the data you specify as second param and it's length as 3rd param, also returns how many bytes were actually sent
            auto bytes_sent = send(clientSd, cmd.data(), cmd.length(), 0);
            if (bytes_sent <= 0)
                {
                    std::cerr << "Connection terminated...\n";
                    return false;
                }
            else
                {
                    std::cout << "sent bytes..\n";
                }
        }
    else
        {
            return false;
        }
    return true;
}


void pps_tcp_rx::receive_pps(std::string ip_address, int port)
{
    // create a message buffer
    char buf[1500];
    // setup a socket and connection tools
    sockaddr_in sendSockAddr;
    sendSockAddr.sin_family = AF_INET;
    sendSockAddr.sin_addr.s_addr =
        inet_addr(ip_address.c_str());
    sendSockAddr.sin_port = htons(port);
    clientSd = socket(AF_INET, SOCK_STREAM, 0);
    // try to connect...
    int status = connect(clientSd,
        (sockaddr *)&sendSockAddr, sizeof(sendSockAddr));
    if (status < 0)
        {
            std::cout << "pps_tcp_rx: Error connecting to PPS TCP server IP " << ip_address << " at port " << port << std::endl;
            return;
        }
    std::string new_pps_line;

    is_connected = true;
    while (true)
        {
            int numBytesRead = recv(clientSd, buf, sizeof(buf), 0);
            if (numBytesRead > 0)
                {
                    for (int i = 0; i < numBytesRead; i++)
                        {
                            char c = buf[i];
                            if (c == '\n')
                                {
                                    if (new_pps_line.length() > 0)
                                        {
                                            // std::cout << "pps_tcp_rx debug: " << new_pps_line << "\n";
                                            // parse string and push PPS data to the PPS queue
                                            std::stringstream ss(new_pps_line);
                                            std::vector<std::string> data;
                                            while (ss.good())
                                                {
                                                    std::string substr;
                                                    std::getline(ss, substr, ',');
                                                    data.push_back(substr);
                                                }
                                            if (data.size() >= 2)
                                                {
                                                    PpsSamplestamp new_pps;
                                                    // sample counter
                                                    std::size_t found = data.at(0).find("sc=");
                                                    if (found != std::string::npos)
                                                        {
                                                            try
                                                                {
                                                                    new_pps.samplestamp = std::strtoul(data.at(0).substr(found + 3).c_str(), NULL, 0);
                                                                }
                                                            catch (const std::exception &ex)
                                                                {
                                                                    std::cout << "pps_tcp_rx debug: sc parse error str " << data.at(0) << "\n";
                                                                }
                                                        }
                                                    else
                                                        {
                                                            std::cout << "pps_tcp_rx debug: sc parse error str " << data.at(0) << "\n";
                                                        }
                                                    found = data.at(1).find("o=");
                                                    if (found != std::string::npos)
                                                        {
                                                            try
                                                                {
                                                                    new_pps.overflow_reg = std::stoi(data.at(1).substr(found + 2).c_str(), NULL, 0);
                                                                }
                                                            catch (const std::exception &ex)
                                                                {
                                                                    std::cout << "pps_tcp_rx debug: o parse error str " << data.at(0) << "\n";
                                                                }
                                                        }
                                                    else
                                                        {
                                                            std::cout << "pps_tcp_rx debug: o parse error str " << data.at(1) << "\n";
                                                        }
                                                    Pps_queue->push(new_pps);
                                                    // std::cout << "pps_tcp_rx debug: pps pushed!\n";
                                                }
                                            else
                                                {
                                                    std::cout << "pps_tcp_rx debug: protocol error!\n";
                                                }
                                            new_pps_line = "";
                                        }
                                }
                            else
                                new_pps_line += c;
                        }
                }
            else
                {
                    std::cout << "pps_tcp_rx: Socket disconnected!\n!";
                    break;
                }
        }
    is_connected = false;
}
