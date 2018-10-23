/*!
 * \file tcp_cmd_interface.h
 *
 * \brief Class that implements a TCP telecontrol command line interface
 * for GNSS-SDR
 * \author Javier Arribas jarribas (at) cttc.es
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
#ifndef GNSS_SDR_TCPCMDINTERFACE_H_
#define GNSS_SDR_TCPCMDINTERFACE_H_

#include <functional>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <boost/asio.hpp>
#include <glog/logging.h>
#include <cstdint>
#include <gnuradio/message.h>
#include <gnuradio/msg_queue.h>


class TcpCmdInterface
{
public:
    TcpCmdInterface();
    virtual ~TcpCmdInterface();
    void run_cmd_server(int tcp_port);
    void set_msg_queue(gr::msg_queue::sptr control_queue);


private:
    std::unordered_map<std::string, std::function<std::string(const std::vector<std::string> &)>>
        functions;
    std::string status(const std::vector<std::string> &commandLine);
    std::string reset(const std::vector<std::string> &commandLine);
    std::string stop(const std::vector<std::string> &commandLine);
    std::string assistedstart(const std::vector<std::string> &commandLine);
    std::string warmstart(const std::vector<std::string> &commandLine);
    std::string coldstart(const std::vector<std::string> &commandLine);
    std::string set_ch_satellite(const std::vector<std::string> &commandLine);

    void register_functions();

    gr::msg_queue::sptr control_queue_;
    bool keep_running_;
};

#endif /* GNSS_SDR_TCPCMDINTERFACE_H_ */
