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

#include "pvt_interface.h"
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
#include <armadillo>
#include <ctime>

class TcpCmdInterface
{
public:
    TcpCmdInterface();
    virtual ~TcpCmdInterface();
    void run_cmd_server(int tcp_port);
    void set_msg_queue(gr::msg_queue::sptr control_queue);
    /*!
     * \brief gets the UTC time parsed from the last TC command issued
     */
    time_t get_utc_time();
    /*!
     * \brief gets the Latitude, Longitude and Altitude vector from the last TC command issued
     */
    arma::vec get_LLH();

    void set_pvt(std::shared_ptr<PvtInterface> PVT_sptr);

private:
    std::unordered_map<std::string, std::function<std::string(const std::vector<std::string> &)>>
        functions;
    std::string status(const std::vector<std::string> &commandLine);
    std::string reset(const std::vector<std::string> &commandLine);
    std::string standby(const std::vector<std::string> &commandLine);
    std::string hotstart(const std::vector<std::string> &commandLine);
    std::string warmstart(const std::vector<std::string> &commandLine);
    std::string coldstart(const std::vector<std::string> &commandLine);
    std::string set_ch_satellite(const std::vector<std::string> &commandLine);

    void register_functions();

    gr::msg_queue::sptr control_queue_;
    bool keep_running_;

    time_t receiver_utc_time_;

    double rx_latitude_;
    double rx_longitude_;
    double rx_altitude_;

    std::shared_ptr<PvtInterface> PVT_sptr_;
};

#endif /* GNSS_SDR_TCPCMDINTERFACE_H_ */
