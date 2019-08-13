/*!
 * \file tcp_cmd_interface.h
 *
 * \brief Class that implements a TCP/IP telecommand command line interface
 * for GNSS-SDR
 * \author Javier Arribas jarribas (at) cttc.es
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
#ifndef GNSS_SDR_TCP_CMD_INTERFACE_H_
#define GNSS_SDR_TCP_CMD_INTERFACE_H_


#include "concurrent_queue.h"
#include <pmt/pmt.h>
#include <array>
#include <cstdint>
#include <ctime>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class PvtInterface;

class TcpCmdInterface
{
public:
    TcpCmdInterface();
    ~TcpCmdInterface() = default;
    void run_cmd_server(int tcp_port);
    void set_msg_queue(std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue);

    /*!
     * \brief gets the UTC time parsed from the last TC command issued
     */
    time_t get_utc_time();

    /*!
     * \brief gets the Latitude, Longitude and Altitude vector from the last TC command issued
     */
    std::array<float, 3> get_LLH() const;

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

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue_;
    bool keep_running_;

    time_t receiver_utc_time_;

    float rx_latitude_;
    float rx_longitude_;
    float rx_altitude_;

    std::shared_ptr<PvtInterface> PVT_sptr_;
};

#endif /* GNSS_SDR_TCP_CMD_INTERFACE_H_ */
