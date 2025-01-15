/*!
 * \file tcp_cmd_interface.h
 *
 * \brief Class that implements a TCP/IP telecommand command line interface
 * for GNSS-SDR
 * \author Javier Arribas jarribas (at) cttc.es
 *
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
#ifndef GNSS_SDR_TCP_CMD_INTERFACE_H
#define GNSS_SDR_TCP_CMD_INTERFACE_H


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

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver
 * \{ */


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
    time_t get_utc_time() const;

    /*!
     * \brief gets the Latitude, Longitude and Altitude vector from the last TC command issued
     */
    std::array<float, 3> get_LLH() const;

    void set_pvt(std::shared_ptr<PvtInterface> PVT_sptr);

private:
    std::unordered_map<std::string, std::function<std::string(const std::vector<std::string> &)>>
        functions_;
    std::string status(const std::vector<std::string> &commandLine);
    std::string reset(const std::vector<std::string> &commandLine);
    std::string standby(const std::vector<std::string> &commandLine);
    std::string hotstart(const std::vector<std::string> &commandLine);
    std::string warmstart(const std::vector<std::string> &commandLine);
    std::string coldstart(const std::vector<std::string> &commandLine);
    std::string set_ch_satellite(const std::vector<std::string> &commandLine);

    void register_functions();

    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> control_queue_;
    std::shared_ptr<PvtInterface> PVT_sptr_;

    float rx_latitude_;
    float rx_longitude_;
    float rx_altitude_;

    time_t receiver_utc_time_;

    bool keep_running_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_TCP_CMD_INTERFACE_H
