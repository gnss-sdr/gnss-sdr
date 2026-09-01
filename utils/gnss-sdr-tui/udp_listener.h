/*!
 * \file udp_listener.h
 * \brief Asynchronous UDP listener for GNSS-SDR monitor streams.
 * \author Generated for GNSS-SDR contributors
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2025 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_TUI_UDP_LISTENER_H
#define GNSS_SDR_TUI_UDP_LISTENER_H

#include "data_store.h"
#include <atomic>
#include <memory>
#include <string>
#include <thread>

class UdpListener
{
public:
    UdpListener(DataStore& store,
        const std::string& address,
        unsigned short gnss_port,
        unsigned short pvt_port);
    ~UdpListener();

    UdpListener(const UdpListener&) = delete;
    UdpListener& operator=(const UdpListener&) = delete;
    UdpListener(UdpListener&&) = delete;
    UdpListener& operator=(UdpListener&&) = delete;

    void start();
    void stop();

private:
    int create_socket(const std::string& address, unsigned short port);
    void listen_gnss();
    void listen_pvt();

    DataStore& store_;
    std::string address_;
    unsigned short gnss_port_;
    unsigned short pvt_port_;
    std::atomic<bool> running_{false};
    std::unique_ptr<std::thread> gnss_thread_;
    std::unique_ptr<std::thread> pvt_thread_;
};

#endif  // GNSS_SDR_TUI_UDP_LISTENER_H
