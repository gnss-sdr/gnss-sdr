/*!
 * \file udp_listener.cc
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

#include "udp_listener.h"
#include "gnss_synchro.pb.h"
#include "monitor_pvt.pb.h"
#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>

UdpListener::UdpListener(DataStore& store,
    const std::string& address,
    unsigned short gnss_port,
    unsigned short pvt_port)
    : store_(store),
      address_(address),
      gnss_port_(gnss_port),
      pvt_port_(pvt_port) {}

UdpListener::~UdpListener()
{
    stop();
}

int UdpListener::create_socket(const std::string& address, unsigned short port)
{
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
        {
            return -1;
        }

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(address.c_str());

    if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            close(fd);
            return -1;
        }

    return fd;
}

void UdpListener::start()
{
    running_ = true;
    gnss_thread_ = std::make_unique<std::thread>(&UdpListener::listen_gnss, this);
    pvt_thread_ = std::make_unique<std::thread>(&UdpListener::listen_pvt, this);
}

void UdpListener::stop()
{
    running_ = false;
    if (gnss_thread_ && gnss_thread_->joinable())
        {
            gnss_thread_->join();
        }
    if (pvt_thread_ && pvt_thread_->joinable())
        {
            pvt_thread_->join();
        }
}

void UdpListener::listen_gnss()
{
    const int fd = create_socket(address_, gnss_port_);
    if (fd < 0)
        {
            std::cerr << "Failed to create GnssSynchro socket on "
                      << address_ << ":" << gnss_port_ << '\n';
            return;
        }

    char buffer[65536];
    while (running_)
        {
            struct sockaddr_in sender;
            socklen_t sender_len = sizeof(sender);
            const ssize_t len = recvfrom(fd, buffer, sizeof(buffer), 0,
                reinterpret_cast<struct sockaddr*>(&sender), &sender_len);
            if (len <= 0)
                {
                    continue;
                }

            gnss_sdr::Observables observables;
            if (observables.ParseFromArray(buffer, static_cast<int>(len)))
                {
                    for (const auto& obs : observables.observable())
                        {
                            SatelliteData s;
                            s.system = obs.system();
                            s.signal = obs.signal();
                            s.prn = obs.prn();
                            s.channel_id = obs.channel_id();
                            s.cn0_db_hz = obs.cn0_db_hz();
                            s.carrier_doppler_hz = obs.carrier_doppler_hz();
                            s.carrier_phase_rads = obs.carrier_phase_rads();
                            s.pseudorange_m = obs.pseudorange_m();
                            s.flag_valid_pseudorange = obs.flag_valid_pseudorange();
                            s.flag_pll_locked = obs.flag_pll_180_deg_phase_locked();
                            s.flag_cycle_slip = obs.flag_cycle_slip();
                            store_.update_satellite(s.channel_id, s);
                        }
                }
        }

    close(fd);
}

void UdpListener::listen_pvt()
{
    const int fd = create_socket(address_, pvt_port_);
    if (fd < 0)
        {
            std::cerr << "Failed to create PVT socket on "
                      << address_ << ":" << pvt_port_ << '\n';
            return;
        }

    char buffer[65536];
    while (running_)
        {
            struct sockaddr_in sender;
            socklen_t sender_len = sizeof(sender);
            const ssize_t len = recvfrom(fd, buffer, sizeof(buffer), 0,
                reinterpret_cast<struct sockaddr*>(&sender), &sender_len);
            if (len <= 0)
                {
                    continue;
                }

            gnss_sdr::MonitorPvt pvt_msg;
            if (pvt_msg.ParseFromArray(buffer, static_cast<int>(len)))
                {
                    PvtData p;
                    p.latitude = pvt_msg.latitude();
                    p.longitude = pvt_msg.longitude();
                    p.height = pvt_msg.height();
                    p.hdop = pvt_msg.hdop();
                    p.vdop = pvt_msg.vdop();
                    p.pdop = pvt_msg.pdop();
                    p.gdop = pvt_msg.gdop();
                    p.valid_sats = pvt_msg.valid_sats();
                    p.utc_time = pvt_msg.utc_time();
                    store_.update_pvt(p);
                }
        }

    close(fd);
}
