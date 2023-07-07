/*!
 * \file ppstcprx.h
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

#ifndef SRC_LIBS_PPSTCPRX_H_
#define SRC_LIBS_PPSTCPRX_H_
#include "concurrent_queue.h"
#include "pps_samplestamp.h"
#include <arpa/inet.h>
#include <memory>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>

class pps_tcp_rx
{
private:
    std::shared_ptr<Concurrent_Queue<PpsSamplestamp>> Pps_queue;
    int clientSd;

public:
    volatile bool is_connected;
    pps_tcp_rx();
    virtual ~pps_tcp_rx();

    void receive_pps(std::string ip_address, int port);
    bool send_cmd(std::string cmd);

    void set_pps_samplestamp_queue(std::shared_ptr<Concurrent_Queue<PpsSamplestamp>> queue);
};

#endif /* SRC_LIBS_PPSTCPRX_H_ */
