/*!
 * \file rtklib_tls.h
 * \brief TLS client transport for RTKLIB streams
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2026 Carles Fernandez-Prades
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTKLIB_TLS_H
#define GNSS_SDR_RTKLIB_TLS_H

#include "rtklib.h"
#include <memory>
#include <string>

class Rtklib_Tls_Client
{
public:
    explicit Rtklib_Tls_Client(std::string hostname);
    ~Rtklib_Tls_Client();

    Rtklib_Tls_Client(const Rtklib_Tls_Client &) = delete;
    Rtklib_Tls_Client &operator=(const Rtklib_Tls_Client &) = delete;

    bool initialize(char *msg);
    void reset();

    // Return 1 when the TLS session is established, 0 while a non-blocking
    // handshake is in progress, and -1 on a fatal error.
    int handshake(socket_t sock, char *msg);

    // Return bytes transferred, 0 when the operation would block, and -1 on a
    // fatal TLS error or orderly peer shutdown.
    int read(unsigned char *buff, int n, char *msg);
    int write(const unsigned char *buff, int n, char *msg);

private:
    class Impl;
    std::unique_ptr<Impl> d_impl;
};

#endif  // GNSS_SDR_RTKLIB_TLS_H
