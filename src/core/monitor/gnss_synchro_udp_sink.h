/*!
 * \file gnss_synchro_udp_sink.h
 * \brief Interface of a class that sends serialized Gnss_Synchro objects
 * over udp to one or multiple endponits
 * \author Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
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

#ifndef GNSS_SDR_GNSS_SYNCHRO_UDP_SINK_H
#define GNSS_SDR_GNSS_SYNCHRO_UDP_SINK_H

#include "gnss_synchro.h"
#include "serdes_gnss_synchro.h"
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <cstdint>
#include <string>
#include <vector>

/** \addtogroup Core
 * \{ */
/** \addtogroup Gnss_Synchro_Monitor
 * \{ */


#if USE_BOOST_ASIO_IO_CONTEXT
using b_io_context = boost::asio::io_context;
#else
using b_io_context = boost::asio::io_service;
#endif

/*!
 * \brief This class sends serialized Gnss_Synchro objects
 * over UDP to one or multiple endpoints.
 */
class Gnss_Synchro_Udp_Sink
{
public:
    Gnss_Synchro_Udp_Sink(const std::vector<std::string>& addresses, const uint16_t& port, bool enable_protobuf);
    bool write_gnss_synchro(const std::vector<Gnss_Synchro>& stocks);

private:
    b_io_context io_context;
    boost::asio::ip::udp::socket socket;
    boost::system::error_code error;
    std::vector<boost::asio::ip::udp::endpoint> endpoints;
    Serdes_Gnss_Synchro serdes;
    bool use_protobuf;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SYNCHRO_UDP_SINK_H
