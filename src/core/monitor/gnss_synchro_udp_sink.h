/*!
 * \file gnss_synchro_udp_sink.h
 * \brief Interface of a class that sends serialized Gnss_Synchro objects
 * over udp to one or multiple endponits
 * \author Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GNSS_SYNCHRO_UDP_SINK_H_
#define GNSS_SDR_GNSS_SYNCHRO_UDP_SINK_H_

#include "gnss_synchro.h"
#include "serdes_gnss_synchro.h"
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <cstdint>
#include <string>
#include <vector>

#if BOOST_GREATER_1_65
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
    std::vector<Gnss_Synchro> stocks;
    Serdes_Gnss_Synchro serdes;
    bool use_protobuf;
};


#endif  // GNSS_SDR_GNSS_SYNCHRO_UDP_SINK_H_
