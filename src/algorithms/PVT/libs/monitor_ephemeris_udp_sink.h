/*!
 * \file monitor_ephemeris_udp_sink.h
 * \brief Interface of a class that sends serialized Gps_Ephemeris and
 * Galileo_Ephemeris objects over udp to one or multiple endpoints.
 * \author Javier Arribas, 2021. jarribas(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_MONITOR_EPHEMERIS_UDP_SINK_H
#define GNSS_SDR_MONITOR_EPHEMERIS_UDP_SINK_H

#include "galileo_ephemeris.h"
#include "gps_ephemeris.h"
#include "serdes_galileo_eph.h"
#include "serdes_gps_eph.h"
#include <boost/asio.hpp>
#include <memory>
#include <string>
#include <vector>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


#if USE_BOOST_ASIO_IO_CONTEXT
using b_io_context = boost::asio::io_context;
#else
using b_io_context = boost::asio::io_service;
#endif

class Monitor_Ephemeris_Udp_Sink
{
public:
    Monitor_Ephemeris_Udp_Sink(const std::vector<std::string>& addresses, const uint16_t& port, bool protobuf_enabled);
    bool write_gps_ephemeris(const std::shared_ptr<Gps_Ephemeris>& monitor_gps_eph);
    bool write_galileo_ephemeris(const std::shared_ptr<Galileo_Ephemeris>& monitor_gal_eph);

private:
    Serdes_Galileo_Eph serdes_gal;
    Serdes_Gps_Eph serdes_gps;
    b_io_context io_context;
    boost::asio::ip::udp::socket socket;
    std::vector<boost::asio::ip::udp::endpoint> endpoints;
    boost::system::error_code error;
    bool use_protobuf;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_MONITOR_EPHEMERIS_UDP_SINK_H
