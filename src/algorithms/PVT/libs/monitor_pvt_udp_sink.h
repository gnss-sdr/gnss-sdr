/*!
 * \file monitor_pvt_udp_sink.h
 * \brief Interface of a class that sends serialized Monitor_Pvt objects
 * over udp to one or multiple endpoints
 * \author Álvaro Cebrián Juan, 2019. acebrianjuan(at)gmail.com
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

#ifndef GNSS_SDR_MONITOR_PVT_UDP_SINK_H_
#define GNSS_SDR_MONITOR_PVT_UDP_SINK_H_

#include "monitor_pvt.h"
#include "serdes_monitor_pvt.h"
#include <boost/asio.hpp>

#if BOOST_GREATER_1_65
using b_io_context = boost::asio::io_context;
#else
using b_io_context = boost::asio::io_service;
#endif

class Monitor_Pvt_Udp_Sink
{
public:
    Monitor_Pvt_Udp_Sink(const std::vector<std::string>& addresses, const uint16_t& port, bool protobuf_enabled);
    bool write_monitor_pvt(const std::shared_ptr<Monitor_Pvt>& monitor_pvt);

private:
    b_io_context io_context;
    boost::asio::ip::udp::socket socket;
    boost::system::error_code error;
    std::vector<boost::asio::ip::udp::endpoint> endpoints;
    Serdes_Monitor_Pvt serdes;
    bool use_protobuf;
};


#endif /* GNSS_SDR_MONITOR_PVT_UDP_SINK_H_ */
