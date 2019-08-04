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


#endif /* GNSS_SDR_GNSS_SYNCHRO_UDP_SINK_H_ */
