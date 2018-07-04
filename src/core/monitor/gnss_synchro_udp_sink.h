/*!
 * \file gnss_synchro_udp_sink.h
 * \brief Interface of a class that sends serialized Gnss_Synchro objects
 * over udp to one or multiple endponits
 * \author Álvaro Cebrián Juan, 2018. acebrianjuan(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SYNCHRO_UDP_SINK_H_
#define GNSS_SYNCHRO_UDP_SINK_H_

#include <boost/asio.hpp>
#include "gnss_synchro.h"

class Gnss_Synchro_Udp_Sink
{
public:
    Gnss_Synchro_Udp_Sink(std::vector<std::string> addresses, const unsigned short &port);
    bool write_gnss_synchro(std::vector<Gnss_Synchro> stocks);

private:
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;
    boost::system::error_code error;
    std::vector<boost::asio::ip::udp::endpoint> endpoints;
    std::vector<Gnss_Synchro> stocks;
};


#endif /* GNSS_SYNCHRO_UDP_SINK_H_ */
