/*!
 * \file gnss_synchro_udp_sink.cc
 * \brief Implementation of a class that sends serialized Gnss_Synchro
 * objects over udp to one or multiple endponits
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

#include "gnss_synchro_udp_sink.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <sstream>
#include <iostream>

Gnss_Synchro_Udp_Sink::Gnss_Synchro_Udp_Sink(std::vector<std::string> addresses, const unsigned short& port) : socket{io_service}
{
    for (auto address : addresses)
        {
            boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(address, error), port);
            endpoints.push_back(endpoint);
        }
}

bool Gnss_Synchro_Udp_Sink::write_gnss_synchro(std::vector<Gnss_Synchro> stocks)
{
    std::ostringstream archive_stream;
    boost::archive::binary_oarchive oa{archive_stream};
    oa << stocks;
    std::string outbound_data = archive_stream.str();

    for (auto endpoint : endpoints)
        {
            socket.open(endpoint.protocol(), error);
            socket.connect(endpoint, error);

            try
                {
                    socket.send(boost::asio::buffer(outbound_data));
                }
            catch (boost::system::system_error const& e)
                {
                    return false;
                }
        }
    return true;
}
