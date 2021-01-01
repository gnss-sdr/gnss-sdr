/*!
 * \file gnss_synchro_udp_sink.cc
 * \brief Implementation of a class that sends serialized Gnss_Synchro
 * objects over udp to one or multiple endponits
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

#include "gnss_synchro_udp_sink.h"
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <iostream>
#include <sstream>

Gnss_Synchro_Udp_Sink::Gnss_Synchro_Udp_Sink(const std::vector<std::string>& addresses, const uint16_t& port, bool enable_protobuf) : socket{io_context}
{
    use_protobuf = enable_protobuf;
    if (enable_protobuf)
        {
            serdes = Serdes_Gnss_Synchro();
        }
    for (const auto& address : addresses)
        {
            boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(address, error), port);
            endpoints.push_back(endpoint);
        }
}


bool Gnss_Synchro_Udp_Sink::write_gnss_synchro(const std::vector<Gnss_Synchro>& stocks)
{
    std::string outbound_data;
    if (use_protobuf == false)
        {
            std::ostringstream archive_stream;
            boost::archive::binary_oarchive oa{archive_stream};
            oa << stocks;
            outbound_data = archive_stream.str();
        }
    else
        {
            outbound_data = serdes.createProtobuffer(stocks);
        }
    for (const auto& endpoint : endpoints)
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
