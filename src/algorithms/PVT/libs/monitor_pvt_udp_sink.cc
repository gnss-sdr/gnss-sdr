/*!
 * \file monitor_pvt_udp_sink.cc
 * \brief Implementation of a class that sends serialized Monitor_Pvt
 * objects over udp to one or multiple endpoints
 * \author Álvaro Cebrián Juan, 2019. acebrianjuan(at)gmail.com
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

#include "monitor_pvt_udp_sink.h"
#include <boost/archive/binary_oarchive.hpp>
#include <iostream>
#include <sstream>


Monitor_Pvt_Udp_Sink::Monitor_Pvt_Udp_Sink(const std::vector<std::string>& addresses,
    const uint16_t& port,
    bool protobuf_enabled) : socket{io_context},
                             use_protobuf(protobuf_enabled)
{
    for (const auto& address : addresses)
        {
            boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(address, error), port);
            endpoints.push_back(endpoint);
        }

    if (use_protobuf)
        {
            serdes = Serdes_Monitor_Pvt();
        }
}


bool Monitor_Pvt_Udp_Sink::write_monitor_pvt(const Monitor_Pvt* const monitor_pvt)
{
    std::string outbound_data;
    if (use_protobuf == false)
        {
            std::ostringstream archive_stream;
            boost::archive::binary_oarchive oa{archive_stream};
            oa << *monitor_pvt;
            outbound_data = archive_stream.str();
        }
    else
        {
            outbound_data = serdes.createProtobuffer(monitor_pvt);
        }

    for (const auto& endpoint : endpoints)
        {
            socket.open(endpoint.protocol(), error);

            try
                {
                    if (socket.send_to(boost::asio::buffer(outbound_data), endpoint) == 0)
                        {
                            return false;
                        }
                }
            catch (boost::system::system_error const& e)
                {
                    return false;
                }
        }
    return true;
}
