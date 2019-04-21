/*!
 * \file monitor_pvt_udp_sink.cc
 * \brief Implementation of a class that sends serialized Monitor_Pvt
 * objects over udp to one or multiple endpoints
 * \author Álvaro Cebrián Juan, 2019. acebrianjuan(at)gmail.com
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

#include "monitor_pvt_udp_sink.h"
#include <boost/archive/binary_oarchive.hpp>
#include <iostream>
#include <sstream>


Monitor_Pvt_Udp_Sink::Monitor_Pvt_Udp_Sink(std::vector<std::string> addresses, const uint16_t& port, bool protobuf_enabled) : socket{io_service}
{
    for (const auto& address : addresses)
        {
            boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(address, error), port);
            endpoints.push_back(endpoint);
        }
    monitor_pvt.TOW_at_current_symbol_ms = 0U;
    monitor_pvt.week = 0U;
    monitor_pvt.RX_time = 0.0;
    monitor_pvt.user_clk_offset = 0.0;
    monitor_pvt.pos_x = 0.0;
    monitor_pvt.pos_y = 0.0;
    monitor_pvt.pos_z = 0.0;
    monitor_pvt.vel_x = 0.0;
    monitor_pvt.vel_y = 0.0;
    monitor_pvt.vel_z = 0.0;
    monitor_pvt.cov_xx = 0.0;
    monitor_pvt.cov_yy = 0.0;
    monitor_pvt.cov_zz = 0.0;
    monitor_pvt.cov_xy = 0.0;
    monitor_pvt.cov_yz = 0.0;
    monitor_pvt.cov_zx = 0.0;
    monitor_pvt.latitude = 0.0;
    monitor_pvt.longitude = 0.0;
    monitor_pvt.height = 0.0;
    monitor_pvt.valid_sats = 0;
    monitor_pvt.solution_status = 0;
    monitor_pvt.solution_type = 0;
    monitor_pvt.AR_ratio_factor = 0.0;
    monitor_pvt.AR_ratio_threshold = 0.0;
    monitor_pvt.gdop = 0.0;
    monitor_pvt.pdop = 0.0;
    monitor_pvt.hdop = 0.0;
    monitor_pvt.vdop = 0.0;

    use_protobuf = protobuf_enabled;
    if (use_protobuf)
        {
            serdes = Serdes_Monitor_Pvt();
        }
}


bool Monitor_Pvt_Udp_Sink::write_monitor_pvt(const Monitor_Pvt& monitor_pvt)
{
    std::string outbound_data;
    if (use_protobuf == false)
        {
            std::ostringstream archive_stream;
            boost::archive::binary_oarchive oa{archive_stream};
            oa << monitor_pvt;
            outbound_data = archive_stream.str();
        }
    else
        {
            outbound_data = serdes.createProtobuffer(monitor_pvt);
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
