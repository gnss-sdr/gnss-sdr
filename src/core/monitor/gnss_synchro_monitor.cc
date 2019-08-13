/*!
 * \file gnss_synchro_monitor.cc
 * \brief Implementation of a receiver monitoring block which allows sending
 * a data stream with the receiver internal parameters (Gnss_Synchro objects)
 * to local or remote clients over UDP.
 *
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

#include "gnss_synchro_monitor.h"
#include "gnss_synchro.h"
#include <algorithm>
#include <iostream>
#include <utility>


gnss_synchro_monitor_sptr gnss_synchro_make_monitor(unsigned int n_channels,
    int decimation_factor,
    int udp_port,
    const std::vector<std::string>& udp_addresses,
    bool enable_protobuf)
{
    return gnss_synchro_monitor_sptr(new gnss_synchro_monitor(n_channels,
        decimation_factor,
        udp_port,
        udp_addresses,
        enable_protobuf));
}


gnss_synchro_monitor::gnss_synchro_monitor(unsigned int n_channels,
    int decimation_factor,
    int udp_port,
    const std::vector<std::string>& udp_addresses,
    bool enable_protobuf) : gr::sync_block("gnss_synchro_monitor",
                                gr::io_signature::make(n_channels, n_channels, sizeof(Gnss_Synchro)),
                                gr::io_signature::make(0, 0, 0))
{
    d_decimation_factor = decimation_factor;
    d_nchannels = n_channels;

    udp_sink_ptr = std::unique_ptr<Gnss_Synchro_Udp_Sink>(new Gnss_Synchro_Udp_Sink(udp_addresses, udp_port, enable_protobuf));

    count = 0;
}


int gnss_synchro_monitor::work(int noutput_items, gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items __attribute__((unused)))
{
    const auto** in = reinterpret_cast<const Gnss_Synchro**>(&input_items[0]);  // Get the input buffer pointer
    for (int epoch = 0; epoch < noutput_items; epoch++)
        {
            count++;
            if (count >= d_decimation_factor)
                {
                    for (unsigned int i = 0; i < d_nchannels; i++)
                        {
                            std::vector<Gnss_Synchro> stocks;
                            stocks.push_back(in[i][epoch]);
                            udp_sink_ptr->write_gnss_synchro(stocks);
                        }
                    count = 0;
                }
        }
    return noutput_items;
}
