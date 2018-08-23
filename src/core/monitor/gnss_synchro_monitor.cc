/*!
 * \file gnss_synchro_monitor.cc
 * \brief Interface of a Position Velocity and Time computation block
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

#include "gnss_synchro_monitor.h"
#include "gnss_synchro.h"
#include <glog/logging.h>
#include <algorithm>
#include <iostream>


using google::LogMessage;


gnss_synchro_monitor_sptr gnss_synchro_make_monitor(unsigned int n_channels,
    int output_rate_ms,
    int udp_port,
    std::vector<std::string> udp_addresses)
{
    return gnss_synchro_monitor_sptr(new gnss_synchro_monitor(n_channels,
        output_rate_ms,
        udp_port,
        udp_addresses));
}


gnss_synchro_monitor::gnss_synchro_monitor(unsigned int n_channels,
    int output_rate_ms,
    int udp_port,
    std::vector<std::string> udp_addresses) : gr::sync_block("gnss_synchro_monitor",
                                                  gr::io_signature::make(n_channels, n_channels, sizeof(Gnss_Synchro)),
                                                  gr::io_signature::make(0, 0, 0))
{
    d_output_rate_ms = output_rate_ms;
    d_nchannels = n_channels;

    udp_sink_ptr = std::unique_ptr<Gnss_Synchro_Udp_Sink>(new Gnss_Synchro_Udp_Sink(udp_addresses, udp_port));

    count = 0;
}


gnss_synchro_monitor::~gnss_synchro_monitor()
{
}


int gnss_synchro_monitor::work(int noutput_items, gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items __attribute__((unused)))
{
    const Gnss_Synchro** in = reinterpret_cast<const Gnss_Synchro**>(&input_items[0]);  // Get the input buffer pointer
    for (int epoch = 0; epoch < noutput_items; epoch++)
        {
            count++;
            if (count >= d_output_rate_ms)
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
