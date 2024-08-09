/*!
 * \file gnss_synchro_monitor.cc
 * \brief Implementation of a receiver monitoring block which allows sending
 * a data stream with the receiver internal parameters (Gnss_Synchro objects)
 * to local or remote clients over UDP.
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

#include "gnss_synchro_monitor.h"
#include "gnss_sdr_make_unique.h"
#include "gnss_synchro.h"
#include <algorithm>
#include <iostream>
#include <utility>


gnss_synchro_monitor_sptr gnss_synchro_make_monitor(int n_channels,
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


gnss_synchro_monitor::gnss_synchro_monitor(int n_channels,
    int decimation_factor,
    int udp_port,
    const std::vector<std::string>& udp_addresses,
    bool enable_protobuf)
    : gr::block("gnss_synchro_monitor",
          gr::io_signature::make(n_channels, n_channels, sizeof(Gnss_Synchro)),
          gr::io_signature::make(0, 0, 0)),
      count(0),
      d_nchannels(n_channels),
      d_decimation_factor(decimation_factor)
{
    udp_sink_ptr = std::make_unique<Gnss_Synchro_Udp_Sink>(udp_addresses, udp_port, enable_protobuf);
}


void gnss_synchro_monitor::forecast(int noutput_items __attribute__((unused)), gr_vector_int& ninput_items_required)
{
    for (int32_t channel_index = 0; channel_index < d_nchannels; channel_index++)
        {
            // Set the required number of inputs to 0 so that a lone input on any channel can be pushed to UDP
            ninput_items_required[channel_index] = 0;
        }
}


int gnss_synchro_monitor::general_work(int noutput_items __attribute__((unused)), gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items, gr_vector_void_star& output_items __attribute__((unused)))
{
    // Get the input buffer pointer
    const auto** in = reinterpret_cast<const Gnss_Synchro**>(&input_items[0]);

    // Loop through each input stream channel
    for (int channel_index = 0; channel_index < d_nchannels; channel_index++)
        {
            // Loop through each item in each input stream channel
            for (int item_index = 0; item_index < ninput_items[channel_index]; item_index++)
                {
                    // Use the count variable to limit how many items are sent per channel
                    count++;
                    if (count >= d_decimation_factor)
                        {
                            // Convert to a vector and write to the UDP sink
                            std::vector<Gnss_Synchro> stocks;
                            stocks.push_back(in[channel_index][item_index]);
                            udp_sink_ptr->write_gnss_synchro(stocks);
                            // Reset count variable
                            count = 0;
                            // Consume the number of items for the input stream channel
                            consume(channel_index, ninput_items[channel_index]);
                        }
                }
        }

    // Not producing any outputs
    return 0;
}
