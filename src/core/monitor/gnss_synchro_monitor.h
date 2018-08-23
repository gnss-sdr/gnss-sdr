/*!
 * \file gnss_synchro_monitor.h
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

#ifndef GNSS_SDR_GNSS_SYNCHRO_MONITOR_H
#define GNSS_SDR_GNSS_SYNCHRO_MONITOR_H


#include "gnss_synchro_udp_sink.h"
#include <gnuradio/sync_block.h>
#include <fstream>
#include <utility>
#include <string>


class gnss_synchro_monitor;

typedef boost::shared_ptr<gnss_synchro_monitor> gnss_synchro_monitor_sptr;

gnss_synchro_monitor_sptr gnss_synchro_make_monitor(unsigned int n_channels,
    int output_rate_ms,
    int udp_port,
    std::vector<std::string> udp_addresses);

/*!
 * \brief This class implements a block that computes the PVT solution with Galileo E1 signals
 */
class gnss_synchro_monitor : public gr::sync_block
{
private:
    friend gnss_synchro_monitor_sptr gnss_synchro_make_monitor(unsigned int nchannels,
        int output_rate_ms,
        int udp_port,
        std::vector<std::string> udp_addresses);

    unsigned int d_nchannels;

    int d_output_rate_ms;

    std::unique_ptr<Gnss_Synchro_Udp_Sink> udp_sink_ptr;

    unsigned int count;


public:
    gnss_synchro_monitor(unsigned int nchannels,
        int output_rate_ms,
        int udp_port,
        std::vector<std::string> udp_addresses);

    ~gnss_synchro_monitor();  //!< Default destructor

    int work(int noutput_items, gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);
};

#endif
