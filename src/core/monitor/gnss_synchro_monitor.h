/*!
 * \file gnss_synchro_monitor.h
 * \brief Interface of a receiver monitoring block which allows sending
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
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GNSS_SYNCHRO_MONITOR_H
#define GNSS_SDR_GNSS_SYNCHRO_MONITOR_H

#include "gnss_synchro_udp_sink.h"
#include <boost/shared_ptr.hpp>
#include <gnuradio/runtime_types.h>  // for gr_vector_void_star
#include <gnuradio/sync_block.h>
#include <memory>
#include <string>
#include <vector>


class gnss_synchro_monitor;

using gnss_synchro_monitor_sptr = boost::shared_ptr<gnss_synchro_monitor>;

gnss_synchro_monitor_sptr gnss_synchro_make_monitor(unsigned int n_channels,
    int decimation_factor,
    int udp_port,
    const std::vector<std::string>& udp_addresses,
    bool enable_protobuf);

/*!
 * \brief This class implements a monitoring block which allows sending
 * a data stream with the receiver internal parameters (Gnss_Synchro objects)
 * to local or remote clients over UDP.
 */
class gnss_synchro_monitor : public gr::sync_block
{
public:
    ~gnss_synchro_monitor() = default;  //!< Default destructor

    int work(int noutput_items, gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);

private:
    friend gnss_synchro_monitor_sptr gnss_synchro_make_monitor(unsigned int n_channels,
        int decimation_factor,
        int udp_port,
        const std::vector<std::string>& udp_addresses,
        bool enable_protobuf);

    gnss_synchro_monitor(unsigned int n_channels,
        int decimation_factor,
        int udp_port,
        const std::vector<std::string>& udp_addresses,
        bool enable_protobuf);

    unsigned int d_nchannels;
    int d_decimation_factor;
    std::unique_ptr<Gnss_Synchro_Udp_Sink> udp_sink_ptr;
    int count;
};

#endif
