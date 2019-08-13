/*!
 * \file gnss_sdr_valve.cc
 * \brief Implementation of a GNU Radio block that sends a STOP message to the
 * control queue right after a specific number of samples have passed through it.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
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

#include "gnss_sdr_valve.h"
#include "command_event.h"
#include <glog/logging.h>           // for LOG
#include <gnuradio/io_signature.h>  // for io_signature
#include <algorithm>                // for min
#include <cstring>                  // for memcpy
#include <unistd.h>                 // for usleep
#include <utility>

Gnss_Sdr_Valve::Gnss_Sdr_Valve(size_t sizeof_stream_item,
    uint64_t nitems,
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue,
    bool stop_flowgraph) : gr::sync_block("valve",
                               gr::io_signature::make(1, 20, sizeof_stream_item),
                               gr::io_signature::make(1, 20, sizeof_stream_item)),
                           d_nitems(nitems),
                           d_ncopied_items(0),
                           d_queue(std::move(queue)),
                           d_stop_flowgraph(stop_flowgraph)
{
    d_open_valve = false;
}


boost::shared_ptr<Gnss_Sdr_Valve> gnss_sdr_make_valve(size_t sizeof_stream_item, uint64_t nitems, std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue, bool stop_flowgraph)
{
    boost::shared_ptr<Gnss_Sdr_Valve> valve_(new Gnss_Sdr_Valve(sizeof_stream_item, nitems, std::move(queue), stop_flowgraph));
    return valve_;
}


boost::shared_ptr<Gnss_Sdr_Valve> gnss_sdr_make_valve(size_t sizeof_stream_item, uint64_t nitems, std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue)
{
    boost::shared_ptr<Gnss_Sdr_Valve> valve_(new Gnss_Sdr_Valve(sizeof_stream_item, nitems, std::move(queue), true));
    return valve_;
}


void Gnss_Sdr_Valve::open_valve()
{
    d_open_valve = true;
}


int Gnss_Sdr_Valve::work(int noutput_items,
    gr_vector_const_void_star &input_items,
    gr_vector_void_star &output_items)
{
    if (d_open_valve == false)
        {
            if (d_ncopied_items >= d_nitems)
                {
                    LOG(INFO) << "Stopping receiver, " << d_ncopied_items << " samples processed";
                    d_queue->push(pmt::make_any(command_event_make(200, 0)));
                    if (d_stop_flowgraph)
                        {
                            return -1;  // Done!
                        }
                    usleep(1000000);
                    return 0;  // do not produce or consume
                }
            uint64_t n = std::min(d_nitems - d_ncopied_items, static_cast<uint64_t>(noutput_items));
            if (n == 0)
                {
                    return 0;
                }
            // multichannel support
            for (unsigned int ch = 0; ch < output_items.size(); ch++)
                {
                    memcpy(output_items[ch], input_items[ch], n * input_signature()->sizeof_stream_item(ch));
                }
            d_ncopied_items += n;
            return n;
        }
    for (unsigned int ch = 0; ch < output_items.size(); ch++)
        {
            memcpy(output_items[ch], input_items[ch], noutput_items * input_signature()->sizeof_stream_item(ch));
        }
    return noutput_items;
}
