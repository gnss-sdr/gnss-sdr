/*!
 * \file gnss_sdr_valve.cc
 * \brief Implementation of a GNU Radio block that sends a STOP message to the
 * control queue right after a specific number of samples have passed through it.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "gnss_sdr_valve.h"
#include <algorithm> // for min
#include <cstring>  // for memcpy
#include <gnuradio/io_signature.h>
#include "control_message_factory.h"
#include <glog/logging.h>

gnss_sdr_valve::gnss_sdr_valve (size_t sizeof_stream_item,
        unsigned long long nitems,
        gr::msg_queue::sptr queue) : gr::sync_block("valve",
                gr::io_signature::make(1, 1, sizeof_stream_item),
                gr::io_signature::make(1, 1, sizeof_stream_item) ),
                d_nitems(nitems), d_ncopied_items(0), d_queue(queue)
{}


boost::shared_ptr<gr::block> gnss_sdr_make_valve (size_t sizeof_stream_item, unsigned long long nitems, gr::msg_queue::sptr queue)
{
    boost::shared_ptr<gnss_sdr_valve> valve_(new gnss_sdr_valve(sizeof_stream_item, nitems, queue));
    return valve_;
}



int gnss_sdr_valve::work (int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    if (d_ncopied_items >= d_nitems)
        {
            ControlMessageFactory* cmf = new ControlMessageFactory();
            d_queue->handle(cmf->GetQueueMessage(200,0));
            LOG(INFO) << "Stoping reciver, "<< d_ncopied_items << " samples processed";
            delete cmf;
            return -1;    // Done!
        }
    unsigned long long n = std::min(d_nitems - d_ncopied_items, static_cast<long long unsigned int>(noutput_items));
    if (n == 0) return 0;
    memcpy (output_items[0], input_items[0], n * input_signature()->sizeof_stream_item(0));
    //for(long long i = 0; i++; i<n)
    //    {
    //        output_items[i] = input_items[i];
    //    }
    d_ncopied_items += n;
    return n;
}
