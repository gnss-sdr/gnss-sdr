/*!
 * \file gnss_sdr_valve.cc
 * \brief Implementation of a GNU Radio block that sends a STOP message to the
 * control queue right after a specific number of samples have passed through it.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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
#include <gnuradio/gr_io_signature.h>
#include "control_message_factory.h"
#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

gnss_sdr_valve::gnss_sdr_valve (size_t sizeof_stream_item,
        int nitems,
        gr_msg_queue_sptr queue) : gr_sync_block ("valve",
                gr_make_io_signature (1, 1, sizeof_stream_item),
                gr_make_io_signature (1, 1, sizeof_stream_item)),
                d_nitems (nitems), d_ncopied_items (0), d_queue(queue)
{}



gr_block_sptr gnss_sdr_make_valve (size_t sizeof_stream_item,
        int nitems,
        gr_msg_queue_sptr queue)
{
    return gr_block_sptr (new gnss_sdr_valve (sizeof_stream_item, nitems, queue));
}



int gnss_sdr_valve::work (int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    if (d_ncopied_items >= d_nitems)
        {

            ControlMessageFactory* cmf = new ControlMessageFactory();
            d_queue->handle(cmf->GetQueueMessage(200,0));
            delete cmf;
            return -1;	// Done!
        }
    unsigned n = std::min(d_nitems - d_ncopied_items, noutput_items);
    if (n == 0)
        return 0;
    memcpy (output_items[0], input_items[0], n * input_signature()->sizeof_stream_item (0));
    d_ncopied_items += n;
    return n;
}
