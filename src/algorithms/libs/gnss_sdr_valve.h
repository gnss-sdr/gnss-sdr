/*!
 * \file gnss_sdr_valve.h
 * \brief  Interface of a GNU Radio block that sends a STOP message to the
 * control queue right after a specific number of samples have passed through it.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
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


#ifndef GNSS_SDR_GNSS_SDR_VALVE_H_
#define GNSS_SDR_GNSS_SDR_VALVE_H_

#include <cstring>
#include <gr_sync_block.h>
#include <gr_msg_queue.h>

gr_block_sptr gnss_sdr_make_valve (size_t sizeof_stream_item,
        int nitems,
        gr_msg_queue_sptr queue);

class gnss_sdr_valve : public gr_sync_block
{
    friend gr_block_sptr gnss_sdr_make_valve(size_t sizeof_stream_item,
            int nitems,
            gr_msg_queue_sptr queue);
    gnss_sdr_valve (size_t sizeof_stream_item,
            int nitems,
            gr_msg_queue_sptr queue);
    int	d_nitems;
    int	d_ncopied_items;
    gr_msg_queue_sptr d_queue;

public:
    int work (int noutput_items,
            gr_vector_const_void_star &input_items,
            gr_vector_void_star &output_items);
};

#endif /*GNSS_SDR_GNSS_SDR_VALVE_H_*/
