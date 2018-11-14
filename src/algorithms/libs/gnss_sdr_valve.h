/*!
 * \file gnss_sdr_valve.h
 * \brief  Interface of a GNU Radio block that sends a STOP message to the
 * control queue right after a specific number of samples have passed through it.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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


#ifndef GNSS_SDR_GNSS_SDR_VALVE_H_
#define GNSS_SDR_GNSS_SDR_VALVE_H_

#include <gnuradio/sync_block.h>
#include <gnuradio/msg_queue.h>
#include <boost/shared_ptr.hpp>

boost::shared_ptr<gr::block> gnss_sdr_make_valve(size_t sizeof_stream_item,
    unsigned long long nitems,
    gr::msg_queue::sptr queue);
boost::shared_ptr<gr::block> gnss_sdr_make_valve(size_t sizeof_stream_item,
    unsigned long long nitems,
    gr::msg_queue::sptr queue,
    bool stop_flowgraph);
/*!
 * \brief Implementation of a GNU Radio block that sends a STOP message to the
 * control queue right after a specific number of samples have passed through it.
 */
class gnss_sdr_valve : public gr::sync_block
{
    friend boost::shared_ptr<gr::block> gnss_sdr_make_valve(size_t sizeof_stream_item,
        unsigned long long nitems,
        gr::msg_queue::sptr queue);
    friend boost::shared_ptr<gr::block> gnss_sdr_make_valve(size_t sizeof_stream_item,
        unsigned long long nitems,
        gr::msg_queue::sptr queue,
        bool stop_flowgraph);


    unsigned long long d_nitems;
    unsigned long long d_ncopied_items;
    gr::msg_queue::sptr d_queue;
    bool d_stop_flowgraph;
    bool d_open_valve;

public:
    gnss_sdr_valve(size_t sizeof_stream_item,
        unsigned long long nitems,
        gr::msg_queue::sptr queue, bool stop_flowgraph);
    void open_valve();

    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);
};

#endif /*GNSS_SDR_GNSS_SDR_VALVE_H_*/
