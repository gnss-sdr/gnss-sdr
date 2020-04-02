/*!
 * \file gnss_sdr_valve.h
 * \brief  Interface of a GNU Radio block that sends a STOP message to the
 * control queue right after a specific number of samples have passed through it.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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


#ifndef GNSS_SDR_GNSS_SDR_VALVE_H
#define GNSS_SDR_GNSS_SDR_VALVE_H

#include "concurrent_queue.h"
#include <memory>
#include <gnuradio/sync_block.h>  // for sync_block
#include <gnuradio/types.h>       // for gr_vector_const_void_star
#include <pmt/pmt.h>
#include <cstddef>  // for size_t
#include <cstdint>
#include <memory>

class Gnss_Sdr_Valve;

std::shared_ptr<Gnss_Sdr_Valve> gnss_sdr_make_valve(
    size_t sizeof_stream_item,
    uint64_t nitems,
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

std::shared_ptr<Gnss_Sdr_Valve> gnss_sdr_make_valve(
    size_t sizeof_stream_item,
    uint64_t nitems,
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue,
    bool stop_flowgraph);

/*!
 * \brief Implementation of a GNU Radio block that sends a STOP message to the
 * control queue right after a specific number of samples have passed through it.
 */
class Gnss_Sdr_Valve : public gr::sync_block
{
public:
    void open_valve();

    int work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items);

private:
    friend std::shared_ptr<Gnss_Sdr_Valve> gnss_sdr_make_valve(
        size_t sizeof_stream_item,
        uint64_t nitems,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue);

    friend std::shared_ptr<Gnss_Sdr_Valve> gnss_sdr_make_valve(
        size_t sizeof_stream_item,
        uint64_t nitems,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue,
        bool stop_flowgraph);

    Gnss_Sdr_Valve(size_t sizeof_stream_item,
        uint64_t nitems,
        std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue, bool stop_flowgraph);

    uint64_t d_nitems;
    uint64_t d_ncopied_items;
    std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> d_queue;
    bool d_stop_flowgraph;
    bool d_open_valve;
};

#endif  // GNSS_SDR_GNSS_SDR_VALVE_H
