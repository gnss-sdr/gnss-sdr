/*!
 * \file gnss_sdr_valve.h
 * \brief  Interface of a GNU Radio block that sends a STOP message to the
 * control queue right after a specific number of samples have passed through it.
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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


#ifndef GNSS_SDR_GNSS_SDR_VALVE_H
#define GNSS_SDR_GNSS_SDR_VALVE_H

#include "concurrent_queue.h"
#include "gnss_block_interface.h"
#include <gnuradio/sync_block.h>  // for sync_block
#include <gnuradio/types.h>       // for gr_vector_const_void_star
#include <pmt/pmt.h>
#include <cstddef>  // for size_t
#include <cstdint>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_libs
 * \{ */


class Gnss_Sdr_Valve;

gnss_shared_ptr<Gnss_Sdr_Valve> gnss_sdr_make_valve(
    size_t sizeof_stream_item,
    uint64_t nitems,
    Concurrent_Queue<pmt::pmt_t>* queue);

gnss_shared_ptr<Gnss_Sdr_Valve> gnss_sdr_make_valve(
    size_t sizeof_stream_item,
    uint64_t nitems,
    Concurrent_Queue<pmt::pmt_t>* queue,
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
        gr_vector_const_void_star& input_items,
        gr_vector_void_star& output_items);

private:
    friend gnss_shared_ptr<Gnss_Sdr_Valve> gnss_sdr_make_valve(
        size_t sizeof_stream_item,
        uint64_t nitems,
        Concurrent_Queue<pmt::pmt_t>* queue);

    friend gnss_shared_ptr<Gnss_Sdr_Valve> gnss_sdr_make_valve(
        size_t sizeof_stream_item,
        uint64_t nitems,
        Concurrent_Queue<pmt::pmt_t>* queue,
        bool stop_flowgraph);

    Gnss_Sdr_Valve(size_t sizeof_stream_item,
        uint64_t nitems,
        Concurrent_Queue<pmt::pmt_t>* queue, bool stop_flowgraph);

    uint64_t d_nitems;
    uint64_t d_ncopied_items;
    Concurrent_Queue<pmt::pmt_t>* d_queue;
    bool d_stop_flowgraph;
    bool d_open_valve;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_SDR_VALVE_H
