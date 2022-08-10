/*!
 * \file zmq_signal_source.h
 * \brief Signal source which reads from ZeroMQ.
 * \author Jim Melton, 2022. jim.melton(at)sncorp.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ZMQ_SIGNAL_SOURCE_H
#define GNSS_SDR_ZMQ_SIGNAL_SOURCE_H

#include "signal_source_base.h"
//
#include "concurrent_queue.h"
#include <gnuradio/zeromq/sub_source.h>
#include <pmt/pmt.h>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */

//! This class supports the following properties:
//!
//!   .pass_tags  - boolean flag if tags should be propagated (default false)
//!   .timeout_ms - receive timeout, in milliseconds (default 100)
//!   .hwm        - ZMQ high water mark (default -1, ZMQ default)
//!   .vlen       - vector length of the input items (default 1, one item)
//!   .endpointN  - the ZMQ endpoint to be connected to (repeat for each channel)
//!
//!   .item_type - data type of the samples (default "gr_complex")
//!
//! (probably should be abstracted to the base class)
//!
//!   .dump     - whether to archive input data
//!
//!   .dump_filename - if dumping, path to file for output
//!

class ZmqSignalSource : public SignalSourceBase
{
public:
    ZmqSignalSource(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_stream,
        unsigned int out_stream,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~ZmqSignalSource() = default;

    size_t item_size() override;

    auto connect(gr::top_block_sptr top_block) -> void override;
    auto disconnect(gr::top_block_sptr top_block) -> void override;
    auto get_right_block() -> gr::basic_block_sptr override;
    auto get_right_block(int RF_channel) -> gr::basic_block_sptr override;

private:
    std::vector<gr::zeromq::sub_source::sptr> d_source_blocks;

    size_t d_item_size;
};

/** \} */
/** \} */
#endif
