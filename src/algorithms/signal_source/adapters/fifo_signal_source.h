/*!
 * \file fifo_signal_source.h
 *
 * \brief Header file of the class for retrieving samples through a Unix FIFO
 * \author Malte Lenhart, 2021. malte.lenhart(at)mailbox.org
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2021  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_FIFO_SIGNAL_SOURCE_H
#define GNSS_SDR_FIFO_SIGNAL_SOURCE_H

#include "concurrent_queue.h"
#include "signal_source_base.h"
#include <pmt/pmt.h>
#include <cstddef>
#include <string>

/** \addtogroup Signal_Source
 * \{ */
/** \addtogroup Signal_Source_adapters
 * \{ */

// forward declaration to avoid include in header
class ConfigurationInterface;

//! \brief Class that reads a sample stream from a Unix FIFO.
//!
//! This class supports the following properties:
//!
//!   .filename - the path to the input file
//!             - may be overridden by the -signal_source or -s command-line arguments
//!
//!   .sample_type - data type read out from the FIFO. default ishort ;
//!                - note: not output format. that is always gr_complex
//!
//!   .dump     - whether to archive input data
//!
//!   .dump_filename - if dumping, path to file for output
//!
class FifoSignalSource : public SignalSourceBase
{
public:
    FifoSignalSource(const ConfigurationInterface* configuration, const std::string& role,
        unsigned int in_streams, unsigned int out_streams,
        Concurrent_Queue<pmt::pmt_t>* queue);

    ~FifoSignalSource() = default;

    //! override methods from GNSSBlockInterface
    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    size_t item_size() override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

protected:
private:
    //! output size - always gr_complex
    const size_t item_size_;
    //! internal fifo_reader_ class acts as signal source
    const gnss_shared_ptr<gr::block> fifo_reader_;

    gnss_shared_ptr<gr::block> file_sink_;
    const bool dump_;
    const std::string dump_filename_;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_FIFO_SIGNAL_SOURCE_H
