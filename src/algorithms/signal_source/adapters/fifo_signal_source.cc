/*!
 * \file fifo_signal_source.cc
 *
 * \brief Implementation of the class for retrieving samples through a Unix FIFO
 * \author Malte Lenhart, 2021. malte.lenhart(at)mailbox.org
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

#include "fifo_signal_source.h"
#include "configuration_interface.h"
#include "fifo_reader.h"
#include "gnss_sdr_string_literals.h"
#include <glog/logging.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/blocks/file_source.h>


using namespace std::string_literals;

FifoSignalSource::FifoSignalSource(ConfigurationInterface const* configuration,
    std::string const& role, unsigned int in_streams, unsigned int out_streams,
    [[maybe_unused]] Concurrent_Queue<pmt::pmt_t>* queue)
    : SignalSourceBase(configuration, role, "Fifo_Signal_Source"s),
      item_size_(sizeof(gr_complex)),  // currenty output item size is always gr_complex
      fifo_reader_(FifoReader::make(configuration->property(role + ".filename", "../data/example_capture.dat"s),
          configuration->property(role + ".sample_type", "ishort"s))),
      dump_(configuration->property(role + ".dump", false)),
      dump_filename_(configuration->property(role + ".dump_filename", "./data/signal_source.dat"s))
{
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << (dump_filename_ + ".bin"s);
            file_sink_ = gr::blocks::file_sink::make(item_size_, (dump_filename_ + ".bin").c_str());
        }

    if (in_streams > 0)
        {
            LOG(ERROR) << "A signal source does not have an input stream";
        }
    if (out_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void FifoSignalSource::connect(gr::top_block_sptr top_block)
{
    // here we could add a throttle as done in the file_source_base if required
    if (dump_)
        {
            top_block->connect(fifo_reader_, 0, file_sink_, 0);
            DLOG(INFO) << "connected source to file sink";
        }
}


void FifoSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->disconnect(fifo_reader_, 0, file_sink_, 0);
            DLOG(INFO) << "disconnected source from file sink";
        }
}


size_t FifoSignalSource::item_size()
{
    return item_size_;
}


gr::basic_block_sptr FifoSignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::blocks::file_source::sptr();
}


gr::basic_block_sptr FifoSignalSource::get_right_block()
{
    return fifo_reader_;
}
