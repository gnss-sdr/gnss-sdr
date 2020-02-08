/*!
 * \file gen_signal_source.cc
 * \brief It wraps blocks that generates synthesized GNSS signal and filters
 *  it.
 * \author Marc Molina, 2013. marc.molina.pena@gmail.com
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
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#include "gen_signal_source.h"
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <gnuradio/message.h>
#include <sstream>
#include <utility>


// Constructor
GenSignalSource::GenSignalSource(GNSSBlockInterface *signal_generator, GNSSBlockInterface *filter,
    std::string role, std::shared_ptr<Concurrent_Queue<pmt::pmt_t>> queue) : signal_generator_(signal_generator),
                                                                             filter_(filter),
                                                                             role_(std::move(role)),
                                                                             queue_(std::move(queue))
{
    connected_ = false;
}


// Destructor
GenSignalSource::~GenSignalSource()
{
    delete signal_generator_;
    delete filter_;
}


void GenSignalSource::connect(gr::top_block_sptr top_block)
{
    if (connected_)
        {
            LOG(WARNING) << "Signal conditioner already connected internally";
            return;
        }

    signal_generator_->connect(top_block);
    filter_->connect(top_block);

    top_block->connect(signal_generator_->get_right_block(), 0,
        filter_->get_left_block(), 0);

    DLOG(INFO) << "signal_generator -> filter";

    connected_ = true;
}


void GenSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (!connected_)
        {
            LOG(WARNING) << "Signal conditioner already disconnected internally";
            return;
        }

    top_block->disconnect(signal_generator_->get_right_block(), 0,
        filter_->get_left_block(), 0);

    signal_generator_->disconnect(top_block);
    filter_->disconnect(top_block);

    connected_ = false;
}


gr::basic_block_sptr GenSignalSource::get_left_block()
{
    return signal_generator_->get_left_block();
}


gr::basic_block_sptr GenSignalSource::get_right_block()
{
    return filter_->get_right_block();
}
