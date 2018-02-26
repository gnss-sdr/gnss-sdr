/*!
 * \file gen_signal_source.cc
 * \brief It wraps blocks that generates synthesized GNSS signal and filters
 *  it.
 * \author Marc Molina, 2013. marc.molina.pena@gmail.com
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

#include "gen_signal_source.h"
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <gnuradio/io_signature.h>
#include <gnuradio/message.h>
#include <glog/logging.h>
#include <sstream>

using google::LogMessage;

// Constructor
GenSignalSource::GenSignalSource(GNSSBlockInterface *signal_generator, GNSSBlockInterface *filter,
        std::string role, boost::shared_ptr<gr::msg_queue> queue) :
    signal_generator_(signal_generator),
    filter_(filter),
    role_(role),
    queue_(queue)
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

