/*!
 * \file beamformer_filter.cc
 * \brief Interface of an adapter of a digital beamformer
 * \author Javier Arribas jarribas (at) cttc.es
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

#include "beamformer_filter.h"
#include "beamformer.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <gnuradio/blocks/file_sink.h>


BeamformerFilter::BeamformerFilter(
    const ConfigurationInterface* configuration, const std::string& role,
    unsigned int in_stream, unsigned int out_stream)
    : role_(role),
      in_stream_(in_stream),
      out_stream_(out_stream),
      dump_(configuration->property(role + ".dump", false))
{
    const std::string default_item_type("gr_complex");
    const std::string default_dump_file("./data/input_filter.dat");
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);
    DLOG(INFO) << "role " << role_;
    if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            beamformer_ = make_beamformer_sptr();
            DLOG(INFO) << "Item size " << item_size_;
            DLOG(INFO) << "resampler(" << beamformer_->unique_id() << ")";
        }
    else
        {
            LOG(WARNING) << item_type_
                         << " unrecognized item type for beamformer";
            item_size_ = 0;
        }
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }
    if (in_stream_ > 8)
        {
            LOG(ERROR) << "This implementation only supports eight input streams";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void BeamformerFilter::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->connect(beamformer_, 0, file_sink_, 0);
            DLOG(INFO) << "connected beamformer output to file sink";
        }
    else
        {
            DLOG(INFO) << "nothing to connect internally";
        }
}


void BeamformerFilter::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->disconnect(beamformer_, 0, file_sink_, 0);
        }
}


gr::basic_block_sptr BeamformerFilter::get_left_block()
{
    return beamformer_;
}


gr::basic_block_sptr BeamformerFilter::get_right_block()
{
    return beamformer_;
}
