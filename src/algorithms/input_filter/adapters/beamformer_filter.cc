/*!
 * \file beamformer_filter.cc
 * \brief Interface of an adapter of a digital beamformer
 * \author Javier Arribas jarribas (at) cttc.es
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

#include "beamformer_filter.h"
#include "beamformer.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <gnuradio/blocks/file_sink.h>


using google::LogMessage;

BeamformerFilter::BeamformerFilter(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_stream, unsigned int out_stream) : role_(role), in_stream_(in_stream), out_stream_(out_stream)
{
    std::string default_item_type = "gr_complex";
    std::string default_dump_file = "./data/input_filter.dat";
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    dump_ = configuration->property(role + ".dump", false);
    DLOG(INFO) << "dump_ is " << dump_;
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            beamformer_ = make_beamformer();
            DLOG(INFO) << "Item size " << item_size_;
            DLOG(INFO) << "resampler(" << beamformer_->unique_id() << ")";
        }
    else
        {
            LOG(WARNING) << item_type_
                         << " unrecognized item type for beamformer";
            item_size_ = sizeof(gr_complex);
        }
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }
    samples_ = 0;
}


BeamformerFilter::~BeamformerFilter() {}


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
