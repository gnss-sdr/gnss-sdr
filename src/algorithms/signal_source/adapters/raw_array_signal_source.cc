/*!
 * \file raw_array_signal_source.cc
 * \brief CTTC Experimental GNSS 8 channels array signal source
 * \author Javier Arribas, jarribas(at)cttc.es
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

#include "raw_array_signal_source.h"
#include "configuration_interface.h"
#include <dbfcttc/raw_array.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/msg_queue.h>
#include <glog/logging.h>


using google::LogMessage;

RawArraySignalSource::RawArraySignalSource(ConfigurationInterface* configuration,
    std::string role, unsigned int in_stream, unsigned int out_stream, gr::msg_queue::sptr queue) : role_(role), in_stream_(in_stream), out_stream_(out_stream), queue_(queue)
{
    std::string default_item_type = "gr_complex";
    std::string default_dump_file = "./data/raw_array_source.dat";
    item_type_ = configuration->property(role + ".item_type", default_item_type);


    //dump_ = configuration->property(role + ".dump", false);
    //dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    dump_ = false;

    std::string default_ethernet_dev = "eth0";
    eth_device_ = configuration->property(role + ".ethernet_dev", default_ethernet_dev);

    int channels_;
    channels_ = configuration->property(role + ".channels", 8);

    int snapshots_per_frame_;
    snapshots_per_frame_ = configuration->property(role + ".snapshots_per_frame", 80);

    int inter_frame_delay_;
    inter_frame_delay_ = configuration->property(role + ".inter_frame_delay", 10);

    int sampling_freq_;
    sampling_freq_ = configuration->property(role + ".sampling_freq", 5000000);

    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            raw_array_source_ = gr::dbfcttc::raw_array::make(eth_device_.c_str(), channels_, snapshots_per_frame_, inter_frame_delay_, sampling_freq_);
            DLOG(INFO) << "Item size " << item_size_;
            DLOG(INFO) << "raw_array_source(" << raw_array_source_->unique_id() << ")";
        }
    //    else if (item_type_.compare("short") == 0)
    //        {
    //            item_size_ = sizeof(short);
    //            resampler_ = direct_resampler_make_conditioner_ss(sample_freq_in_,
    //                    sample_freq_out_);
    //        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type for raw_array_source_";
            item_size_ = sizeof(gr_complex);
        }
    if (dump_)
        {
            //TODO: multichannel recorder
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
        }
    if (dump_)
        {
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }
}


RawArraySignalSource::~RawArraySignalSource()
{
}


void RawArraySignalSource::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            //TODO: multichannel recorder
            top_block->connect(raw_array_source_, 0, file_sink_, 0);
            DLOG(INFO) << "connected raw_array_source_ to file sink";
        }
    else
        {
            DLOG(INFO) << "nothing to connect internally";
        }
}


void RawArraySignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            //TODO: multichannel recorder
            top_block->disconnect(raw_array_source_, 0, file_sink_, 0);
        }
}


gr::basic_block_sptr RawArraySignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::block_sptr();
}


gr::basic_block_sptr RawArraySignalSource::get_right_block()
{
    return raw_array_source_;
}
