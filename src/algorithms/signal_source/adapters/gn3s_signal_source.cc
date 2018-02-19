/*!
 * \file gn3s_signal_source.cc
 * \brief GN3S USB dongle GPS RF front-end signal sampler driver
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

#include "gn3s_signal_source.h"
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/msg_queue.h>
#include <glog/logging.h>
#include <gn3s/gn3s_source_cc.h>
#include "configuration_interface.h"


using google::LogMessage;

Gn3sSignalSource::Gn3sSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream, unsigned int out_stream, gr::msg_queue::sptr queue) :
        role_(role), in_stream_(in_stream), out_stream_(out_stream), queue_(queue)
{
    std::string default_item_type = "short";
    std::string default_dump_file = "./data/gn3s_source.dat";
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);

    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            gn3s_source_ = gn3s_make_source_cc();
            DLOG(INFO) << "Item size " << item_size_;
            DLOG(INFO) << "gn3s_source(" << gn3s_source_->unique_id() << ")";

        }
    //    else if (item_type_.compare("short") == 0)
    //        {
    //            item_size_ = sizeof(short);
    //            resampler_ = direct_resampler_make_conditioner_ss(sample_freq_in_,
    //                    sample_freq_out_);
    //        }
    else
        {
            LOG(WARNING) << item_type_
                    << " unrecognized item type for resampler";
            item_size_ = sizeof(short);
        }
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
        }
    if (dump_)
        {
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }
}



Gn3sSignalSource::~Gn3sSignalSource()
{}



void Gn3sSignalSource::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->connect(gn3s_source_, 0, file_sink_, 0);
            DLOG(INFO) << "connected gn3s_source to file sink";
        }
    else
        {
            DLOG(INFO) << "nothing to connect internally";
        }
}



void Gn3sSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->disconnect(gn3s_source_, 0, file_sink_, 0);
        }
}


gr::basic_block_sptr Gn3sSignalSource::get_left_block()
{
    LOG(WARNING) << "Left block of a signal source should not be retrieved";
    return gr::block_sptr();
}


gr::basic_block_sptr Gn3sSignalSource::get_right_block()
{
    return gn3s_source_;
}
