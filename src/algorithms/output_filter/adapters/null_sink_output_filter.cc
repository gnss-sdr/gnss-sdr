/*!
 * \file null_sink_output_filter.cc
 * \brief  Implementation of an adapter of a null sink output filter block to an
 * OutputFilterInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
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

#include "null_sink_output_filter.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include "configuration_interface.h"

using google::LogMessage;

NullSinkOutputFilter::NullSinkOutputFilter(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams) :
                role_(role),
                in_streams_(in_streams),
                out_streams_(out_streams)
{
    std::string default_item_type = "short";
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    if(item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
        }
    else if(item_type_.compare("short") == 0)
        {
            item_size_ = sizeof(short);
        }
    else if(item_type_.compare("float") == 0)
        {
            item_size_ = sizeof(float);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type. Using float";
            item_size_ = sizeof(float);
        }
    sink_ = gr::blocks::null_sink::make(item_size_);
    DLOG(INFO) << "null_sink(" << sink_->unique_id() << ")";
}



NullSinkOutputFilter::~NullSinkOutputFilter()
{}



void NullSinkOutputFilter::connect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	DLOG(INFO) << "nothing to connect internally";
}



void NullSinkOutputFilter::disconnect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	// Nothing to connect
}



gr::basic_block_sptr NullSinkOutputFilter::get_left_block()
{
    return sink_;
}



gr::basic_block_sptr NullSinkOutputFilter::get_right_block()
{
    LOG(WARNING) << "Right block of a signal sink should not be retrieved";
    //return gr::blocks::sptr::make();
    return sink_;
}
