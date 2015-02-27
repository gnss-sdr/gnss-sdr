/*!
 * \file file_output_filter.cc
 * \brief Implementation of an adapter of a file output filter block
 * to an OutputFilterInterface
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

#include "file_output_filter.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include "configuration_interface.h"

using google::LogMessage;

FileOutputFilter::FileOutputFilter(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams) :
        role_(role),
        in_streams_(in_streams),
        out_streams_(out_streams)
{
    std::string default_filename = "./output.dat";
    std::string default_item_type = "short";
    filename_ = configuration->property(role + ".filename", default_filename);
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    if(item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
        }
    else if(item_type_.compare("float") == 0)
        {
            item_size_ = sizeof(float);
        }
    else if(item_type_.compare("short") == 0)
        {
            item_size_ = sizeof(short);
        }
    else
        {
            LOG(WARNING) << item_type_ << " Unrecognized item type. Using short.";
            item_size_ = sizeof(short);
        }
    file_sink_ = gr::blocks::file_sink::make(item_size_, filename_.c_str());
    DLOG(INFO) << "file sink(" << file_sink_->unique_id() << ")";
}



FileOutputFilter::~FileOutputFilter()
{}



void FileOutputFilter::connect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	DLOG(INFO) << "nothing to connect internally";
}


void FileOutputFilter::disconnect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	// Nothing to disconnect internally
}



gr::basic_block_sptr FileOutputFilter::get_left_block()
{
    return file_sink_;
}



gr::basic_block_sptr FileOutputFilter::get_right_block()
{
    //return file_sink_;//gr_block_sptr();
    return gr::blocks::file_sink::sptr();
}
