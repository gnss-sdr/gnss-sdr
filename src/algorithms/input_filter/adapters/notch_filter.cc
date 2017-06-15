/*!
 * \file notch_filter.cc
 * \brief Adapts a gnuradio gr_notch_filter
 * \author Antonio Ramos, 2017. antonio.ramosdet(at)gmail.com
 *         
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#include "notch_filter.h"
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <volk/volk.h>
#include "configuration_interface.h"

using google::LogMessage;

NotchFilter::NotchFilter(ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams) :
                config_(configuration), role_(role), in_streams_(in_streams),
                out_streams_(out_streams)
{
    (*this).init();
}

NotchFilter::~NotchFilter()
{}

void NotchFilter::connect(gr::top_block_sptr top_block)
{
    
}

void NotchFilter::disconnect(gr::top_block_sptr top_block)
{
    
}


gr::basic_block_sptr NotchFilter::get_left_block() 
{

}

gr::basic_block_sptr NotchFilter::get_right_block()
{
    
}

void NotchFilter::init()
{
    
}