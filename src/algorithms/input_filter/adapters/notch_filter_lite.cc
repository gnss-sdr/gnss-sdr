/*!
 * \file notch_filter_lite.cc
 * \brief Adapts a gnuradio gr_notch_filter_lite
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

#include "notch_filter_lite.h"
#include "configuration_interface.h"
#include "notch_lite_cc.h"
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <cmath>

using google::LogMessage;

NotchFilterLite::NotchFilterLite(ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams) :
                role_(role), in_streams_(in_streams),
                out_streams_(out_streams)
{
    size_t item_size_;
    float p_c_factor;
    float default_p_c_factor = 0.9;
    float pfa;
    float default_pfa = 0.001;
    int length_;
    int default_length_ = 32;
    int n_segments_est;
    int default_n_segments_est = 12500;
    int n_segments_reset;
    int default_n_segments_reset = 5000000;
    float default_samp_freq = 4000000;
    float samp_freq = configuration->property("SignalSource.sampling_frequency", default_samp_freq);
    float default_coeff_rate = samp_freq * 0.1; 
    float coeff_rate;
    std::string default_item_type = "gr_complex";
    std::string default_dump_file = "./data/input_filter.dat";
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    dump_ = configuration->property(role + ".dump", false);
    DLOG(INFO) << "dump_ is " << dump_;
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);
    p_c_factor = configuration->property(role + ".p_c_factor", default_p_c_factor);
    pfa = configuration->property(role + ".pfa", default_pfa);
    coeff_rate = configuration->property(role + ".coeff_rate", default_coeff_rate);
    length_ = configuration->property(role + ".length", default_length_);
    n_segments_est = configuration->property(role + ".segments_est", default_n_segments_est);
    n_segments_reset = configuration->property(role + ".segments_reset", default_n_segments_reset);
    int n_segments_coeff = static_cast<int>((samp_freq / coeff_rate) / static_cast<float>(length_));
    n_segments_coeff = std::max(1, n_segments_coeff);
    if (item_type_.compare("gr_complex") == 0)
        {
            item_size_ = sizeof(gr_complex);
            notch_filter_lite_ = make_notch_filter_lite(p_c_factor, pfa, length_, n_segments_est, n_segments_reset, n_segments_coeff);
            DLOG(INFO) << "Item size " << item_size_;
            DLOG(INFO) << "input filter(" << notch_filter_lite_->unique_id() << ")";
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type for notch filter";
            item_size_ = sizeof(gr_complex);
        }
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }
}


NotchFilterLite::~NotchFilterLite()
{}


void NotchFilterLite::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->connect(notch_filter_lite_, 0, file_sink_, 0);
            DLOG(INFO) << "connected notch filter output to file sink";
        }
    else
        {
            DLOG(INFO) << "nothing to connect internally";
        }   
}


void NotchFilterLite::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->disconnect(notch_filter_lite_, 0, file_sink_, 0);
        }
}


gr::basic_block_sptr NotchFilterLite::get_left_block() 
{
    return notch_filter_lite_;
}


gr::basic_block_sptr NotchFilterLite::get_right_block()
{
    return notch_filter_lite_;
}
