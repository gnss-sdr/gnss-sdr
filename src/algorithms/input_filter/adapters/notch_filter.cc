/*!
 * \file notch_filter.cc
 * \brief Adapts a gnuradio gr_notch_filter
 * \author Antonio Ramos, 2017. antonio.ramosdet(at)gmail.com
 *
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

#include "notch_filter.h"
#include "configuration_interface.h"
#include "notch_cc.h"
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>


NotchFilter::NotchFilter(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(role),
      in_streams_(in_streams),
      out_streams_(out_streams),
      dump_(configuration->property(role + ".dump", false))
{
    const std::string default_item_type("gr_complex");
    const std::string default_dump_file("./data/input_filter.dat");
    const float default_pfa = 0.001;
    const float default_p_c_factor = 0.9;
    const int default_length_ = 32;
    const int default_n_segments_est = 12500;
    const int default_n_segments_reset = 5000000;

    const float pfa = configuration->property(role + ".pfa", default_pfa);
    const float p_c_factor = configuration->property(role + ".p_c_factor", default_p_c_factor);
    const int length_ = configuration->property(role + ".length", default_length_);
    const int n_segments_est = configuration->property(role + ".segments_est", default_n_segments_est);
    const int n_segments_reset = configuration->property(role + ".segments_reset", default_n_segments_reset);

    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);
    item_type_ = configuration->property(role + ".item_type", default_item_type);

    DLOG(INFO) << "role " << role_;
    if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            notch_filter_ = make_notch_filter(pfa, p_c_factor, length_, n_segments_est, n_segments_reset);
            DLOG(INFO) << "Item size " << item_size_;
            DLOG(INFO) << "input filter(" << notch_filter_->unique_id() << ")";
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type for notch filter";
            item_size_ = 0;  // notify wrong configuration
        }
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void NotchFilter::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->connect(notch_filter_, 0, file_sink_, 0);
            DLOG(INFO) << "connected notch filter output to file sink";
        }
    else
        {
            DLOG(INFO) << "nothing to connect internally";
        }
}


void NotchFilter::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->disconnect(notch_filter_, 0, file_sink_, 0);
        }
}


gr::basic_block_sptr NotchFilter::get_left_block()
{
    return notch_filter_;
}


gr::basic_block_sptr NotchFilter::get_right_block()
{
    return notch_filter_;
}
