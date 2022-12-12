/*!
 * \file notch_filter_lite.cc
 * \brief Adapts a gnuradio gr_notch_filter_lite
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

#include "notch_filter_lite.h"
#include "configuration_interface.h"
#include "notch_lite_cc.h"
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <algorithm>  // for max


NotchFilterLite::NotchFilterLite(const ConfigurationInterface* configuration,
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
    const float default_p_c_factor = 0.9;
    const float default_pfa = 0.001;
    const float default_samp_freq = 4000000;
    const int default_n_segments_reset = 5000000;
    const int default_length_ = 32;
    const int default_n_segments_est = 12500;
    const float samp_freq = configuration->property("SignalSource.sampling_frequency", default_samp_freq);
    const float default_coeff_rate = samp_freq * 0.1F;
    const float p_c_factor = configuration->property(role + ".p_c_factor", default_p_c_factor);
    const float pfa = configuration->property(role + ".pfa", default_pfa);
    const float coeff_rate = configuration->property(role + ".coeff_rate", default_coeff_rate);
    const int length_ = configuration->property(role + ".length", default_length_);
    const int n_segments_est = configuration->property(role + ".segments_est", default_n_segments_est);
    const int n_segments_reset = configuration->property(role + ".segments_reset", default_n_segments_reset);

    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);
    item_type_ = configuration->property(role + ".item_type", default_item_type);

    int n_segments_coeff = static_cast<int>((samp_freq / coeff_rate) / static_cast<float>(length_));
    n_segments_coeff = std::max(1, n_segments_coeff);
    DLOG(INFO) << "role " << role_;
    if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            notch_filter_lite_ = make_notch_filter_lite(p_c_factor, pfa, length_, n_segments_est, n_segments_reset, n_segments_coeff);
            DLOG(INFO) << "Item size " << item_size_;
            DLOG(INFO) << "input filter(" << notch_filter_lite_->unique_id() << ")";
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type for notch filter";
            item_size_ = 0;
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
