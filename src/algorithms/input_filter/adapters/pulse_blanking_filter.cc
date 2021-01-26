/*!
 * \file pulse_blanking_filter.cc
 * \brief Instantiates the GNSS-SDR pulse blanking filter
 * \author Javier Arribas 2017
 *         Antonio Ramos  2017
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

#include "pulse_blanking_filter.h"
#include "configuration_interface.h"
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <gnuradio/filter/firdes.h>
#include <cmath>
#include <utility>
#include <vector>


PulseBlankingFilter::PulseBlankingFilter(const ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(std::move(role)), in_streams_(in_streams), out_streams_(out_streams)
{
    size_t item_size;
    xlat_ = false;
    const std::string default_item_type("gr_complex");
    const std::string default_dump_filename("../data/input_filter.dat");

    DLOG(INFO) << "role " << role_;

    item_type_ = configuration->property(role_ + ".item_type", default_item_type);
    dump_ = configuration->property(role_ + ".dump", false);
    dump_filename_ = configuration->property(role_ + ".dump_filename", default_dump_filename);
    const float default_pfa_ = 0.04;
    const float pfa = configuration->property(role_ + ".pfa", default_pfa_);
    const int default_length_ = 32;
    const int length_ = configuration->property(role_ + ".length", default_length_);
    const int default_n_segments_est = 12500;
    const int n_segments_est = configuration->property(role_ + ".segments_est", default_n_segments_est);
    const int default_n_segments_reset = 5000000;
    const int n_segments_reset = configuration->property(role_ + ".segments_reset", default_n_segments_reset);
    if (item_type_ == "gr_complex")
        {
            item_size = sizeof(gr_complex);    // output
            input_size_ = sizeof(gr_complex);  // input
            pulse_blanking_cc_ = make_pulse_blanking_cc(pfa, length_, n_segments_est, n_segments_reset);
        }
    else
        {
            LOG(ERROR) << "Unknown input filter item_types conversion";
            item_size = sizeof(gr_complex);  // avoids uninitialization
            input_size_ = 0;                 // notify wrong configuration
        }
    const double default_if = 0.0;
    const double if_aux = configuration->property(role_ + ".if", default_if);
    const double if_ = configuration->property(role_ + ".IF", if_aux);
    if (std::abs(if_) > 1.0)
        {
            xlat_ = true;
            const double default_sampling_freq = 4000000.0;
            const double sampling_freq_ = configuration->property(role_ + ".sampling_frequency", default_sampling_freq);
            const double default_bw = 2000000.0;
            const double bw_ = configuration->property(role_ + ".bw", default_bw);
            const double default_tw = bw_ / 10.0;
            const double tw_ = configuration->property(role_ + ".tw", default_tw);
            const std::vector<float> taps = gr::filter::firdes::low_pass(1.0, sampling_freq_, bw_, tw_);
            freq_xlating_ = gr::filter::freq_xlating_fir_filter_ccf::make(1, taps, if_, sampling_freq_);
        }
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            std::cout << "Dumping output into file " << dump_filename_ << '\n';
            file_sink_ = gr::blocks::file_sink::make(item_size, dump_filename_.c_str());
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


void PulseBlankingFilter::connect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            if (dump_)
                {
                    top_block->connect(pulse_blanking_cc_, 0, file_sink_, 0);
                }
            if (xlat_)
                {
                    top_block->connect(freq_xlating_, 0, pulse_blanking_cc_, 0);
                }
        }

    else
        {
            LOG(ERROR) << " Unknown input filter input/output item type conversion";
        }
}


void PulseBlankingFilter::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_ == "gr_complex")
        {
            if (dump_)
                {
                    top_block->disconnect(pulse_blanking_cc_, 0, file_sink_, 0);
                }
            if (xlat_)
                {
                    top_block->disconnect(freq_xlating_, 0, pulse_blanking_cc_, 0);
                }
        }
    else
        {
            LOG(ERROR) << " Unknown input filter input/output item type conversion";
        }
}


gr::basic_block_sptr PulseBlankingFilter::get_left_block()
{
    if (item_type_ == "gr_complex")
        {
            if (xlat_)
                {
                    return freq_xlating_;
                }
            return pulse_blanking_cc_;
        }
    LOG(ERROR) << " Unknown input filter input/output item type conversion";
    return nullptr;
}


gr::basic_block_sptr PulseBlankingFilter::get_right_block()
{
    if (item_type_ == "gr_complex")
        {
            return pulse_blanking_cc_;
        }
    LOG(ERROR) << " Unknown input filter input/output item type conversion";
    return nullptr;
}
