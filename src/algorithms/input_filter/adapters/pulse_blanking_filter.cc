/*!
 * \file pulse_blanking_filter.cc
 * \brief Instantiates the GNSS-SDR pulse blanking filter
 * \author Javier Arribas 2017
 *         Antonio Ramos  2017
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

#include "pulse_blanking_filter.h"
#include <boost/lexical_cast.hpp>
#include <vector>
#include <glog/logging.h>
#include <gnuradio/filter/firdes.h>
#include "configuration_interface.h"

using google::LogMessage;

PulseBlankingFilter::PulseBlankingFilter(ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams) :
                        config_(configuration), role_(role), in_streams_(in_streams),
                        out_streams_(out_streams)
{
    size_t item_size;
    std::string default_input_item_type = "gr_complex";
    std::string default_output_item_type = "gr_complex";
    std::string default_dump_filename = "../data/input_filter.dat";
    
    DLOG(INFO) << "role " << role_;

    input_item_type_ = config_->property(role_ + ".input_item_type", default_input_item_type);
    output_item_type_ = config_->property(role_ + ".output_item_type", default_output_item_type);
    dump_ = config_->property(role_ + ".dump", false);
    dump_filename_ = config_->property(role_ + ".dump_filename", default_dump_filename);
    float default_pfa_ = 0.04;
    float pfa = config_->property(role_ + ".pfa", default_pfa_);
    int default_length_ = 32;
    int length_ = config_->property(role_ + ".length", default_length_);
    int default_n_segments_est = 12500;
    int n_segments_est = config_->property(role_ + ".segments_est", default_n_segments_est);
    int default_n_segments_reset = 5000000;
    int n_segments_reset = config_->property(role_ + ".segments_reset", default_n_segments_reset);
    if (input_item_type_.compare("gr_complex") == 0)
        {
            item_size = sizeof(gr_complex); //output
            input_size_ = sizeof(gr_complex); //input
            pulse_blanking_cc_ = make_pulse_blanking_cc(pfa, length_, n_segments_est, n_segments_reset);
        }
    else
        {
            LOG(ERROR) << " Unknown input filter input/output item type conversion";
            item_size = sizeof(gr_complex); //avoids uninitialization
            input_size_ = sizeof(gr_complex); //avoids uninitialization
        }
    float default_if = 0.0;
    float if_ = config_->property(role_ + ".if", default_if);
    if (if_ > 0.0)
        {
            double default_bw = 2000000.0;
            double bw_ = config_->property(role_ + ".bw", default_bw);
            double default_tw = bw_ / 15.0;
            double tw_ = config_->property(role_ + ".tw", default_tw);
            const std::vector<float> taps = gr::filter::firdes::low_pass(1.0, config_->property("SignalSource.sampling_frequency", 2000000.0), bw_ / 2.0, tw_);
            freq_xlating_ = gr::filter::freq_xlating_fir_filter_ccf::make(1, taps, if_, config_->property("SignalSource.sampling_frequency", 2000000.0));
        }
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            std::cout << "Dumping output into file " << dump_filename_ << std::endl;
            file_sink_ = gr::blocks::file_sink::make(item_size, dump_filename_.c_str());
        }
}



PulseBlankingFilter::~PulseBlankingFilter()
{}



void PulseBlankingFilter::connect(gr::top_block_sptr top_block)
{
    if (input_item_type_.compare("gr_complex") == 0)
        {
            if (dump_)
                {
                    top_block->connect(pulse_blanking_cc_, 0, file_sink_, 0);
                }
            if (config_->property(role_ + ".if", 0.0) > 0.0)
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
    if (input_item_type_.compare("gr_complex") == 0)
        {
            if (dump_)
                {
                    top_block->disconnect(pulse_blanking_cc_, 0, file_sink_, 0);
                }
            if (config_->property(role_ + ".if", 0.0) > 0.0)
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
    if (input_item_type_.compare("gr_complex") == 0)
        {
            if (config_->property(role_ + ".if", 0.0) > 0.0)
                {
                    return freq_xlating_;
                }
            else
                {
                    return pulse_blanking_cc_;
                }
        }
    else
        {
            return nullptr;
            LOG(ERROR) << " Unknown input filter input/output item type conversion";
        }
}


gr::basic_block_sptr PulseBlankingFilter::get_right_block()
{
    if (input_item_type_.compare("gr_complex") == 0)
        {
            return pulse_blanking_cc_;
        }
    else
        {
            return nullptr;
            LOG(ERROR) << " Unknown input filter input/output item type conversion";
        }
}
