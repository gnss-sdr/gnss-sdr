/*!
 * \file pulse_blanking_filter.cc
 * \brief Instantiates the GNSS-SDR pulse blanking filter
 * \author Javier Arribas 2017
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

#include "pulse_blanking_filter.h"
#include <boost/lexical_cast.hpp>
#include <gnuradio/blocks/file_sink.h>
#include <glog/logging.h>
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

    double Pfa = config_->property(role_ + ".Pfa", 0.001);

    if (input_item_type_.compare("gr_complex") == 0)
        {
            item_size = sizeof(gr_complex); //output
            input_size_ = sizeof(gr_complex); //input
            pulse_blanking_cc_ = make_pulse_blanking_cc(Pfa);
        }
    else
        {
            LOG(ERROR) << " Unknown input filter input/output item type conversion";
            item_size = sizeof(gr_complex); //avoids uninitialization
            input_size_ = sizeof(gr_complex); //avoids uninitialization
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
            return pulse_blanking_cc_;
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
