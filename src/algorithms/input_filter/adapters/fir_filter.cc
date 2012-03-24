/*!
 * \file fir_filter.cc
 * \brief Adapts a gnuradio gr_fir_filter designed with gr_remez
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2012  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#include "fir_filter.h"
#include "configuration_interface.h"
#include <string>
#include <boost/lexical_cast.hpp>
#include <gnuradio/gr_io_signature.h>
#include <gnuradio/gr_file_sink.h>
#include <gnuradio/gr_remez.h>
#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

FirFilter::FirFilter(ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams,
        gr_msg_queue_sptr queue) :
        config_(configuration), role_(role), in_streams_(in_streams),
        out_streams_(out_streams), queue_(queue)
{

    size_t item_size;
    (*this).init();

    if ((taps_item_type_.compare("float") == 0) && (input_item_type_.compare(
            "gr_complex") == 0) && (output_item_type_.compare("gr_complex")
                    == 0))
        {
            item_size = sizeof(gr_complex);
            fir_filter_ccf_ = gr_make_fir_filter_ccf(1, taps_);
            DLOG(INFO) << "input_filter(" << fir_filter_ccf_->unique_id() << ")";

        }
    else
        {
            LOG_AT_LEVEL(ERROR) << taps_item_type_
                    << " unknown input filter item type";
        }
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr_make_file_sink(item_size, dump_filename_.c_str());
        }

}

FirFilter::~FirFilter()
{}

void FirFilter::connect(gr_top_block_sptr top_block)
{

    if (dump_)
        {
            top_block->connect(fir_filter_ccf_, 0, file_sink_, 0);
        }
    else
        {
            DLOG(INFO) << "Nothing to connect internally";
        }
}

void FirFilter::disconnect(gr_top_block_sptr top_block)
{

    if (dump_)
        {
            top_block->connect(fir_filter_ccf_, 0, file_sink_, 0);
        }

}

gr_basic_block_sptr FirFilter::get_left_block()
{
    return fir_filter_ccf_;
}

gr_basic_block_sptr FirFilter::get_right_block()
{
    return fir_filter_ccf_;
}

void FirFilter::init()
{
    std::string default_input_item_type = "gr_complex";
    std::string default_output_item_type = "gr_complex";
    std::string default_taps_item_type = "float";
    std::string default_dump_filename = "../data/input_filter.dat";
    int default_number_of_taps = 6;
    unsigned int default_number_of_bands = 2;
    std::vector<double> default_bands = { 0.0, 0.4, 0.6, 1.0 };
    std::vector<double> default_ampl = { 1.0, 1.0, 0.0, 0.0 };
    std::vector<double> default_error_w = { 1.0, 1.0 };
    std::string default_filter_type = "bandpass";
    int default_grid_density = 16;

    DLOG(INFO) << "role " << role_;

    input_item_type_ = config_->property(role_ + ".input_item_type",
            default_input_item_type);
    output_item_type_ = config_->property(role_ + ".output_item_type",
            default_output_item_type);
    taps_item_type_ = config_->property(role_ + ".taps_item_type",
            default_taps_item_type);

    dump_ = config_->property(role_ + ".dump", false);
    dump_filename_ = config_->property(role_ + ".dump_filename",
            default_dump_filename);
    int number_of_taps = config_->property(role_ + ".number_of_taps",
            default_number_of_taps);
    unsigned int number_of_bands = config_->property(role_ + ".number_of_bands",
            default_number_of_bands);

    std::vector<double> bands;
    std::vector<double> ampl;
    std::vector<double> error_w;
    std::string option;
    double option_value;

    for (unsigned int i = 0; i < number_of_bands; i++)
        {

            option = ".band" + boost::lexical_cast<std::string>(i + 1) + "_begin";
            option_value = config_->property(role_ + option, default_bands[i]);
            bands.push_back(option_value);

            option = ".band" + boost::lexical_cast<std::string>(i + 1) + "_end";
            option_value = config_->property(role_ + option, default_bands[i]);
            bands.push_back(option_value);

            option = ".ampl" + boost::lexical_cast<std::string>(i + 1) + "_begin";
            option_value = config_->property(role_ + option, default_bands[i]);
            ampl.push_back(option_value);

            option = ".ampl" + boost::lexical_cast<std::string>(i + 1) + "_end";
            option_value = config_->property(role_ + option, default_bands[i]);
            ampl.push_back(option_value);

            option = ".band" + boost::lexical_cast<std::string>(i + 1) + "_error";
            option_value = config_->property(role_ + option, default_bands[i]);
            error_w.push_back(option_value);
        }

    std::string filter_type = config_->property(role_ + ".filter_type", default_filter_type);
    int grid_density = config_->property(role_ + ".grid_density", default_grid_density);

    std::vector<double> taps_d = gr_remez(number_of_taps - 1, bands, ampl,
            error_w, filter_type, grid_density);
    taps_.reserve(taps_d.size());
    for (std::vector<double>::iterator it = taps_d.begin(); it != taps_d.end(); it++)
        {
            taps_.push_back(float(*it));
        }
}
