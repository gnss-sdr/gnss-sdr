/*!
 * \file freq_xlating_fir_filter.cc
 * \brief Adapts a gnuradio gr_freq_xlating_fir_filter designed with gr_remez
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2014  (see AUTHORS file for a list of contributors)
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

#include "freq_xlating_fir_filter.h"
#include <boost/lexical_cast.hpp>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/filter/pm_remez.h>
#include <glog/logging.h>
#include "configuration_interface.h"

using google::LogMessage;

FreqXlatingFirFilter::FreqXlatingFirFilter(ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
                        config_(configuration), role_(role), in_streams_(in_streams),
                        out_streams_(out_streams), queue_(queue)
{
    size_t item_size;
    (*this).init();
    int decimation_factor;
    int default_decimation_factor = 1;
    decimation_factor = config_->property(role_ + ".decimation_factor", default_decimation_factor);

    if ((taps_item_type_.compare("float") == 0) && (input_item_type_.compare("gr_complex") == 0)
            && (output_item_type_.compare("gr_complex") == 0))
        {
            item_size = sizeof(gr_complex); //output
            input_size_ = sizeof(gr_complex); //input
            freq_xlating_fir_filter_ccf_ = gr::filter::freq_xlating_fir_filter_ccf::make(decimation_factor, taps_, intermediate_freq_, sampling_freq_);
            DLOG(INFO) << "input_filter(" << freq_xlating_fir_filter_ccf_->unique_id() << ")";
        }
    else if((taps_item_type_.compare("float") == 0) && (input_item_type_.compare("float") == 0)
            && (output_item_type_.compare("gr_complex") == 0))
        {
            item_size = sizeof(gr_complex);
            input_size_ = sizeof(float); //input
            freq_xlating_fir_filter_fcf_ = gr::filter::freq_xlating_fir_filter_fcf::make(decimation_factor, taps_, intermediate_freq_, sampling_freq_);
            DLOG(INFO) << "input_filter(" << freq_xlating_fir_filter_fcf_->unique_id() << ")";
        }
    else if((taps_item_type_.compare("float") == 0) && (input_item_type_.compare("short") == 0)
            && (output_item_type_.compare("gr_complex") == 0))
        {
            item_size = sizeof(gr_complex);
            input_size_ = sizeof(short); //input
            freq_xlating_fir_filter_scf_ = gr::filter::freq_xlating_fir_filter_scf::make(decimation_factor, taps_, intermediate_freq_, sampling_freq_);
            DLOG(INFO) << "input_filter(" << freq_xlating_fir_filter_scf_->unique_id() << ")";
        }
    else
        {
            LOG(ERROR) << taps_item_type_ << " unknown input filter item type";
            item_size = sizeof(gr_complex); //avoids unitialization
        }

    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            std::cout << "Dumping output into file " << dump_filename_ << std::endl;
            file_sink_ = gr::blocks::file_sink::make(item_size, dump_filename_.c_str());
        }
}



FreqXlatingFirFilter::~FreqXlatingFirFilter()
{}



void FreqXlatingFirFilter::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            if (input_size_ == sizeof(float))
                {
                    top_block->connect(freq_xlating_fir_filter_fcf_, 0, file_sink_, 0);
                }
            else if (input_size_ == sizeof(short))
                {
                    top_block->connect(freq_xlating_fir_filter_scf_, 0, file_sink_, 0);
                }
            else
                {
                    top_block->connect(freq_xlating_fir_filter_ccf_, 0, file_sink_, 0);
                }
        }
    else
        {
            DLOG(INFO) << "Nothing to connect internally";
        }
}



void FreqXlatingFirFilter::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            if (input_size_ == sizeof(float))
                {
                    top_block->disconnect(freq_xlating_fir_filter_fcf_, 0, file_sink_, 0);
                }
            else if (input_size_ == sizeof(short))
                {
                    top_block->disconnect(freq_xlating_fir_filter_scf_, 0, file_sink_, 0);
                }
            else
                {
                    top_block->disconnect(freq_xlating_fir_filter_ccf_, 0, file_sink_, 0);
                }
        }

}


gr::basic_block_sptr FreqXlatingFirFilter::get_left_block()
{
    if (input_size_ == sizeof(float))
        {
            return freq_xlating_fir_filter_fcf_;
        }
    else if (input_size_ == sizeof(short))
        {
            return freq_xlating_fir_filter_scf_;
        }
    else
        {
            return freq_xlating_fir_filter_ccf_;
        }

}


gr::basic_block_sptr FreqXlatingFirFilter::get_right_block()
{
    if (input_size_ == sizeof(float))
        {
            return freq_xlating_fir_filter_fcf_;
        }
    else if (input_size_ == sizeof(short))
        {
            return freq_xlating_fir_filter_scf_;
        }
    else
        {
            return freq_xlating_fir_filter_ccf_;
        }
}


void FreqXlatingFirFilter::init()
{
    std::string default_input_item_type = "gr_complex";
    std::string default_output_item_type = "gr_complex";
    std::string default_taps_item_type = "float";
    std::string default_dump_filename = "../data/input_filter.dat";
    double default_intermediate_freq = 0;
    double default_sampling_freq = 4000000;
    int default_number_of_taps = 6;
    unsigned int default_number_of_bands = 2;
    std::vector<double> default_bands = { 0.0, 0.4, 0.6, 1.0 };
    std::vector<double> default_ampl = { 1.0, 1.0, 0.0, 0.0 };
    std::vector<double> default_error_w = { 1.0, 1.0 };
    std::string default_filter_type = "bandpass";
    int default_grid_density = 16;

    DLOG(INFO) << "role " << role_;

    input_item_type_ = config_->property(role_ + ".input_item_type", default_input_item_type);
    output_item_type_ = config_->property(role_ + ".output_item_type", default_output_item_type);
    taps_item_type_ = config_->property(role_ + ".taps_item_type", default_taps_item_type);
    dump_ = config_->property(role_ + ".dump", false);
    dump_filename_ = config_->property(role_ + ".dump_filename", default_dump_filename);
    intermediate_freq_ = config_->property(role_ + ".IF", default_intermediate_freq);
    sampling_freq_ = config_->property(role_ + ".sampling_frequency", default_sampling_freq);
    int number_of_taps = config_->property(role_ + ".number_of_taps", default_number_of_taps);
    unsigned int number_of_bands = config_->property(role_ + ".number_of_bands", default_number_of_bands);

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

    std::vector<double> taps_d = gr::filter::pm_remez(number_of_taps - 1, bands, ampl,
              error_w, filter_type, grid_density);

    taps_.reserve(taps_d.size());
    for (std::vector<double>::iterator it = taps_d.begin(); it != taps_d.end(); it++)
        {
            taps_.push_back(float(*it));
            //std::cout<<"TAP="<<float(*it)<<std::endl;
        }
}
