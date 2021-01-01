/*!
 * \file fir_filter.cc
 * \brief Adapts a gnuradio gr_fir_filter designed with gr_remez
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *         Carles Fernandez-Prades, 2015  cfernandez(at)cttc.es
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

#include "fir_filter.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <gnuradio/filter/pm_remez.h>
#include <volk/volk.h>
#include <utility>


FirFilter::FirFilter(const ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(std::move(role)), in_streams_(in_streams), out_streams_(out_streams)
{
    config_ = configuration;
    (*this).init();
    item_size_ = 0;
    if ((taps_item_type_ == "float") && (input_item_type_ == "gr_complex") && (output_item_type_ == "gr_complex"))
        {
            item_size_ = sizeof(gr_complex);
            fir_filter_ccf_ = gr::filter::fir_filter_ccf::make(1, taps_);
            DLOG(INFO) << "input_filter(" << fir_filter_ccf_->unique_id() << ")";
            if (dump_)
                {
                    DLOG(INFO) << "Dumping output into file " << dump_filename_;
                    file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "cshort") && (output_item_type_ == "cshort"))
        {
            item_size_ = sizeof(lv_16sc_t);
            cshort_to_float_x2_ = make_cshort_to_float_x2();
            fir_filter_fff_1_ = gr::filter::fir_filter_fff::make(1, taps_);
            fir_filter_fff_2_ = gr::filter::fir_filter_fff::make(1, taps_);
            DLOG(INFO) << "I input_filter(" << fir_filter_fff_1_->unique_id() << ")";
            DLOG(INFO) << "Q input_filter(" << fir_filter_fff_2_->unique_id() << ")";
            float_to_short_1_ = gr::blocks::float_to_short::make();
            float_to_short_2_ = gr::blocks::float_to_short::make();
            short_x2_to_cshort_ = make_short_x2_to_cshort();
            if (dump_)
                {
                    DLOG(INFO) << "Dumping output into file " << dump_filename_;
                    file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "cshort") && (output_item_type_ == "gr_complex"))
        {
            item_size_ = sizeof(gr_complex);
            cshort_to_float_x2_ = make_cshort_to_float_x2();
            fir_filter_fff_1_ = gr::filter::fir_filter_fff::make(1, taps_);
            fir_filter_fff_2_ = gr::filter::fir_filter_fff::make(1, taps_);
            DLOG(INFO) << "I input_filter(" << fir_filter_fff_1_->unique_id() << ")";
            DLOG(INFO) << "Q input_filter(" << fir_filter_fff_2_->unique_id() << ")";
            float_to_complex_ = gr::blocks::float_to_complex::make();
            if (dump_)
                {
                    DLOG(INFO) << "Dumping output into file " << dump_filename_;
                    file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
                }
        }

    else if ((taps_item_type_ == "float") && (input_item_type_ == "cbyte") && (output_item_type_ == "gr_complex"))
        {
            item_size_ = sizeof(gr_complex);
            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();

            fir_filter_fff_1_ = gr::filter::fir_filter_fff::make(1, taps_);
            fir_filter_fff_2_ = gr::filter::fir_filter_fff::make(1, taps_);
            DLOG(INFO) << "I input_filter(" << fir_filter_fff_1_->unique_id() << ")";
            DLOG(INFO) << "Q input_filter(" << fir_filter_fff_2_->unique_id() << ")";

            float_to_complex_ = gr::blocks::float_to_complex::make();

            if (dump_)
                {
                    DLOG(INFO) << "Dumping output into file " << dump_filename_;
                    file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "cbyte") && (output_item_type_ == "cbyte"))
        {
            item_size_ = sizeof(lv_8sc_t);
            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();

            fir_filter_fff_1_ = gr::filter::fir_filter_fff::make(1, taps_);
            fir_filter_fff_2_ = gr::filter::fir_filter_fff::make(1, taps_);
            DLOG(INFO) << "I input_filter(" << fir_filter_fff_1_->unique_id() << ")";
            DLOG(INFO) << "Q input_filter(" << fir_filter_fff_2_->unique_id() << ")";

            float_to_char_1_ = gr::blocks::float_to_char::make();
            float_to_char_2_ = gr::blocks::float_to_char::make();

            char_x2_cbyte_ = make_byte_x2_to_complex_byte();

            if (dump_)
                {
                    DLOG(INFO) << "Dumping output into file " << dump_filename_;
                    file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
                }
        }
    else
        {
            LOG(ERROR) << " Unknown item type conversion";
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


void FirFilter::init()
{
    const std::string default_input_item_type("gr_complex");
    const std::string default_output_item_type("gr_complex");
    const std::string default_taps_item_type("float");
    const std::string default_dump_filename("../data/input_filter.dat");
    const int default_number_of_taps = 6;
    const unsigned int default_number_of_bands = 2;
    const std::vector<double> default_bands = {0.0, 0.4, 0.6, 1.0};
    const std::vector<double> default_ampl = {1.0, 1.0, 0.0, 0.0};
    const std::vector<double> default_error_w = {1.0, 1.0};
    const std::string default_filter_type("bandpass");
    const int default_grid_density = 16;

    DLOG(INFO) << "role " << role_;

    input_item_type_ = config_->property(role_ + ".input_item_type", default_input_item_type);
    output_item_type_ = config_->property(role_ + ".output_item_type", default_output_item_type);
    taps_item_type_ = config_->property(role_ + ".taps_item_type", default_taps_item_type);
    dump_ = config_->property(role_ + ".dump", false);
    dump_filename_ = config_->property(role_ + ".dump_filename", default_dump_filename);
    const int number_of_taps = config_->property(role_ + ".number_of_taps", default_number_of_taps);
    const unsigned int number_of_bands = config_->property(role_ + ".number_of_bands", default_number_of_bands);

    std::vector<double> bands;
    std::vector<double> ampl;
    std::vector<double> error_w;
    std::string option;
    double option_value;
    for (unsigned int i = 0; i < number_of_bands; i++)
        {
            option = ".band" + std::to_string(i + 1) + "_begin";
            option_value = config_->property(role_ + option, default_bands[i]);
            bands.push_back(option_value);

            option = ".band" + std::to_string(i + 1) + "_end";
            option_value = config_->property(role_ + option, default_bands[i]);
            bands.push_back(option_value);

            option = ".ampl" + std::to_string(i + 1) + "_begin";
            option_value = config_->property(role_ + option, default_bands[i]);
            ampl.push_back(option_value);

            option = ".ampl" + std::to_string(i + 1) + "_end";
            option_value = config_->property(role_ + option, default_bands[i]);
            ampl.push_back(option_value);

            option = ".band" + std::to_string(i + 1) + "_error";
            option_value = config_->property(role_ + option, default_bands[i]);
            error_w.push_back(option_value);
        }

    const std::string filter_type = config_->property(role_ + ".filter_type", default_filter_type);
    const int grid_density = config_->property(role_ + ".grid_density", default_grid_density);

    // pm_remez implements the Parks-McClellan FIR filter design.
    // It calculates the optimal (in the Chebyshev/minimax sense) FIR filter
    // impulse response given a set of band edges, the desired response on
    // those bands, and the weight given to the error in those bands.
    const std::vector<double> taps_d = gr::filter::pm_remez(number_of_taps - 1, bands, ampl, error_w, filter_type, grid_density);
    taps_.reserve(taps_d.size());
    for (const auto& it : taps_d)
        {
            taps_.push_back(static_cast<float>(it));
        }
}


void FirFilter::connect(gr::top_block_sptr top_block)
{
    if ((taps_item_type_ == "float") && (input_item_type_ == "gr_complex") && (output_item_type_ == "gr_complex"))
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
    else if ((taps_item_type_ == "float") && (input_item_type_ == "cshort") && (output_item_type_ == "cshort"))
        {
            top_block->connect(cshort_to_float_x2_, 0, fir_filter_fff_1_, 0);
            top_block->connect(cshort_to_float_x2_, 1, fir_filter_fff_2_, 0);
            top_block->connect(fir_filter_fff_1_, 0, float_to_short_1_, 0);
            top_block->connect(fir_filter_fff_2_, 0, float_to_short_2_, 0);
            top_block->connect(float_to_short_1_, 0, short_x2_to_cshort_, 0);
            top_block->connect(float_to_short_2_, 0, short_x2_to_cshort_, 1);
            if (dump_)
                {
                    top_block->connect(short_x2_to_cshort_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "cbyte") && (output_item_type_ == "gr_complex"))
        {
            top_block->connect(cbyte_to_float_x2_, 0, fir_filter_fff_1_, 0);
            top_block->connect(cbyte_to_float_x2_, 1, fir_filter_fff_2_, 0);
            top_block->connect(fir_filter_fff_1_, 0, float_to_complex_, 0);
            top_block->connect(fir_filter_fff_2_, 0, float_to_complex_, 1);
            if (dump_)
                {
                    top_block->connect(float_to_complex_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "cbyte") && (output_item_type_ == "cbyte"))
        {
            top_block->connect(cbyte_to_float_x2_, 0, fir_filter_fff_1_, 0);
            top_block->connect(cbyte_to_float_x2_, 1, fir_filter_fff_2_, 0);
            top_block->connect(fir_filter_fff_1_, 0, float_to_char_1_, 0);
            top_block->connect(fir_filter_fff_2_, 0, float_to_char_2_, 0);
            top_block->connect(float_to_char_1_, 0, char_x2_cbyte_, 0);
            top_block->connect(float_to_char_2_, 0, char_x2_cbyte_, 1);
            if (dump_)
                {
                    top_block->connect(char_x2_cbyte_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "cshort") && (output_item_type_ == "gr_complex"))
        {
            top_block->connect(cshort_to_float_x2_, 0, fir_filter_fff_1_, 0);
            top_block->connect(cshort_to_float_x2_, 1, fir_filter_fff_2_, 0);
            top_block->connect(fir_filter_fff_1_, 0, float_to_complex_, 0);
            top_block->connect(fir_filter_fff_2_, 0, float_to_complex_, 1);
            if (dump_)
                {
                    top_block->connect(float_to_complex_, 0, file_sink_, 0);
                }
        }
    else
        {
            LOG(ERROR) << " Unknown item type conversion";
        }
}


void FirFilter::disconnect(gr::top_block_sptr top_block)
{
    if ((taps_item_type_ == "float") && (input_item_type_ == "gr_complex") && (output_item_type_ == "gr_complex"))
        {
            if (dump_)
                {
                    top_block->disconnect(fir_filter_ccf_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "cbyte") && (output_item_type_ == "gr_complex"))
        {
            top_block->disconnect(fir_filter_fff_2_, 0, float_to_complex_, 1);
            top_block->disconnect(fir_filter_fff_1_, 0, float_to_complex_, 0);
            top_block->disconnect(cbyte_to_float_x2_, 1, fir_filter_fff_2_, 0);
            top_block->disconnect(cbyte_to_float_x2_, 0, fir_filter_fff_1_, 0);
            if (dump_)
                {
                    top_block->disconnect(float_to_complex_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "cshort") && (output_item_type_ == "cshort"))
        {
            top_block->disconnect(cshort_to_float_x2_, 0, fir_filter_fff_1_, 0);
            top_block->disconnect(cshort_to_float_x2_, 1, fir_filter_fff_2_, 0);
            top_block->disconnect(fir_filter_fff_1_, 0, float_to_short_1_, 0);
            top_block->disconnect(fir_filter_fff_2_, 0, float_to_short_2_, 0);
            top_block->disconnect(float_to_short_1_, 0, short_x2_to_cshort_, 0);
            top_block->disconnect(float_to_short_2_, 0, short_x2_to_cshort_, 1);
            if (dump_)
                {
                    top_block->disconnect(short_x2_to_cshort_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "cbyte") && (output_item_type_ == "cbyte"))
        {
            top_block->disconnect(float_to_char_2_, 0, char_x2_cbyte_, 1);
            top_block->disconnect(float_to_char_1_, 0, char_x2_cbyte_, 0);
            top_block->disconnect(fir_filter_fff_2_, 0, float_to_char_2_, 0);
            top_block->disconnect(fir_filter_fff_1_, 0, float_to_char_1_, 0);
            top_block->disconnect(cbyte_to_float_x2_, 0, fir_filter_fff_1_, 0);
            top_block->disconnect(cbyte_to_float_x2_, 1, fir_filter_fff_2_, 0);
            if (dump_)
                {
                    top_block->disconnect(char_x2_cbyte_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "cshort") && (output_item_type_ == "gr_complex"))
        {
            top_block->disconnect(cshort_to_float_x2_, 0, fir_filter_fff_1_, 0);
            top_block->disconnect(cshort_to_float_x2_, 1, fir_filter_fff_2_, 0);
            top_block->disconnect(fir_filter_fff_1_, 0, float_to_complex_, 0);
            top_block->disconnect(fir_filter_fff_2_, 0, float_to_complex_, 1);
            if (dump_)
                {
                    top_block->disconnect(float_to_complex_, 0, file_sink_, 0);
                }
        }
    else
        {
            LOG(ERROR) << " Unknown item type conversion";
        }
}


gr::basic_block_sptr FirFilter::get_left_block()
{
    if ((taps_item_type_ == "float") && (input_item_type_ == "gr_complex") && (output_item_type_ == "gr_complex"))
        {
            return fir_filter_ccf_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "cshort") && (output_item_type_ == "cshort"))
        {
            return cshort_to_float_x2_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "cbyte") && (output_item_type_ == "gr_complex"))
        {
            return cbyte_to_float_x2_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "cbyte") && (output_item_type_ == "cbyte"))
        {
            return cbyte_to_float_x2_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "cshort") && (output_item_type_ == "gr_complex"))
        {
            return cshort_to_float_x2_;
        }
    LOG(WARNING) << "Unknown item type conversion";
    return nullptr;
}


gr::basic_block_sptr FirFilter::get_right_block()
{
    if ((taps_item_type_ == "float") && (input_item_type_ == "gr_complex") && (output_item_type_ == "gr_complex"))
        {
            return fir_filter_ccf_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "cshort") && (output_item_type_ == "cshort"))
        {
            return short_x2_to_cshort_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "cbyte") && (output_item_type_ == "gr_complex"))
        {
            return float_to_complex_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "cbyte") && (output_item_type_ == "cbyte"))
        {
            return char_x2_cbyte_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "cshort") && (output_item_type_ == "gr_complex"))
        {
            return float_to_complex_;
        }
    LOG(WARNING) << "Unknown input filter taps item type";
    return nullptr;
}
