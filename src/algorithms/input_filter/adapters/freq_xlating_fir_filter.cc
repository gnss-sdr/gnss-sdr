/*!
 * \file freq_xlating_fir_filter.cc
 * \brief Adapts a gnuradio gr_freq_xlating_fir_filter designed with gr_remez or gr_firdes
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *         Antonio Ramos, 2017. antonio.ramos(at)cttc.es
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

#include "freq_xlating_fir_filter.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/filter/pm_remez.h>
#include <volk/volk.h>
#include <utility>


FreqXlatingFirFilter::FreqXlatingFirFilter(const ConfigurationInterface* configuration,
    std::string role,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(std::move(role)),
      in_streams_(in_streams),
      out_streams_(out_streams),
      dump_(configuration->property(role + ".dump", false))
{
    const std::string default_input_item_type("gr_complex");
    const std::string default_output_item_type("gr_complex");
    const std::string default_taps_item_type("float");
    const std::string default_dump_filename("../data/input_filter.dat");
    const double default_intermediate_freq = 0.0;
    const double default_sampling_freq = 4000000.0;
    const int default_number_of_taps = 6;
    const unsigned int default_number_of_bands = 2;
    const std::vector<double> default_bands = {0.0, 0.4, 0.6, 1.0};
    const std::vector<double> default_ampl = {1.0, 1.0, 0.0, 0.0};
    const std::vector<double> default_error_w = {1.0, 1.0};
    const std::string default_filter_type("bandpass");
    const int default_grid_density = 16;
    const int default_decimation_factor = 1;

    const int number_of_taps = configuration->property(role_ + ".number_of_taps", default_number_of_taps);
    const unsigned int number_of_bands = configuration->property(role_ + ".number_of_bands", default_number_of_bands);
    const std::string filter_type = configuration->property(role_ + ".filter_type", default_filter_type);

    dump_filename_ = configuration->property(role_ + ".dump_filename", default_dump_filename);
    input_item_type_ = configuration->property(role_ + ".input_item_type", default_input_item_type);
    output_item_type_ = configuration->property(role_ + ".output_item_type", default_output_item_type);
    taps_item_type_ = configuration->property(role_ + ".taps_item_type", default_taps_item_type);
    intermediate_freq_ = configuration->property(role_ + ".IF", default_intermediate_freq);
    sampling_freq_ = configuration->property(role_ + ".sampling_frequency", default_sampling_freq);
    decimation_factor_ = configuration->property(role_ + ".decimation_factor", default_decimation_factor);

    if (filter_type != "lowpass")
        {
            std::vector<double> bands;
            std::vector<double> ampl;
            std::vector<double> error_w;
            std::string option;
            double option_value;

            for (unsigned int i = 0; i < number_of_bands; i++)
                {
                    option = ".band" + std::to_string(i + 1) + "_begin";
                    option_value = configuration->property(role_ + option, default_bands[i]);
                    bands.push_back(option_value);

                    option = ".band" + std::to_string(i + 1) + "_end";
                    option_value = configuration->property(role_ + option, default_bands[i]);
                    bands.push_back(option_value);

                    option = ".ampl" + std::to_string(i + 1) + "_begin";
                    option_value = configuration->property(role_ + option, default_bands[i]);
                    ampl.push_back(option_value);

                    option = ".ampl" + std::to_string(i + 1) + "_end";
                    option_value = configuration->property(role_ + option, default_bands[i]);
                    ampl.push_back(option_value);

                    option = ".band" + std::to_string(i + 1) + "_error";
                    option_value = configuration->property(role_ + option, default_bands[i]);
                    error_w.push_back(option_value);
                }

            const int grid_density = configuration->property(role_ + ".grid_density", default_grid_density);
            const std::vector<double> taps_d = gr::filter::pm_remez(number_of_taps - 1, bands, ampl, error_w, filter_type, grid_density);
            taps_ = std::vector<float>(taps_d.begin(), taps_d.end());
        }
    else
        {
            const double default_bw = (sampling_freq_ / decimation_factor_) / 2;
            const double bw_ = configuration->property(role_ + ".bw", default_bw);
            const double default_tw = bw_ / 10.0;
            const double tw_ = configuration->property(role_ + ".tw", default_tw);
            taps_ = gr::filter::firdes::low_pass(1.0, sampling_freq_, bw_, tw_);
        }

    size_t item_size;
    DLOG(INFO) << "role " << role_;
    LOG(INFO) << "Created freq_xlating_fir_filter with " << taps_.size() << " taps";
    if ((taps_item_type_ == "float") && (input_item_type_ == "gr_complex") && (output_item_type_ == "gr_complex"))
        {
            item_size = sizeof(gr_complex);    // output
            input_size_ = sizeof(gr_complex);  // input
            freq_xlating_fir_filter_ccf_ = gr::filter::freq_xlating_fir_filter_ccf::make(decimation_factor_, taps_, intermediate_freq_, sampling_freq_);
            DLOG(INFO) << "input_filter(" << freq_xlating_fir_filter_ccf_->unique_id() << ")";
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "float") && (output_item_type_ == "gr_complex"))
        {
            item_size = sizeof(gr_complex);
            input_size_ = sizeof(float);  // input
            freq_xlating_fir_filter_fcf_ = gr::filter::freq_xlating_fir_filter_fcf::make(decimation_factor_, taps_, intermediate_freq_, sampling_freq_);
            DLOG(INFO) << "input_filter(" << freq_xlating_fir_filter_fcf_->unique_id() << ")";
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "short") && (output_item_type_ == "gr_complex"))
        {
            item_size = sizeof(gr_complex);
            input_size_ = sizeof(int16_t);  // input
            freq_xlating_fir_filter_scf_ = gr::filter::freq_xlating_fir_filter_scf::make(decimation_factor_, taps_, intermediate_freq_, sampling_freq_);
            DLOG(INFO) << "input_filter(" << freq_xlating_fir_filter_scf_->unique_id() << ")";
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "short") && (output_item_type_ == "cshort"))
        {
            item_size = sizeof(lv_16sc_t);
            input_size_ = sizeof(int16_t);  // input
            freq_xlating_fir_filter_scf_ = gr::filter::freq_xlating_fir_filter_scf::make(decimation_factor_, taps_, intermediate_freq_, sampling_freq_);
            DLOG(INFO) << "input_filter(" << freq_xlating_fir_filter_scf_->unique_id() << ")";
            complex_to_float_ = gr::blocks::complex_to_float::make();
            float_to_short_1_ = gr::blocks::float_to_short::make();
            float_to_short_2_ = gr::blocks::float_to_short::make();
            short_x2_to_cshort_ = make_short_x2_to_cshort();
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "byte") && (output_item_type_ == "gr_complex"))
        {
            item_size = sizeof(gr_complex);
            input_size_ = sizeof(int8_t);  // input
            gr_char_to_short_ = gr::blocks::char_to_short::make();
            freq_xlating_fir_filter_scf_ = gr::filter::freq_xlating_fir_filter_scf::make(decimation_factor_, taps_, intermediate_freq_, sampling_freq_);
            DLOG(INFO) << "input_filter(" << freq_xlating_fir_filter_scf_->unique_id() << ")";
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "byte") && (output_item_type_ == "cbyte"))
        {
            item_size = sizeof(lv_8sc_t);
            input_size_ = sizeof(int8_t);  // input
            gr_char_to_short_ = gr::blocks::char_to_short::make();
            freq_xlating_fir_filter_scf_ = gr::filter::freq_xlating_fir_filter_scf::make(decimation_factor_, taps_, intermediate_freq_, sampling_freq_);
            DLOG(INFO) << "input_filter(" << freq_xlating_fir_filter_scf_->unique_id() << ")";
            complex_to_complex_byte_ = make_complex_float_to_complex_byte();
        }
    else
        {
            LOG(ERROR) << " Unknown input filter input/output item type conversion";
            item_size = sizeof(gr_complex);  // avoids uninitialization
            input_size_ = 0;                 // notifies wrong configuration
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


void FreqXlatingFirFilter::connect(gr::top_block_sptr top_block)
{
    if ((taps_item_type_ == "float") && (input_item_type_ == "gr_complex") && (output_item_type_ == "gr_complex"))
        {
            if (dump_)
                {
                    top_block->connect(freq_xlating_fir_filter_ccf_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "float") && (output_item_type_ == "gr_complex"))
        {
            if (dump_)
                {
                    top_block->connect(freq_xlating_fir_filter_fcf_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "short") && (output_item_type_ == "gr_complex"))
        {
            if (dump_)
                {
                    top_block->connect(freq_xlating_fir_filter_scf_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "short") && (output_item_type_ == "cshort"))
        {
            top_block->connect(freq_xlating_fir_filter_scf_, 0, complex_to_float_, 0);
            top_block->connect(complex_to_float_, 0, float_to_short_1_, 0);
            top_block->connect(complex_to_float_, 1, float_to_short_2_, 0);
            top_block->connect(float_to_short_1_, 0, short_x2_to_cshort_, 0);
            top_block->connect(float_to_short_2_, 0, short_x2_to_cshort_, 0);
            if (dump_)
                {
                    top_block->connect(short_x2_to_cshort_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "byte") && (output_item_type_ == "gr_complex"))
        {
            top_block->connect(gr_char_to_short_, 0, freq_xlating_fir_filter_scf_, 0);
            if (dump_)
                {
                    top_block->connect(freq_xlating_fir_filter_scf_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "byte") && (output_item_type_ == "cbyte"))
        {
            top_block->connect(gr_char_to_short_, 0, freq_xlating_fir_filter_scf_, 0);
            top_block->connect(freq_xlating_fir_filter_scf_, 0, complex_to_complex_byte_, 0);
            if (dump_)
                {
                    top_block->connect(complex_to_complex_byte_, 0, file_sink_, 0);
                }
        }
    else
        {
            LOG(ERROR) << " Unknown input filter input/output item type conversion";
        }
}


void FreqXlatingFirFilter::disconnect(gr::top_block_sptr top_block)
{
    if ((taps_item_type_ == "float") && (input_item_type_ == "gr_complex") && (output_item_type_ == "gr_complex"))
        {
            if (dump_)
                {
                    top_block->disconnect(freq_xlating_fir_filter_ccf_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "float") && (output_item_type_ == "gr_complex"))
        {
            if (dump_)
                {
                    top_block->disconnect(freq_xlating_fir_filter_fcf_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "short") && (output_item_type_ == "gr_complex"))
        {
            if (dump_)
                {
                    top_block->disconnect(freq_xlating_fir_filter_scf_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "short") && (output_item_type_ == "cshort"))
        {
            top_block->disconnect(freq_xlating_fir_filter_scf_, 0, complex_to_float_, 0);
            top_block->disconnect(complex_to_float_, 0, float_to_short_1_, 0);
            top_block->disconnect(complex_to_float_, 1, float_to_short_2_, 0);
            top_block->disconnect(float_to_short_1_, 0, short_x2_to_cshort_, 0);
            top_block->disconnect(float_to_short_2_, 0, short_x2_to_cshort_, 0);
            if (dump_)
                {
                    top_block->disconnect(short_x2_to_cshort_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "byte") && (output_item_type_ == "gr_complex"))
        {
            top_block->disconnect(gr_char_to_short_, 0, freq_xlating_fir_filter_scf_, 0);
            if (dump_)
                {
                    top_block->disconnect(freq_xlating_fir_filter_scf_, 0, file_sink_, 0);
                }
        }
    else if ((taps_item_type_ == "float") && (input_item_type_ == "byte") && (output_item_type_ == "cbyte"))
        {
            top_block->disconnect(gr_char_to_short_, 0, freq_xlating_fir_filter_scf_, 0);
            top_block->disconnect(freq_xlating_fir_filter_scf_, 0, complex_to_complex_byte_, 0);
            if (dump_)
                {
                    top_block->disconnect(complex_to_complex_byte_, 0, file_sink_, 0);
                }
        }
    else
        {
            LOG(ERROR) << " Unknown input filter input/output item type conversion";
        }
}


gr::basic_block_sptr FreqXlatingFirFilter::get_left_block()
{
    if ((taps_item_type_ == "float") && (input_item_type_ == "gr_complex") && (output_item_type_ == "gr_complex"))
        {
            return freq_xlating_fir_filter_ccf_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "float") && (output_item_type_ == "gr_complex"))
        {
            return freq_xlating_fir_filter_fcf_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "short") && (output_item_type_ == "gr_complex"))
        {
            return freq_xlating_fir_filter_scf_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "short") && (output_item_type_ == "cshort"))
        {
            return freq_xlating_fir_filter_scf_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "byte") && (output_item_type_ == "gr_complex"))
        {
            return gr_char_to_short_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "byte") && (output_item_type_ == "cbyte"))
        {
            return gr_char_to_short_;
        }

    LOG(WARNING) << " Unknown input filter input/output item type conversion";
    return nullptr;
}


gr::basic_block_sptr FreqXlatingFirFilter::get_right_block()
{
    if ((taps_item_type_ == "float") && (input_item_type_ == "gr_complex") && (output_item_type_ == "gr_complex"))
        {
            return freq_xlating_fir_filter_ccf_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "float") && (output_item_type_ == "gr_complex"))
        {
            return freq_xlating_fir_filter_fcf_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "short") && (output_item_type_ == "gr_complex"))
        {
            return freq_xlating_fir_filter_scf_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "short") && (output_item_type_ == "cshort"))
        {
            return short_x2_to_cshort_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "byte") && (output_item_type_ == "gr_complex"))
        {
            return freq_xlating_fir_filter_scf_;
        }
    if ((taps_item_type_ == "float") && (input_item_type_ == "byte") && (output_item_type_ == "cbyte"))
        {
            return complex_to_complex_byte_;
        }

    LOG(WARNING) << " Unknown input filter input/output item type conversion";
    return nullptr;
}
