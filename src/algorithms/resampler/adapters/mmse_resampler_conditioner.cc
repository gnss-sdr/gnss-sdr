/*!
 * \file mmse_resampler_conditioner.cc
 * \brief Implementation of an adapter of a MMSE resampler conditioner block
 * to a SignalConditionerInterface
 * \author Antonio Ramos, 2018. antonio.ramos(at)cttc.es
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

#include "mmse_resampler_conditioner.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <gnuradio/blocks/file_sink.h>
#include <cmath>
#include <limits>
#include <vector>

MmseResamplerConditioner::MmseResamplerConditioner(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_stream,
    unsigned int out_stream)
    : role_(role),
      in_stream_(in_stream),
      out_stream_(out_stream),
      dump_(configuration->property(role + ".dump", false))
{
    const std::string default_item_type("gr_complex");
    const std::string default_dump_file("./data/signal_conditioner.dat");
    const double fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000.0);
    const double fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    item_type_ = configuration->property(role + ".item_type", default_item_type);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_file);
    sample_freq_in_ = configuration->property(role_ + ".sample_freq_in", 4000000.0);
    sample_freq_out_ = configuration->property(role_ + ".sample_freq_out", fs_in);

    if (std::fabs(fs_in - sample_freq_out_) > std::numeric_limits<double>::epsilon())
        {
            std::string aux_warn = "CONFIGURATION WARNING: Parameters GNSS-SDR.internal_fs_sps and " + role_ + ".sample_freq_out are not set to the same value!";
            LOG(WARNING) << aux_warn;
            std::cout << aux_warn << '\n';
        }

    if (item_type_ == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);

            // create a FIR low pass filter
            std::vector<float> taps = gr::filter::firdes::low_pass(1.0,
                sample_freq_in_,
                sample_freq_out_ / 2.1,
                sample_freq_out_ / 5);
            std::cout << "Enabled fractional resampler low pass filter with " << taps.size() << " taps\n";
            fir_filter_ccf_ = gr::filter::fir_filter_ccf::make(1, taps);

#ifdef GR_GREATER_38
            resampler_ = gr::filter::mmse_resampler_cc::make(0.0, static_cast<float>(sample_freq_in_ / sample_freq_out_));
#else
            resampler_ = gr::filter::fractional_resampler_cc::make(0.0, static_cast<float>(sample_freq_in_ / sample_freq_out_));
#endif
            DLOG(INFO) << "sample_freq_in " << sample_freq_in_;
            DLOG(INFO) << "sample_freq_out" << sample_freq_out_;
            DLOG(INFO) << "Item size " << item_size_;
            DLOG(INFO) << "resampler(" << resampler_->unique_id() << ")";
        }
    else
        {
            LOG(WARNING) << item_type_ << " unrecognized item type for resampler";
            item_size_ = sizeof(gr_complex);
        }
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
            DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }
    if (in_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_stream_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void MmseResamplerConditioner::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->connect(fir_filter_ccf_, 0, resampler_, 0);
            top_block->connect(resampler_, 0, file_sink_, 0);
            DLOG(INFO) << "connected resampler to file sink";
        }
    else
        {
            top_block->connect(fir_filter_ccf_, 0, resampler_, 0);
        }
}


void MmseResamplerConditioner::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->disconnect(fir_filter_ccf_, 0, resampler_, 0);
            top_block->disconnect(resampler_, 0, file_sink_, 0);
        }
    else
        {
            top_block->disconnect(fir_filter_ccf_, 0, resampler_, 0);
        }
}


gr::basic_block_sptr MmseResamplerConditioner::get_left_block()
{
    return fir_filter_ccf_;
}


gr::basic_block_sptr MmseResamplerConditioner::get_right_block()
{
    return resampler_;
}
