/*!
 * \file ishort_to_complex.cc
 * \brief Adapts an I/Q interleaved short integer sample stream to a gr_complex (float) stream
 * \author Javier Arribas, jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#include "ishort_to_complex.h"
#include "configuration_interface.h"
#include <glog/logging.h>


IshortToComplex::IshortToComplex(ConfigurationInterface* configuration, const std::string& role,
    unsigned int in_streams, unsigned int out_streams) : config_(configuration), role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    std::string default_input_item_type = "short";
    std::string default_output_item_type = "gr_complex";
    std::string default_dump_filename = "../data/input_filter.dat";

    DLOG(INFO) << "role " << role_;

    input_item_type_ = config_->property(role_ + ".input_item_type", default_input_item_type);

    dump_ = config_->property(role_ + ".dump", false);
    dump_filename_ = config_->property(role_ + ".dump_filename", default_dump_filename);
    inverted_spectrum = configuration->property(role + ".inverted_spectrum", false);

    size_t item_size = sizeof(gr_complex);

    gr_interleaved_short_to_complex_ = gr::blocks::interleaved_short_to_complex::make();

    DLOG(INFO) << "data_type_adapter_(" << gr_interleaved_short_to_complex_->unique_id() << ")";

    if (inverted_spectrum)
        {
            conjugate_cc_ = make_conjugate_cc();
        }
    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
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


void IshortToComplex::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            if (inverted_spectrum)
                {
                    top_block->connect(gr_interleaved_short_to_complex_, 0, conjugate_cc_, 0);
                    top_block->connect(conjugate_cc_, 0, file_sink_, 0);
                }
            else
                {
                    top_block->connect(gr_interleaved_short_to_complex_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (inverted_spectrum)
                {
                    top_block->connect(gr_interleaved_short_to_complex_, 0, conjugate_cc_, 0);
                }
            else
                {
                    DLOG(INFO) << "Nothing to connect internally";
                }
        }
}


void IshortToComplex::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            if (inverted_spectrum)
                {
                    top_block->disconnect(gr_interleaved_short_to_complex_, 0, conjugate_cc_, 0);
                    top_block->disconnect(conjugate_cc_, 0, file_sink_, 0);
                }
            else
                {
                    top_block->disconnect(gr_interleaved_short_to_complex_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (inverted_spectrum)
                {
                    top_block->disconnect(gr_interleaved_short_to_complex_, 0, conjugate_cc_, 0);
                }
        }
}


gr::basic_block_sptr IshortToComplex::get_left_block()
{
    return gr_interleaved_short_to_complex_;
}


gr::basic_block_sptr IshortToComplex::get_right_block()
{
    if (inverted_spectrum)
        {
            return conjugate_cc_;
        }
    return gr_interleaved_short_to_complex_;
}
