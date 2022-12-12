/*!
 * \file ishort_to_cshort.cc
 * \brief Adapts an I/Q interleaved short integer (16 bits) sample stream
 * to a gr_complex (float) stream
 * \author Carles Fernandez-Prades, cfernandez(at)cttc.es
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

#include "ishort_to_cshort.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <volk/volk.h>


IshortToCshort::IshortToCshort(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams),
                                inverted_spectrum(configuration->property(role + ".inverted_spectrum", false)),
                                dump_(configuration->property(role + ".dump", false))
{
    const std::string default_input_item_type("short");
    const std::string default_output_item_type("cshort");
    const std::string default_dump_filename("../data/input_filter.dat");

    DLOG(INFO) << "role " << role_;

    input_item_type_ = configuration->property(role_ + ".input_item_type", default_input_item_type);
    dump_filename_ = configuration->property(role_ + ".dump_filename", default_dump_filename);

    interleaved_short_to_complex_short_ = make_interleaved_short_to_complex_short();

    DLOG(INFO) << "data_type_adapter_(" << interleaved_short_to_complex_short_->unique_id() << ")";

    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            const size_t item_size = sizeof(lv_16sc_t);
            file_sink_ = gr::blocks::file_sink::make(item_size, dump_filename_.c_str());
        }
    if (inverted_spectrum)
        {
            conjugate_sc_ = make_conjugate_sc();
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


void IshortToCshort::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            if (inverted_spectrum)
                {
                    top_block->connect(interleaved_short_to_complex_short_, 0, conjugate_sc_, 0);
                    top_block->connect(conjugate_sc_, 0, file_sink_, 0);
                }
            else
                {
                    top_block->connect(interleaved_short_to_complex_short_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (inverted_spectrum)
                {
                    top_block->connect(interleaved_short_to_complex_short_, 0, conjugate_sc_, 0);
                }
            else
                {
                    DLOG(INFO) << "Nothing to connect internally";
                }
        }
}


void IshortToCshort::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            if (inverted_spectrum)
                {
                    top_block->disconnect(interleaved_short_to_complex_short_, 0, conjugate_sc_, 0);
                    top_block->disconnect(conjugate_sc_, 0, file_sink_, 0);
                }
            else
                {
                    top_block->disconnect(interleaved_short_to_complex_short_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (inverted_spectrum)
                {
                    top_block->disconnect(interleaved_short_to_complex_short_, 0, conjugate_sc_, 0);
                }
        }
}


gr::basic_block_sptr IshortToCshort::get_left_block()
{
    return interleaved_short_to_complex_short_;
}


gr::basic_block_sptr IshortToCshort::get_right_block()
{
    if (inverted_spectrum)
        {
            return conjugate_sc_;
        }
    return interleaved_short_to_complex_short_;
}
