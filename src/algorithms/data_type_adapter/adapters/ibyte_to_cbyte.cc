/*!
 * \file ibyte_to_cbyte.cc
 * \brief Adapts an I/Q interleaved byte (unsigned char) sample stream
 * into a std::complex<unsigned char> stream
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
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

#include "ibyte_to_cbyte.h"
#include "configuration_interface.h"
#include <volk/volk.h>
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


IbyteToCbyte::IbyteToCbyte(const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams),
                                inverted_spectrum(configuration->property(role + ".inverted_spectrum", false)),
                                dump_(configuration->property(role + ".dump", false))
{
    const std::string default_input_item_type("byte");
    const std::string default_output_item_type("lv_8sc_t");
    const std::string default_dump_filename("./data_type_adapter.dat");

    DLOG(INFO) << "role " << role_;

    input_item_type_ = configuration->property(role_ + ".input_item_type", default_input_item_type);
    dump_filename_ = configuration->property(role_ + ".dump_filename", default_dump_filename);

    ibyte_to_cbyte_ = make_interleaved_byte_to_complex_byte();

    DLOG(INFO) << "data_type_adapter_(" << ibyte_to_cbyte_->unique_id() << ")";

    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            const size_t item_size = sizeof(lv_8sc_t);
            file_sink_ = gr::blocks::file_sink::make(item_size, dump_filename_.c_str());
        }
    if (inverted_spectrum)
        {
            conjugate_ic_ = make_conjugate_ic();
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


void IbyteToCbyte::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            if (inverted_spectrum)
                {
                    top_block->connect(ibyte_to_cbyte_, 0, conjugate_ic_, 0);
                    top_block->connect(conjugate_ic_, 0, file_sink_, 0);
                }
            else
                {
                    top_block->connect(ibyte_to_cbyte_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (inverted_spectrum)
                {
                    top_block->connect(ibyte_to_cbyte_, 0, conjugate_ic_, 0);
                }
            else
                {
                    DLOG(INFO) << "Nothing to connect internally";
                }
        }
}


void IbyteToCbyte::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            if (inverted_spectrum)
                {
                    top_block->disconnect(ibyte_to_cbyte_, 0, conjugate_ic_, 0);
                    top_block->disconnect(conjugate_ic_, 0, file_sink_, 0);
                }
            else
                {
                    top_block->disconnect(ibyte_to_cbyte_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (inverted_spectrum)
                {
                    top_block->disconnect(ibyte_to_cbyte_, 0, conjugate_ic_, 0);
                }
        }
}


gr::basic_block_sptr IbyteToCbyte::get_left_block()
{
    return ibyte_to_cbyte_;
}


gr::basic_block_sptr IbyteToCbyte::get_right_block()
{
    if (inverted_spectrum)
        {
            return conjugate_ic_;
        }
    return ibyte_to_cbyte_;
}
