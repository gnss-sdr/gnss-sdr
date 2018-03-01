/*!
 * \file ibyte_to_cbyte.cc
 * \brief Adapts an I/Q interleaved byte (unsigned char) sample stream
 * into a std::complex<unsigned char> stream
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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

#include "ibyte_to_cbyte.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <volk/volk.h>

using google::LogMessage;

IbyteToCbyte::IbyteToCbyte(ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams) :
                config_(configuration), role_(role), in_streams_(in_streams),
                out_streams_(out_streams)
{
    std::string default_input_item_type = "byte";
    std::string default_output_item_type = "lv_8sc_t";
    std::string default_dump_filename = "../data/input_filter.dat";

    DLOG(INFO) << "role " << role_;

    input_item_type_ = config_->property(role_ + ".input_item_type", default_input_item_type);

    dump_ = config_->property(role_ + ".dump", false);
    dump_filename_ = config_->property(role_ + ".dump_filename", default_dump_filename);
    inverted_spectrum = configuration->property(role + ".inverted_spectrum", false);

    size_t item_size = sizeof(lv_8sc_t);

    ibyte_to_cbyte_ = make_interleaved_byte_to_complex_byte();

    DLOG(INFO) << "data_type_adapter_(" << ibyte_to_cbyte_->unique_id() << ")";

    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            file_sink_ = gr::blocks::file_sink::make(item_size, dump_filename_.c_str());
        }
    if(inverted_spectrum)
        {
            conjugate_ic_ = make_conjugate_ic();
        }
}


IbyteToCbyte::~IbyteToCbyte()
{}


void IbyteToCbyte::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            if(inverted_spectrum)
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
            if(inverted_spectrum)
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
            if(inverted_spectrum)
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
            if(inverted_spectrum)
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
    if(inverted_spectrum)
        {
            return conjugate_ic_;
        }
    else
        {
            return ibyte_to_cbyte_;
        }
}
