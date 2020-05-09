/*!
 * \file byte_to_short.cc
 * \brief Adapts an 8-bits sample stream (IF) to a short int stream (IF)
 * \author Carles Fernandez Prades, cfernandez(at)cttc.es
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

#include "byte_to_short.h"
#include "configuration_interface.h"
#include <glog/logging.h>
#include <cstdint>
#include <utility>


ByteToShort::ByteToShort(ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : config_(configuration), role_(std::move(role)), in_streams_(in_streams), out_streams_(out_streams)
{
    std::string default_input_item_type = "byte";
    std::string default_output_item_type = "short";
    std::string default_dump_filename = "../data/input_filter.dat";

    DLOG(INFO) << "role " << role_;

    input_item_type_ = config_->property(role_ + ".input_item_type", default_input_item_type);

    dump_ = config_->property(role_ + ".dump", false);
    dump_filename_ = config_->property(role_ + ".dump_filename", default_dump_filename);

    size_t item_size = sizeof(int16_t);

    gr_char_to_short_ = gr::blocks::char_to_short::make();

    DLOG(INFO) << "data_type_adapter_(" << gr_char_to_short_->unique_id() << ")";

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


void ByteToShort::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->connect(gr_char_to_short_, 0, file_sink_, 0);
        }
    else
        {
            DLOG(INFO) << "Nothing to connect internally";
        }
}


void ByteToShort::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->disconnect(gr_char_to_short_, 0, file_sink_, 0);
        }
}


gr::basic_block_sptr ByteToShort::get_left_block()
{
    return gr_char_to_short_;
}


gr::basic_block_sptr ByteToShort::get_right_block()
{
    return gr_char_to_short_;
}
