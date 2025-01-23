/*!
 * \file cshort_to_grcomplex.cc
 * \brief Adapts an 16-bits complex sample stream to a float complex stream
 * \author Carles Fernandez Prades, 2014 cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "cshort_to_grcomplex.h"
#include "configuration_interface.h"
#include <utility>
#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


CshortToGrComplex::CshortToGrComplex(const ConfigurationInterface* configuration,
    std::string role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(std::move(role)),
                                in_streams_(in_streams),
                                out_streams_(out_streams),
                                dump_(configuration->property(role_ + ".dump", false))
{
    const std::string default_dump_filename("./data_type_adapter.dat");

    DLOG(INFO) << "role " << role_;

    dump_filename_ = configuration->property(role_ + ".dump_filename", default_dump_filename);
    cshort_to_gr_complex_ = make_cshort_to_gr_complex();

    DLOG(INFO) << "data_type_adapter_(" << cshort_to_gr_complex_->unique_id() << ")";

    if (dump_)
        {
            DLOG(INFO) << "Dumping output into file " << dump_filename_;
            const size_t item_size = sizeof(gr_complex);
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


void CshortToGrComplex::connect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->connect(cshort_to_gr_complex_, 0, file_sink_, 0);
        }
    else
        {
            DLOG(INFO) << "Nothing to connect internally";
        }
}


void CshortToGrComplex::disconnect(gr::top_block_sptr top_block)
{
    if (dump_)
        {
            top_block->disconnect(cshort_to_gr_complex_, 0, file_sink_, 0);
        }
}


gr::basic_block_sptr CshortToGrComplex::get_left_block()
{
    return cshort_to_gr_complex_;
}


gr::basic_block_sptr CshortToGrComplex::get_right_block()
{
    return cshort_to_gr_complex_;
}
