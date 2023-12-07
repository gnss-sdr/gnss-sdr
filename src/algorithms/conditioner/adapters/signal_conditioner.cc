/*!
 * \file signal_conditioner.cc
 * \brief It holds blocks to change data type, filter and resample input data.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
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

#include "signal_conditioner.h"
#include <glog/logging.h>
#include <stdexcept>
#include <utility>


// Constructor
SignalConditioner::SignalConditioner(std::shared_ptr<GNSSBlockInterface> data_type_adapt,
    std::shared_ptr<GNSSBlockInterface> in_filt,
    std::shared_ptr<GNSSBlockInterface> res,
    std::string role) : data_type_adapt_(std::move(data_type_adapt)),
                        in_filt_(std::move(in_filt)),
                        res_(std::move(res)),
                        role_(std::move(role)),
                        connected_(false)
{
}


void SignalConditioner::connect(gr::top_block_sptr top_block)
{
    if (connected_)
        {
            LOG(WARNING) << "Signal conditioner already connected internally";
            return;
        }
    if (data_type_adapt_ == nullptr)
        {
            throw std::invalid_argument("DataTypeAdapter implementation not defined");
        }
    if (in_filt_ == nullptr)
        {
            throw std::invalid_argument("InputFilter implementation not defined");
        }
    if (res_ == nullptr)
        {
            throw std::invalid_argument("Resampler implementation not defined");
        }
    data_type_adapt_->connect(top_block);
    in_filt_->connect(top_block);
    res_->connect(top_block);

    if (in_filt_->item_size() == 0)
        {
            throw std::invalid_argument("itemsize mismatch: Invalid input/ouput data type configuration for the InputFilter");
        }

    const size_t data_type_adapter_output_size = data_type_adapt_->get_right_block()->output_signature()->sizeof_stream_item(0);
    const size_t input_filter_input_size = in_filt_->get_left_block()->input_signature()->sizeof_stream_item(0);
    const size_t input_filter_output_size = in_filt_->get_right_block()->output_signature()->sizeof_stream_item(0);
    const size_t resampler_input_size = res_->get_left_block()->input_signature()->sizeof_stream_item(0);

    if (data_type_adapter_output_size != input_filter_input_size)
        {
            throw std::invalid_argument("itemsize mismatch: Invalid input/ouput data type configuration for the DataTypeAdapter/InputFilter connection");
        }

    if (input_filter_output_size != resampler_input_size)
        {
            throw std::invalid_argument("itemsize mismatch: Invalid input/ouput data type configuration for the Input Filter/Resampler connection");
        }

    top_block->connect(data_type_adapt_->get_right_block(), 0, in_filt_->get_left_block(), 0);
    DLOG(INFO) << "data_type_adapter -> input_filter";

    top_block->connect(in_filt_->get_right_block(), 0, res_->get_left_block(), 0);
    DLOG(INFO) << "input_filter -> resampler";
    connected_ = true;
}


void SignalConditioner::disconnect(gr::top_block_sptr top_block)
{
    if (!connected_)
        {
            LOG(WARNING) << "Signal conditioner already disconnected internally";
            return;
        }

    top_block->disconnect(data_type_adapt_->get_right_block(), 0,
        in_filt_->get_left_block(), 0);
    top_block->disconnect(in_filt_->get_right_block(), 0,
        res_->get_left_block(), 0);

    data_type_adapt_->disconnect(top_block);
    in_filt_->disconnect(top_block);
    res_->disconnect(std::move(top_block));

    connected_ = false;
}


gr::basic_block_sptr SignalConditioner::get_left_block()
{
    return data_type_adapt_->get_left_block();
}


gr::basic_block_sptr SignalConditioner::get_right_block()
{
    return res_->get_right_block();
}
