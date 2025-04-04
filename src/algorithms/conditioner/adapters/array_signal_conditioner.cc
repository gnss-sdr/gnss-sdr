/*!
 * \file array_signal_conditioner.cc
 * \brief It wraps blocks to change data type, filter and resample input data, adapted to array receiver
 * \author Javier Arribas jarribas (at) cttc.es
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

#include "array_signal_conditioner.h"
#include "configuration_interface.h"
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

// Constructor
ArraySignalConditioner::ArraySignalConditioner(std::shared_ptr<GNSSBlockInterface> data_type_adapt,
    std::shared_ptr<GNSSBlockInterface> in_filt,
    std::shared_ptr<GNSSBlockInterface> res,
    std::string role) : data_type_adapt_(std::move(data_type_adapt)),
                        in_filt_(std::move(in_filt)),
                        res_(std::move(res)),
                        role_(std::move(role)),
                        connected_(false)
{
}


void ArraySignalConditioner::connect(gr::top_block_sptr top_block)
{
    // note: the array signal conditioner do not have data type adapter, and must use the array input filter (multichannel)
    if (connected_)
        {
            LOG(WARNING) << "Array Signal conditioner already connected internally";
            return;
        }
    // data_type_adapt_->connect(top_block);
    in_filt_->connect(top_block);
    res_->connect(top_block);

    // top_block->connect(data_type_adapt_->get_right_block(), 0, in_filt_->get_left_block(), 0);
    // DLOG(INFO) << "data_type_adapter -> input_filter";

    top_block->connect(in_filt_->get_right_block(), 0,
        res_->get_left_block(), 0);

    DLOG(INFO) << "Array input_filter -> resampler";

    connected_ = true;
}


void ArraySignalConditioner::disconnect(gr::top_block_sptr top_block)
{
    if (!connected_)
        {
            LOG(WARNING) << "Array Signal conditioner already disconnected internally";
            return;
        }

    // top_block->disconnect(data_type_adapt_->get_right_block(), 0,
    //                      in_filt_->get_left_block(), 0);
    top_block->disconnect(in_filt_->get_right_block(), 0,
        res_->get_left_block(), 0);

    // data_type_adapt_->disconnect(top_block);
    in_filt_->disconnect(top_block);
    res_->disconnect(std::move(top_block));

    connected_ = false;
}


gr::basic_block_sptr ArraySignalConditioner::get_left_block()
{
    // return data_type_adapt_->get_left_block();
    return in_filt_->get_left_block();
}


gr::basic_block_sptr ArraySignalConditioner::get_right_block()
{
    return res_->get_right_block();
}
