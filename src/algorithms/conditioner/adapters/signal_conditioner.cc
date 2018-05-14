/*!
 * \file signal_conditioner.cc
 * \brief It holds blocks to change data type, filter and resample input data.
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "signal_conditioner.h"
#include <glog/logging.h>


using google::LogMessage;

// Constructor
SignalConditioner::SignalConditioner(ConfigurationInterface *configuration,
    std::shared_ptr<GNSSBlockInterface> data_type_adapt,
    std::shared_ptr<GNSSBlockInterface> in_filt,
    std::shared_ptr<GNSSBlockInterface> res,
    std::string role,
    std::string implementation) : data_type_adapt_(data_type_adapt),
                                  in_filt_(in_filt),
                                  res_(res),
                                  role_(role),
                                  implementation_(implementation)
{
    connected_ = false;
    if (configuration)
        {
        };
}


// Destructor
SignalConditioner::~SignalConditioner() {}


void SignalConditioner::connect(gr::top_block_sptr top_block)
{
    if (connected_)
        {
            LOG(WARNING) << "Signal conditioner already connected internally";
            return;
        }
    data_type_adapt_->connect(top_block);
    in_filt_->connect(top_block);
    res_->connect(top_block);

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
    res_->disconnect(top_block);

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
