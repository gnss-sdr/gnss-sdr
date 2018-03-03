/*!
 * \file array_signal_conditioner.cc
 * \brief It wraps blocks to change data type, filter and resample input data, adapted to array receiver
 * \author Javier Arribas jarribas (at) cttc.es
 *
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

#include "array_signal_conditioner.h"
#include <glog/logging.h>


using google::LogMessage;

// Constructor
ArraySignalConditioner::ArraySignalConditioner(ConfigurationInterface *configuration,
    std::shared_ptr<GNSSBlockInterface> data_type_adapt, std::shared_ptr<GNSSBlockInterface> in_filt,
    std::shared_ptr<GNSSBlockInterface> res, std::string role, std::string implementation) : data_type_adapt_(data_type_adapt),
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
ArraySignalConditioner::~ArraySignalConditioner() {}


void ArraySignalConditioner::connect(gr::top_block_sptr top_block)
{
    // note: the array signal conditioner do not have data type adapter, and must use the array input filter (multichannel)
    if (connected_)
        {
            LOG(WARNING) << "Array Signal conditioner already connected internally";
            return;
        }
    //data_type_adapt_->connect(top_block);
    in_filt_->connect(top_block);
    res_->connect(top_block);

    //top_block->connect(data_type_adapt_->get_right_block(), 0, in_filt_->get_left_block(), 0);
    //DLOG(INFO) << "data_type_adapter -> input_filter";

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

    //top_block->disconnect(data_type_adapt_->get_right_block(), 0,
    //                      in_filt_->get_left_block(), 0);
    top_block->disconnect(in_filt_->get_right_block(), 0,
        res_->get_left_block(), 0);

    //data_type_adapt_->disconnect(top_block);
    in_filt_->disconnect(top_block);
    res_->disconnect(top_block);

    connected_ = false;
}


gr::basic_block_sptr ArraySignalConditioner::get_left_block()
{
    //return data_type_adapt_->get_left_block();
    return in_filt_->get_left_block();
}


gr::basic_block_sptr ArraySignalConditioner::get_right_block()
{
    return res_->get_right_block();
}
