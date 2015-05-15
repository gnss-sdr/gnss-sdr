/*!
 * \file gps_l1_ca_observables.cc
 * \brief Implementation of an adapter of a GPS L1 C/A observables block
 * to a ObservablesInterface
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
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


#include "gps_l1_ca_observables.h"
#include "configuration_interface.h"
#include "gps_l1_ca_observables_cc.h"
#include <glog/logging.h>

using google::LogMessage;

GpsL1CaObservables::GpsL1CaObservables(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
                    role_(role),
                    in_streams_(in_streams),
                    out_streams_(out_streams),
                    queue_(queue)
{
    int output_rate_ms;
    output_rate_ms = configuration->property(role + ".output_rate_ms", 500);
    std::string default_dump_filename = "./observables.dat";
    DLOG(INFO) << "role " << role;
    bool flag_averaging;
    flag_averaging = configuration->property(role + ".flag_averaging", false);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    //fs_in_ = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    observables_ = gps_l1_ca_make_observables_cc(in_streams_, queue_, dump_, dump_filename_, output_rate_ms, flag_averaging);
    //observables_->set_fs_in(fs_in_);
    DLOG(INFO) << "pseudorange(" << observables_->unique_id() << ")";
}




GpsL1CaObservables::~GpsL1CaObservables()
{}




void GpsL1CaObservables::connect(gr::top_block_sptr top_block)
{
    if(top_block) { /* top_block is not null */};
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}



void GpsL1CaObservables::disconnect(gr::top_block_sptr top_block)
{
    if(top_block) { /* top_block is not null */};
    // Nothing to disconnect
}




gr::basic_block_sptr GpsL1CaObservables::get_left_block()
{
    return observables_;
}




gr::basic_block_sptr GpsL1CaObservables::get_right_block()
{
    return observables_;
}

