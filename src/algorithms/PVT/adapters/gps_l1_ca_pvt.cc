/*!
 * \file gps_l1_ca_pvt.cc
 * \brief Simple Least Squares implementation for GPS L1 C/A Position Velocity and Time
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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


#include "gps_l1_ca_pvt.h"
#include "configuration_interface.h"
#include "gps_l1_ca_pvt_cc.h"
#include <gnuradio/gr_io_signature.h>
#include <glog/log_severity.h>
#include <glog/logging.h>

extern concurrent_queue<gps_navigation_message> global_gps_nav_msg_queue;

using google::LogMessage;

GpsL1CaPvt::GpsL1CaPvt(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams,
        gr_msg_queue_sptr queue) :
        role_(role),
        in_streams_(in_streams),
        out_streams_(out_streams),
        queue_(queue)
{

    std::string default_dump_filename = "./pvt.dat";

    DLOG(INFO) << "role " << role;

	int averaging_depth;
	averaging_depth=configuration->property(role + ".averaging_depth", 10);

    bool flag_averaging;
    flag_averaging=configuration->property(role + ".flag_averaging", false);

    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);

	pvt_ = gps_l1_ca_make_pvt_cc(in_streams_, queue_, dump_, dump_filename_, averaging_depth, flag_averaging);

    DLOG(INFO) << "pvt(" << pvt_->unique_id() << ")";
    // set the navigation msg queue;

    pvt_->set_navigation_queue(&global_gps_nav_msg_queue);

    DLOG(INFO) << "global navigation message queue assigned to pvt ("<< pvt_->unique_id() << ")";

}

GpsL1CaPvt::~GpsL1CaPvt()
{}


void GpsL1CaPvt::connect(gr_top_block_sptr top_block)
{
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}

void GpsL1CaPvt::disconnect(gr_top_block_sptr top_block)
{
    // Nothing to disconnect
}

gr_basic_block_sptr GpsL1CaPvt::get_left_block()
{
    return pvt_;
}

gr_basic_block_sptr GpsL1CaPvt::get_right_block()
{
    return pvt_;
}

