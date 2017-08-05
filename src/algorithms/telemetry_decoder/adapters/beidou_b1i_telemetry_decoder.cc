/*!
 * \file beidou_b1i_telemetry_decoder.cc
 * \brief Implementation of an adapter of a BEIDOU B1I NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Enric Juan, 2016. enric.juan.92(at)gmail.com
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


#include "beidou_b1i_telemetry_decoder.h"
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "concurrent_queue.h"
#include "gps_ephemeris.h"
#include "gps_almanac.h"
#include "gps_iono.h"
#include "gps_utc_model.h"
#include "configuration_interface.h"

using google::LogMessage;

BeiDouB1iTelemetryDecoder::BeiDouB1iTelemetryDecoder(ConfigurationInterface* configuration,
                                                     std::string role,
                                                     unsigned int in_streams,
                                                     unsigned int out_streams) :
                                                     role_(role),
                                                     in_streams_(in_streams),
                                                     out_streams_(out_streams)
{
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./navigation.dat";
    DLOG(INFO) << "role " << role;
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    // make telemetry decoder object
    telemetry_decoder_ = beidou_b1i_make_telemetry_decoder_cc(satellite_, dump_); // TODO fix me
    DLOG(INFO) << "telemetry_decoder(" << telemetry_decoder_->unique_id() << ")";

    //decimation factor
    int decimation_factor = configuration->property(role + ".decimation_factor", 1);
    telemetry_decoder_->set_decimation(decimation_factor);
    DLOG(INFO) << "global navigation message queue assigned to telemetry_decoder ("<< telemetry_decoder_->unique_id() << ")";
    channel_ = 0;
}


BeiDouB1iTelemetryDecoder::~BeiDouB1iTelemetryDecoder()
{}


void BeiDouB1iTelemetryDecoder::set_satellite(Gnss_Satellite satellite)
{
    satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    telemetry_decoder_->set_satellite(satellite_);
    DLOG(INFO) << "TELEMETRY DECODER: satellite set to " << satellite_;
}


void BeiDouB1iTelemetryDecoder::connect(gr::top_block_sptr top_block)
{
    if(top_block) { /* top_block is not null */};
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void BeiDouB1iTelemetryDecoder::disconnect(gr::top_block_sptr top_block)
{
    if(top_block) { /* top_block is not null */};
    // Nothing to disconnect
}


gr::basic_block_sptr BeiDouB1iTelemetryDecoder::get_left_block()
{
    return telemetry_decoder_;
}


gr::basic_block_sptr BeiDouB1iTelemetryDecoder::get_right_block()
{
    return telemetry_decoder_;
}

