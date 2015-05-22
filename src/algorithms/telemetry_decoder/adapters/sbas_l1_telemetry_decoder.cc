/*!
 * \file sbas_l1_telemetry_decoder.cc
 * \brief Implementation of an adapter of a SBAS telemetry data decoder block
 * to a TelemetryDecoderInterface
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
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


#include "sbas_l1_telemetry_decoder.h"
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "sbas_telemetry_data.h"
#include "sbas_ionospheric_correction.h"
#include "sbas_satellite_correction.h"
#include "sbas_ephemeris.h"
#include "configuration_interface.h"
#include "sbas_l1_telemetry_decoder_cc.h"

extern concurrent_queue<Sbas_Raw_Msg> global_sbas_raw_msg_queue;
extern concurrent_queue<Sbas_Ionosphere_Correction> global_sbas_iono_queue;
extern concurrent_queue<Sbas_Satellite_Correction> global_sbas_sat_corr_queue;
extern concurrent_queue<Sbas_Ephemeris> global_sbas_ephemeris_queue;


using google::LogMessage;

SbasL1TelemetryDecoder::SbasL1TelemetryDecoder(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams,
        boost::shared_ptr<gr::msg_queue> queue) :
        role_(role),
        in_streams_(in_streams),
        out_streams_(out_streams),
        queue_(queue)
{
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./navigation.dat";
    DLOG(INFO) << "role " << role;
    DLOG(INFO) << "vector length " << vector_length_;
    vector_length_ = configuration->property(role + ".vector_length", 2048);
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    int fs_in;
    fs_in = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    // make telemetry decoder object
    telemetry_decoder_ = sbas_l1_make_telemetry_decoder_cc(satellite_, 0, (long)fs_in, vector_length_, queue_, dump_); // TODO fix me
    channel_ = 0;
    DLOG(INFO) << "telemetry_decoder(" << telemetry_decoder_->unique_id() << ")";
    // set the queues;
    telemetry_decoder_->set_raw_msg_queue(&global_sbas_raw_msg_queue);
    telemetry_decoder_->set_iono_queue(&global_sbas_iono_queue);
    telemetry_decoder_->set_sat_corr_queue(&global_sbas_sat_corr_queue);
    telemetry_decoder_->set_ephemeris_queue(&global_sbas_ephemeris_queue);
}


SbasL1TelemetryDecoder::~SbasL1TelemetryDecoder()
{}


void SbasL1TelemetryDecoder::set_satellite(Gnss_Satellite satellite)
{
    satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    telemetry_decoder_->set_satellite(satellite_);
    DLOG(INFO) << "SBAS TELEMETRY DECODER: satellite set to " << satellite_;
}


void SbasL1TelemetryDecoder::connect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	// Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void SbasL1TelemetryDecoder::disconnect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	// Nothing to disconnect
}


gr::basic_block_sptr SbasL1TelemetryDecoder::get_left_block()
{
    return telemetry_decoder_;
}


gr::basic_block_sptr SbasL1TelemetryDecoder::get_right_block()
{
    return telemetry_decoder_;
}

