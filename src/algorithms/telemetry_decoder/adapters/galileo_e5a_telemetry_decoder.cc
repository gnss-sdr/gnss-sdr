/*!
 * \file galileo_e5a_telemetry_decoder.h
 * \brief Interface of an adapter of a GALILEO E5a FNAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 * 		<ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          </ul>
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

#include "galileo_e5a_telemetry_decoder.h"
#include <boost/shared_ptr.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "galileo_ephemeris.h"
#include "galileo_almanac.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "configuration_interface.h"


extern concurrent_queue<Galileo_Ephemeris> global_galileo_ephemeris_queue;
extern concurrent_queue<Galileo_Iono> global_galileo_iono_queue;
extern concurrent_queue<Galileo_Utc_Model> global_galileo_utc_model_queue;
extern concurrent_queue<Galileo_Almanac> global_galileo_almanac_queue;


using google::LogMessage;

GalileoE5aTelemetryDecoder::GalileoE5aTelemetryDecoder(ConfigurationInterface* configuration,
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
    telemetry_decoder_ = galileo_e5a_make_telemetry_decoder_cc(satellite_, 0, (long)fs_in, vector_length_, queue_, dump_); // TODO fix me
    DLOG(INFO) << "telemetry_decoder(" << telemetry_decoder_->unique_id() << ")";
    // set the navigation msg queue;
    telemetry_decoder_->set_ephemeris_queue(&global_galileo_ephemeris_queue);
    telemetry_decoder_->set_iono_queue(&global_galileo_iono_queue);
    telemetry_decoder_->set_almanac_queue(&global_galileo_almanac_queue);
    telemetry_decoder_->set_utc_model_queue(&global_galileo_utc_model_queue);

    DLOG(INFO) << "global navigation message queue assigned to telemetry_decoder ("<< telemetry_decoder_->unique_id() << ")";
    channel_ = 0;
}


GalileoE5aTelemetryDecoder::~GalileoE5aTelemetryDecoder()
{}


void GalileoE5aTelemetryDecoder::set_satellite(Gnss_Satellite satellite)
{
    satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    telemetry_decoder_->set_satellite(satellite_);
    DLOG(INFO) << "TELEMETRY DECODER: satellite set to " << satellite_;
}


void GalileoE5aTelemetryDecoder::connect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	// Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void GalileoE5aTelemetryDecoder::disconnect(gr::top_block_sptr top_block)
{
	if(top_block) { /* top_block is not null */};
	// Nothing to disconnect
}


gr::basic_block_sptr GalileoE5aTelemetryDecoder::get_left_block()
{
    return telemetry_decoder_;
}


gr::basic_block_sptr GalileoE5aTelemetryDecoder::get_right_block()
{
    return telemetry_decoder_;
}
