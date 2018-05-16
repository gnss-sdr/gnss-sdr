/*!
 * \file sbas_l1_telemetry_decoder.cc
 * \brief Implementation of an adapter of a SBAS telemetry data decoder block
 * to a TelemetryDecoderInterface
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
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


#include "sbas_l1_telemetry_decoder.h"
#include "configuration_interface.h"
#include "sbas_ephemeris.h"
#include "sbas_l1_telemetry_decoder_cc.h"
#include <gnuradio/io_signature.h>
#include <glog/logging.h>


using google::LogMessage;

SbasL1TelemetryDecoder::SbasL1TelemetryDecoder(ConfigurationInterface* configuration,
    std::string role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    std::string default_dump_filename = "./navigation.dat";
    DLOG(INFO) << "role " << role;
    dump_ = configuration->property(role + ".dump", false);
    dump_filename_ = configuration->property(role + ".dump_filename", default_dump_filename);
    // make telemetry decoder object
    telemetry_decoder_ = sbas_l1_make_telemetry_decoder_cc(satellite_, dump_);  // TODO fix me
    channel_ = 0;
    DLOG(INFO) << "telemetry_decoder(" << telemetry_decoder_->unique_id() << ")";
}


SbasL1TelemetryDecoder::~SbasL1TelemetryDecoder()
{
}


void SbasL1TelemetryDecoder::set_satellite(const Gnss_Satellite& satellite)
{
    satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    telemetry_decoder_->set_satellite(satellite_);
    DLOG(INFO) << "SBAS TELEMETRY DECODER: satellite set to " << satellite_;
}


void SbasL1TelemetryDecoder::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void SbasL1TelemetryDecoder::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
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
