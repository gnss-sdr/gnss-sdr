/*!
 * \file galileo_e1b_telemetry_decoder.cc
 * \brief Implementation of an adapter of a Galileo INAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Javier Arribas 2013. jarribas(at)cttc.es,
 * Mara Branzanti 2013. mara.branzanti(at)gmail.com
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


#include "galileo_e1b_telemetry_decoder.h"
#include "configuration_interface.h"
#include <glog/logging.h>


GalileoE1BTelemetryDecoder::GalileoE1BTelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    DLOG(INFO) << "role " << role;
    tlm_parameters_.SetFromConfiguration(configuration, role);
    tlm_parameters_.enable_reed_solomon = configuration->property(role + ".enable_reed_solomon", false);
    tlm_parameters_.use_ced = configuration->property(role + ".use_reduced_ced", false);
    // make telemetry decoder object
    telemetry_decoder_ = galileo_make_telemetry_decoder_gs(satellite_, tlm_parameters_, 1);  // unified galileo decoder set to INAV (frame_type=1)
    DLOG(INFO) << "telemetry_decoder(" << telemetry_decoder_->unique_id() << ")";

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void GalileoE1BTelemetryDecoder::set_satellite(const Gnss_Satellite& satellite)
{
    satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    telemetry_decoder_->set_satellite(satellite_);
    DLOG(INFO) << "GALILEO TELEMETRY DECODER: satellite set to " << satellite_;
}


void GalileoE1BTelemetryDecoder::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void GalileoE1BTelemetryDecoder::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to disconnect
}


gr::basic_block_sptr GalileoE1BTelemetryDecoder::get_left_block()
{
    return telemetry_decoder_;
}


gr::basic_block_sptr GalileoE1BTelemetryDecoder::get_right_block()
{
    return telemetry_decoder_;
}
