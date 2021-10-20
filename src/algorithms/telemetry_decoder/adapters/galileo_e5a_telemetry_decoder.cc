/*!
 * \file galileo_e5a_telemetry_decoder.h
 * \brief Interface of an adapter of a GALILEO E5a FNAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *         <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          </ul>
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

#include "galileo_e5a_telemetry_decoder.h"
#include "configuration_interface.h"
#include <glog/logging.h>


GalileoE5aTelemetryDecoder::GalileoE5aTelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : role_(role),
                                in_streams_(in_streams),
                                out_streams_(out_streams)
{
    DLOG(INFO) << "role " << role;
    tlm_parameters_.SetFromConfiguration(configuration, role);
    // make telemetry decoder object
    telemetry_decoder_ = galileo_make_telemetry_decoder_gs(satellite_, tlm_parameters_, 2);  // unified galileo decoder set to FNAV (frame_type=2)
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


void GalileoE5aTelemetryDecoder::set_satellite(const Gnss_Satellite& satellite)
{
    satellite_ = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    telemetry_decoder_->set_satellite(satellite_);
    DLOG(INFO) << "TELEMETRY DECODER: satellite set to " << satellite_;
}


void GalileoE5aTelemetryDecoder::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // Nothing to connect internally
    DLOG(INFO) << "nothing to connect internally";
}


void GalileoE5aTelemetryDecoder::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
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
