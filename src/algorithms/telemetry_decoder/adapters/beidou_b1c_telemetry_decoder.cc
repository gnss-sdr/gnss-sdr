/*!
 * \file beidou_b1c_telemetry_decoder.cc
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "beidou_b1c_telemetry_decoder.h"
#include "beidou_b1c_telemetry_decoder_gs.h"

BeidouB1cTelemetryDecoder::BeidouB1cTelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : TelemetryDecoderAdapterBase(configuration, role, in_streams, out_streams)
{
    InitializeDecoder(beidou_b1c_make_telemetry_decoder_gs(satellite(), tlm_parameters_));
}
