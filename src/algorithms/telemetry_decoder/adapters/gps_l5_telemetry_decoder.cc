/*!
 * \file gps_l5_telemetry_decoder.cc
 * \brief Implementation of an adapter of a GPS L5 NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
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


#include "gps_l5_telemetry_decoder.h"
#include "gps_l5_telemetry_decoder_gs.h"

GpsL5TelemetryDecoder::GpsL5TelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : TelemetryDecoderAdapterBase(configuration,
          role,
          in_streams,
          out_streams)
{
    InitializeDecoder(gps_l5_make_telemetry_decoder_gs(satellite(), tlm_parameters_));
}
