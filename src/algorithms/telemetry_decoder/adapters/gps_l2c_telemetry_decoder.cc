/*!
 * \file gps_l2c_telemetry_decoder.cc
 * \brief Implementation of an adapter of a GPS L2C M NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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


#include "gps_l2c_telemetry_decoder.h"
#include "gps_l2c_telemetry_decoder_gs.h"

GpsL2CTelemetryDecoder::GpsL2CTelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : TelemetryDecoderAdapterBase(configuration,
          role,
          in_streams,
          out_streams)
{
    InitializeDecoder(gps_l2c_make_telemetry_decoder_gs(satellite(), tlm_parameters_));
}
