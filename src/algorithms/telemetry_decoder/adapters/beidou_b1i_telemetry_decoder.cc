/*!
 * \file beidou_b1i_telemetry_decoder.cc
 * \brief Implementation of an adapter of a Beidou B1I NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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


#include "beidou_b1i_telemetry_decoder.h"
#include "beidou_b1i_telemetry_decoder_gs.h"

BeidouB1iTelemetryDecoder::BeidouB1iTelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : TelemetryDecoderAdapterBase(configuration,
          role,
          in_streams,
          out_streams)
{
    InitializeDecoder(beidou_b1i_make_telemetry_decoder_gs(satellite(), tlm_parameters_));
}
