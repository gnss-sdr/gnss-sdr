/*!
 * \file qzss_l1_telemetry_decoder.cc
 * \brief Interface of an adapter of a QZSS L1 NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "qzss_l1_telemetry_decoder.h"
#include "gps_l1_ca_telemetry_decoder_gs.h"


QzssL1TelemetryDecoder::QzssL1TelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : TelemetryDecoderAdapterBase(configuration,
          role,
          in_streams,
          out_streams)
{
    InitializeDecoder(gps_l1_ca_make_telemetry_decoder_gs(satellite(), tlm_parameters_, L1LnavSystem::QZSS));
}