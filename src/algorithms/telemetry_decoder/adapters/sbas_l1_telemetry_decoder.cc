/*!
 * \file sbas_l1_telemetry_decoder.cc
 * \brief Implementation of an adapter of a SBAS telemetry data decoder block
 * to a TelemetryDecoderInterface
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
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

#include "sbas_l1_telemetry_decoder.h"
#include "sbas_l1_telemetry_decoder_gs.h"

SbasL1TelemetryDecoder::SbasL1TelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams) : TelemetryDecoderAdapterBase(role,
                                    "SBAS_L1_Telemetry_Decoder",
                                    in_streams,
                                    out_streams,
                                    sbas_l1_make_telemetry_decoder_gs(configuration != nullptr ? configuration->property(role + ".dump", false) : false)),
                                dump_filename_(configuration != nullptr ? configuration->property(role + ".dump_filename", std::string("./navigation.dat")) : std::string("./navigation.dat"))
{
}
