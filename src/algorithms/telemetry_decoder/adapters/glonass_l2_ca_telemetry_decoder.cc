/*!
 * \file glonass_l2_ca_telemetry_decoder.cc
 * \brief Implementation of an adapter of a GLONASS L2 C/A NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Damian Miralles, 2018. dmiralles2009(at)gmail.com
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

#include "glonass_l2_ca_telemetry_decoder.h"
#include "glonass_l2_ca_telemetry_decoder_gs.h"

GlonassL2CaTelemetryDecoder::GlonassL2CaTelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : TelemetryDecoderAdapterBase(
          role,
          "GLONASS_L2_CA_Telemetry_Decoder",
          in_streams,
          out_streams,
          glonass_l2_ca_make_telemetry_decoder_gs(get_tlm_conf(configuration, role)))
{
}
