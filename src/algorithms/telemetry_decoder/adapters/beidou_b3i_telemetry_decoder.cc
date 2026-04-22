/*!
 * \file beidou_b3i_telemetry_decoder.cc
 * \brief Implementation of an adapter of a Beidou B3I NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
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

#include "beidou_b3i_telemetry_decoder.h"
#include "beidou_b3i_telemetry_decoder_gs.h"

BeidouB3iTelemetryDecoder::BeidouB3iTelemetryDecoder(
    const ConfigurationInterface *configuration,
    const std::string &role,
    unsigned int in_streams,
    unsigned int out_streams)
    : TelemetryDecoderAdapterBase(
          role,
          "BEIDOU_B3I_Telemetry_Decoder",
          in_streams,
          out_streams,
          beidou_b3i_make_telemetry_decoder_gs(get_tlm_conf(configuration, role)))
{
}
