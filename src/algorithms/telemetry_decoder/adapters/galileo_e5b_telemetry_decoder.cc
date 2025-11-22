/*!
 * \file galileo_e5b_telemetry_decoder.cc
 * \brief Interface of an adapter of a GALILEO E5B NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Piyush Gupta 2020 piyush04111999@gmail.com.
 * \note Code added as part of GSoC 2020 Program.
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

#include "galileo_e5b_telemetry_decoder.h"
#include "galileo_telemetry_decoder_gs.h"

GalileoE5bTelemetryDecoder::GalileoE5bTelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : TelemetryDecoderAdapterBase(configuration,
          role,
          in_streams,
          out_streams)
{
    InitializeDecoder(galileo_make_telemetry_decoder_gs(satellite(), tlm_parameters_, 1));
}
