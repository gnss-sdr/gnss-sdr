/*!
 * \file qzss_l5_telemetry_decoder.cc
 * \brief Interface of an adapter of a QZSS L5 NAV data decoder block
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


#include "qzss_l5_telemetry_decoder.h"
//#include "qzss_l5_telemetry_decoder_gs.h"


QzssL5TelemetryDecoder::QzssL5TelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : TelemetryDecoderAdapterBase(configuration,
          role,
          in_streams,
          out_streams)
{
    // TODO: InitializeDecoder(qzss_l5_make_telemetry_decoder_gs(satellite(), tlm_parameters_));
}