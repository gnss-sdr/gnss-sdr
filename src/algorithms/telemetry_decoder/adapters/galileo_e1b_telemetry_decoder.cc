/*!
 * \file galileo_e1b_telemetry_decoder.cc
 * \brief Implementation of an adapter of a Galileo INAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Javier Arribas 2013. jarribas(at)cttc.es,
 * Mara Branzanti 2013. mara.branzanti(at)gmail.com
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

#include "galileo_e1b_telemetry_decoder.h"
#include "galileo_telemetry_decoder_gs.h"

namespace
{
Tlm_Conf get_e1_tlm_conf(const ConfigurationInterface* configuration, const std::string& role)
{
    auto tlm_parameters = get_tlm_conf(configuration, role);

    if (configuration != nullptr)
        {
            tlm_parameters.enable_reed_solomon = configuration->property(role + ".enable_reed_solomon", false);
            tlm_parameters.use_ced = configuration->property(role + ".use_reduced_ced", false);
        }

    return tlm_parameters;
}

}  // namespace

GalileoE1BTelemetryDecoder::GalileoE1BTelemetryDecoder(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : TelemetryDecoderAdapterBase(
          role,
          "Galileo_E1B_Telemetry_Decoder",
          in_streams,
          out_streams,
          galileo_make_telemetry_decoder_gs(get_e1_tlm_conf(configuration, role), 1))
{
}
