/*!
 * \file glonass_l1_ca_telemetry_decoder.h
 * \brief Interface of an adapter of a GLONASS L1 C/A NAV data decoder block
 * to a TelemetryDecoderInterface
 * \note Code added as part of GSoC 2017 program
 * \author Damian Miralles, 2017. dmiralles2009(at)gmail.com
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


#ifndef GNSS_SDR_GLONASS_L1_CA_TELEMETRY_DECODER_H
#define GNSS_SDR_GLONASS_L1_CA_TELEMETRY_DECODER_H

#include "telemetry_decoder_adapter_base.h"
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */


/*!
 * \brief This class implements a NAV data decoder for GLONASS L1 C/A
 */
class GlonassL1CaTelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    GlonassL1CaTelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);
};


/** \} */
/** \} */
#endif
