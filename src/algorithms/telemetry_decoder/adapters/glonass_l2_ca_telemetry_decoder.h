/*!
 * \file glonass_l2_ca_telemetry_decoder.h
 * \brief Interface of an adapter of a GLONASS L2 C/A NAV data decoder block
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


#ifndef GNSS_SDR_GLONASS_L2_CA_TELEMETRY_DECODER_H
#define GNSS_SDR_GLONASS_L2_CA_TELEMETRY_DECODER_H

#include "telemetry_decoder_adapter_base.h"
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */


/*!
 * \brief This class implements a NAV data decoder for GLONASS L2 C/A
 */
class GlonassL2CaTelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    GlonassL2CaTelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    //! Returns "GLONASS_L2_CA_Telemetry_Decoder"
    inline std::string implementation() override
    {
        return "GLONASS_L2_CA_Telemetry_Decoder";
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GLONASS_L2_CA_TELEMETRY_DECODER_H
