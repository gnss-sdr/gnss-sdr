/*!
 * \file galileo_e6_telemetry_decoder.h
 * \brief Interface of an adapter of a GALILEO E6 CNAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Carles Fernandez-Prades, 2020 cfernandez@cttc.es
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


#ifndef GNSS_SDR_GALILEO_E6_TELEMETRY_DECODER_H
#define GNSS_SDR_GALILEO_E6_TELEMETRY_DECODER_H

#include "telemetry_decoder_adapter_base.h"
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */


/*!
 * \brief This class implements a NAV data decoder for Galileo CNAV frames in E6 radio link
 */
class GalileoE6TelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    GalileoE6TelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E6_TELEMETRY_DECODER_H
