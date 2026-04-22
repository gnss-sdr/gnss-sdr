/*!
 * \file galileo_e5a_telemetry_decoder.h
 * \brief Interface of an adapter of a GALILEO E5a FNAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *         <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          </ul>
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

#ifndef GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_H
#define GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_H


#include "telemetry_decoder_adapter_base.h"
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */

/*!
 * \brief This class implements a NAV data decoder for Galileo INAV frames in E1B radio link
 */
class GalileoE5aTelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    GalileoE5aTelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E5A_TELEMETRY_DECODER_H
