/*!
 * \file galileo_e1b_telemetry_decoder.h
 * \brief Interface of an adapter of a GALILEO E1B NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Javier Arribas 2013 jarribas(at)cttc.es,
 *  Mara Branzanti 2013. mara.branzanti(at)gmail.com
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


#ifndef GNSS_SDR_GALILEO_E1B_TELEMETRY_DECODER_H
#define GNSS_SDR_GALILEO_E1B_TELEMETRY_DECODER_H


#include "telemetry_decoder_adapter_base.h"
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */


/*!
 * \brief This class implements a NAV data decoder for Galileo INAV frames in E1B radio link
 */
class GalileoE1BTelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    GalileoE1BTelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Returns "Galileo_E1B_Telemetry_Decoder"
     */
    inline std::string implementation() override
    {
        return "Galileo_E1B_Telemetry_Decoder";
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E1B_TELEMETRY_DECODER_H
