/*!
 * \file galileo_e5b_telemetry_decoder.h
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


#ifndef GNSS_SDR_GALILEO_E5B_TELEMETRY_DECODER_H
#define GNSS_SDR_GALILEO_E5B_TELEMETRY_DECODER_H

#include "telemetry_decoder_adapter_base.h"
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */


/*!
 * \brief This class implements a NAV data decoder for Galileo INAV frames in E5b radio link
 */
class GalileoE5bTelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    GalileoE5bTelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    /*!
     * \brief Returns "Galileo_E5b_Telemetry_Decoder"
     */
    inline std::string implementation() override
    {
        return "Galileo_E5b_Telemetry_Decoder";
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_E5B_TELEMETRY_DECODER_H
