/*!
 * \file gps_l5_telemetry_decoder.h
 * \brief Interface of an adapter of a GPS L5 (CNAV) data decoder block
 * to a TelemetryDecoderInterface
 * \author Antonio Ramos, 2017. antonio.ramos(at)cttc.es
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


#ifndef GNSS_SDR_GPS_L5_TELEMETRY_DECODER_H
#define GNSS_SDR_GPS_L5_TELEMETRY_DECODER_H


#include "telemetry_decoder_adapter_base.h"
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */

/*!
 * \brief This class implements a NAV data decoder for GPS L5
 */
class GpsL5TelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    GpsL5TelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    //! Returns "GPS_L5_Telemetry_Decoder"
    inline std::string implementation() override
    {
        return "GPS_L5_Telemetry_Decoder";
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L5_TELEMETRY_DECODER_H
