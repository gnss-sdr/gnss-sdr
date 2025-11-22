/*!
 * \file gps_l2c_telemetry_decoder.h
 * \brief Interface of an adapter of a GPS L2C (CNAV) data decoder block
 * to a TelemetryDecoderInterface
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GPS_L2C_TELEMETRY_DECODER_H
#define GNSS_SDR_GPS_L2C_TELEMETRY_DECODER_H

#include "telemetry_decoder_adapter_base.h"
#include <string>


/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */


/*!
 * \brief This class implements a NAV data decoder for GPS L2 M
 */
class GpsL2CTelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    GpsL2CTelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    //! Returns "GPS_L2C_Telemetry_Decoder"
    inline std::string implementation() override
    {
        return "GPS_L2C_Telemetry_Decoder";
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GPS_L2C_TELEMETRY_DECODER_H
