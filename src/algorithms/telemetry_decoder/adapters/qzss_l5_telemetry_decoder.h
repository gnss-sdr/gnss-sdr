/*!
 * \file qzss_l5_telemetry_decoder.h
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


#ifndef GNSS_SDR_QZSS_L5_TELEMETRY_DECODER_H
#define GNSS_SDR_QZSS_L5_TELEMETRY_DECODER_H

#include "telemetry_decoder_adapter_base.h"
#include <string>

/** \addtogroup Telemetry_Decoder Telemetry Decoder
 * Classes for the decoding of GNSS Navigation messages.
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters telemetry_decoder_adapters
 * Wrap GNU Radio blocks for the decoding of GNSS Navigation messages with a
 * TelemetryDecoderInterface
 * \{ */


/*!
 * \brief This class implements a NAV data decoder for QZSS L5
 */
class QzssL5TelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    QzssL5TelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_QZSS_L5_TELEMETRY_DECODER_H