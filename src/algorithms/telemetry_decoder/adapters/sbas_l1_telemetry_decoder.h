/*!
 * \file sbas_l1_telemetry_decoder.h
 * \brief Interface of an adapter of a SBAS telemetry data decoder block
 * to a TelemetryDecoderInterface
 * \author Daniel Fehr 2013. daniel.co(at)bluewin.ch
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


#ifndef GNSS_SDR_SBAS_L1_TELEMETRY_DECODER_H
#define GNSS_SDR_SBAS_L1_TELEMETRY_DECODER_H


#include "telemetry_decoder_adapter_base.h"
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */


/*!
 * \brief This class implements a NAV data decoder for SBAS frames in L1 radio link
 */
class SbasL1TelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    SbasL1TelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

private:
    std::string dump_filename_;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SBAS_L1_TELEMETRY_DECODER_H
