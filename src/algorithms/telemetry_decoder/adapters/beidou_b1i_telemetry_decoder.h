/*!
 * \file beidou_b1i_telemetry_decoder.h
 * \brief Interface of an adapter of a Beidou B1I NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Damian Miralles, 2018. dmiralles2009@gmail.com
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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


#ifndef GNSS_SDR_BEIDOU_B1I_TELEMETRY_DECODER_H
#define GNSS_SDR_BEIDOU_B1I_TELEMETRY_DECODER_H


#include "telemetry_decoder_adapter_base.h"
#include <string>

/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */


/*!
 * \brief This class implements a NAV data decoder for BEIDOU B1I
 */
class BeidouB1iTelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    BeidouB1iTelemetryDecoder(
        const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);
};


/** \} */
/** \} */
#endif
