/*!
 * \file beidou_b3i_telemetry_decoder.h
 * \brief Interface of an adapter of a Beidou B3I NAV data decoder block
 * to a TelemetryDecoderInterface
 * \author Damian Miralles, 2019. dmiralles2009@gmail.com
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

#ifndef GNSS_SDR_BEIDOU_B3I_TELEMETRY_DECODER_H
#define GNSS_SDR_BEIDOU_B3I_TELEMETRY_DECODER_H


#include "telemetry_decoder_adapter_base.h"
#include <string>


/** \addtogroup Telemetry_Decoder
 * \{ */
/** \addtogroup Telemetry_Decoder_adapters
 * \{ */


/*!
 * \brief This class implements a NAV data decoder for BEIDOU B1I
 */
class BeidouB3iTelemetryDecoder : public TelemetryDecoderAdapterBase
{
public:
    BeidouB3iTelemetryDecoder(
        const ConfigurationInterface *configuration,
        const std::string &role, unsigned int in_streams,
        unsigned int out_streams);
};

/** \} */
/** \} */
#endif
