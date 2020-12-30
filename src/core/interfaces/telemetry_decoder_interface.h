/*!
 * \file telemetry_decoder_interface.h
 * \brief This class represents an interface to a telemetry decoder block.
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Abstract class for telemetry decoders. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
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


#ifndef GNSS_SDR_TELEMETRY_DECODER_INTERFACE_H
#define GNSS_SDR_TELEMETRY_DECODER_INTERFACE_H

#include "gnss_block_interface.h"
#include "gnss_satellite.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup GNSS_Block_Interfaces
 * \{ */


/*!
 * \brief This abstract class represents an interface to a navigation GNSS block.
 *
 * Abstract class for navigation interfaces. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 */
class TelemetryDecoderInterface : public GNSSBlockInterface
{
public:
    virtual void reset() = 0;
    virtual void set_satellite(const Gnss_Satellite& sat) = 0;
    virtual void set_channel(int channel) = 0;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_TELEMETRY_DECODER_INTERFACE_H
