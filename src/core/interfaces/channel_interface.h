/*!
 * \file channel_interface.h
 * \brief This class represents an interface to a channel GNSS block.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * Abstract class for channel blocks. Since all its methods are virtual,
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

#ifndef GNSS_SDR_CHANNEL_INTERFACE_H
#define GNSS_SDR_CHANNEL_INTERFACE_H

#include "gnss_block_interface.h"
#include "gnss_signal.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup GNSS_Block_Interfaces
 * \{ */


/*!
 * \brief This abstract class represents an interface to a channel GNSS block.
 *
 * Abstract class for channel blocks. Since all its methods are pure virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 */
class ChannelInterface : public GNSSBlockInterface
{
public:
    virtual gr::basic_block_sptr get_left_block_trk() = 0;
    virtual gr::basic_block_sptr get_right_block_trk() = 0;
    virtual gr::basic_block_sptr get_left_block_acq() = 0;
    virtual gr::basic_block_sptr get_right_block_acq() = 0;
    virtual gr::basic_block_sptr get_left_block() = 0;
    virtual gr::basic_block_sptr get_right_block() = 0;
    virtual Gnss_Signal get_signal() = 0;
    virtual void start_acquisition() = 0;
    virtual void assist_acquisition_doppler(double Carrier_Doppler_hz) = 0;
    virtual void stop_channel() = 0;
    virtual void set_signal(const Gnss_Signal&) = 0;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CHANNEL_INTERFACE_H
