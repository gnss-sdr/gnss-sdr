/*!
 * \file tracking_interface.h
 * \brief This class represents an interface to a tracking block.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * Abstract class for tracking interfaces. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_TRACKING_INTERFACE_H_
#define GNSS_SDR_TRACKING_INTERFACE_H_

#include "gnss_block_interface.h"
#include "gnss_synchro.h"

template <typename Data>
class Concurrent_Queue;

/*!
 * \brief This abstract class represents an interface to a tracking block.
 *
 * Abstract class for tracking interfaces. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 *
 */
class TrackingInterface : public GNSSBlockInterface
{
public:
    virtual void start_tracking() = 0;
    virtual void stop_tracking() = 0;
    virtual void set_gnss_synchro(Gnss_Synchro* gnss_synchro) = 0;
    virtual void set_channel(unsigned int channel) = 0;
};

#endif  // GNSS_SDR_TRACKING_INTERFACE_H_
