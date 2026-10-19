/*!
 * \signal_source_interface.h
 * \brief Header file of the interface to a signal_source GNSS block.
 * \author Jim Melton, 2020. jim.melton(at)sncorp.com
 *
 * This header file contains the interface to an abstract class for
 * signal sources. Since all its methods are virtual, this class
 * cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have
 * been implemented by that class or a parent class.
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_SIGNAL_SOURCE_INTERFACE_H
#define GNSS_SDR_SIGNAL_SOURCE_INTERFACE_H

#include "gnss_block_interface.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

/** \addtogroup Core
 * \{ */
/** \addtogroup GNSS_Block_Interfaces GNSS block interfaces
 * GNSS block interfaces.
 * \{ */

/*! \brief This abstract class represents an interface to signal_source GNSS block.
 *
 * Abstract class for signal sources. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 */

class SignalSourceInterface : public GNSSBlockInterface
{
public:
    virtual size_t getRfChannels() const = 0;

    /*!
     * \brief True if samples arrive at the pace of a clock, as from a radio
     * front end or a throttled replay. False if the receiver sets the pace, as
     * in an unthrottled replay of a recording.
     */
    virtual bool is_real_time() const { return true; }

protected:
    SignalSourceInterface()
    {
        VLOG(1) << "SignalSourceInterface: " << this << " ctor";
    }

public:  // required for polymorphic destruction
    ~SignalSourceInterface()
    {
        VLOG(1) << "SignalSourceInterface: " << this << " dtor";
    }
};


#endif
