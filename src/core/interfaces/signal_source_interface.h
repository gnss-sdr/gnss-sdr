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
    virtual std::string role() = 0;
    virtual std::string implementation() = 0;
    virtual size_t item_size() = 0;
    virtual void connect(gr::top_block_sptr top_block) = 0;
    virtual void disconnect(gr::top_block_sptr top_block) = 0;

    virtual gr::basic_block_sptr get_left_block() = 0;
    virtual gr::basic_block_sptr get_right_block() = 0;

    virtual void fixme() = 0;
};



#endif

