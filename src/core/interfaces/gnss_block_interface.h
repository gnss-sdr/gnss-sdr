/*!
 * \file gnss_block_interface.h
 * \brief This interface represents a GNSS block.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * Abstract class for GNSS block interfaces. Since all its methods are virtual,
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


#ifndef GNSS_SDR_GNSS_BLOCK_INTERFACE_H
#define GNSS_SDR_GNSS_BLOCK_INTERFACE_H

#include <gnuradio/top_block.h>
#include <cassert>
#include <string>


/*!
 * \brief This abstract class represents an interface to GNSS blocks.
 *
 * Abstract class for GNSS block interfaces. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 */
class GNSSBlockInterface
{
public:
    virtual ~GNSSBlockInterface() = default;
    virtual std::string role() = 0;
    virtual std::string implementation() = 0;
    virtual size_t item_size() = 0;
    virtual void connect(gr::top_block_sptr top_block) = 0;
    virtual void disconnect(gr::top_block_sptr top_block) = 0;

    virtual gr::basic_block_sptr get_left_block() = 0;
    virtual gr::basic_block_sptr get_right_block() = 0;

    virtual gr::basic_block_sptr get_left_block(int RF_channel)
    {
        assert(RF_channel >= 0);
        if (RF_channel == 0)
            {
            };           // avoid unused param warning
        return nullptr;  // added to support raw array access (non pure virtual to allow left unimplemented)= 0;
    }
    virtual gr::basic_block_sptr get_right_block(int RF_channel)
    {
        assert(RF_channel >= 0);
        if (RF_channel == 0)
            {
            };           // avoid unused param warning
        return nullptr;  // added to support raw array access (non pure virtual to allow left unimplemented)= 0;
    }
};

#endif  // GNSS_SDR_GNSS_BLOCK_INTERFACE_H
