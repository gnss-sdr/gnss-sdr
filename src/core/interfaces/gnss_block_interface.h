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
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_GNSS_BLOCK_INTERFACE_H_
#define GNSS_SDR_GNSS_BLOCK_INTERFACE_H_

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
    virtual ~GNSSBlockInterface() {}
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

#endif /*GNSS_SDR_GNSS_BLOCK_INTERFACE_H_*/
