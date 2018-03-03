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
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CHANNEL_INTERFACE_H_
#define GNSS_SDR_CHANNEL_INTERFACE_H_

#include "gnss_block_interface.h"
#include "gnss_signal.h"

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
    virtual Gnss_Signal get_signal() const = 0;
    virtual void start_acquisition() = 0;
    virtual void set_signal(const Gnss_Signal&) = 0;
};

#endif /* GNSS_SDR_CHANNEL_INTERFACE_H_ */
