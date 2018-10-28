/*!
 * \file acquisition_interface.h
 * \brief Header file of the interface to an acquisition GNSS block.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Luis Esteve, 2011. luis(at)epsilon-formacion.com
 *
 * This header file contains the interface to an abstract class
 * for acquisition algorithms. Since all its methods are virtual,
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

#ifndef GNSS_SDR_ACQUISITION_INTERFACE_H_
#define GNSS_SDR_ACQUISITION_INTERFACE_H_

#include "gnss_block_interface.h"
#include "gnss_synchro.h"

template <typename Data>
class concurrent_queue;

/*! \brief This abstract class represents an interface to an acquisition GNSS block.
 *
 * Abstract class for acquisition algorithms. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 */
class AcquisitionInterface : public GNSSBlockInterface
{
public:
    virtual void set_gnss_synchro(Gnss_Synchro* gnss_synchro) = 0;
    virtual void set_channel(unsigned int channel) = 0;
    virtual void set_threshold(float threshold) = 0;
    virtual void set_doppler_max(unsigned int doppler_max) = 0;
    virtual void set_doppler_step(unsigned int doppler_step) = 0;
    virtual void init() = 0;
    virtual void set_local_code() = 0;
    virtual void set_state(int state) = 0;
    virtual signed int mag() = 0;
    virtual void reset() = 0;
    virtual void stop_acquisition() = 0;
};

#endif /* GNSS_SDR_ACQUISITION_INTERFACE */
