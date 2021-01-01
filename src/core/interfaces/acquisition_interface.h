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

#ifndef GNSS_SDR_ACQUISITION_INTERFACE_H
#define GNSS_SDR_ACQUISITION_INTERFACE_H

#include "gnss_block_interface.h"
#include "gnss_synchro.h"
#include <memory>

/** \addtogroup Core
 * \{ */
/** \addtogroup GNSS_Block_Interfaces GNSS block interfaces
 * GNSS block interfaces.
 * \{ */


template <typename Data>
class Concurrent_Queue;

class ChannelFsm;

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
    virtual void set_channel(unsigned int channel_id) = 0;
    virtual void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) = 0;
    virtual void set_threshold(float threshold) = 0;
    virtual void set_doppler_max(unsigned int doppler_max) = 0;
    virtual void set_doppler_step(unsigned int doppler_step) = 0;
    virtual void set_doppler_center(int doppler_center __attribute__((unused)))
    {
        return;
    }
    virtual void init() = 0;
    virtual void set_local_code() = 0;
    virtual void set_state(int state) = 0;
    virtual signed int mag() = 0;
    virtual void reset() = 0;
    virtual void stop_acquisition() = 0;
    virtual void set_resampler_latency(uint32_t latency_samples) = 0;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_ACQUISITION_INTERFACE */
