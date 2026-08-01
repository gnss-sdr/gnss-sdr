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
    /*!
     * \brief Returns the leftmost GNU Radio block of the tracking sub-chain.
     * \return Shared pointer to the leftmost tracking basic block.
     */
    virtual gr::basic_block_sptr get_left_block_trk() = 0;

    /*!
     * \brief Returns the rightmost GNU Radio block of the tracking sub-chain.
     * \return Shared pointer to the rightmost tracking basic block.
     */
    virtual gr::basic_block_sptr get_right_block_trk() = 0;

    /*!
     * \brief Returns the leftmost GNU Radio block of the acquisition sub-chain.
     * \return Shared pointer to the leftmost acquisition basic block.
     */
    virtual gr::basic_block_sptr get_left_block_acq() = 0;

    /*!
     * \brief Returns the rightmost GNU Radio block of the acquisition sub-chain.
     * \return Shared pointer to the rightmost acquisition basic block.
     */
    virtual gr::basic_block_sptr get_right_block_acq() = 0;

    /*!
     * \brief Returns the leftmost GNU Radio block of the entire channel.
     * \return Shared pointer to the leftmost basic block.
     */
    virtual gr::basic_block_sptr get_left_block() = 0;

    /*!
     * \brief Returns the rightmost GNU Radio block of the entire channel.
     * \return Shared pointer to the rightmost basic block.
     */
    virtual gr::basic_block_sptr get_right_block() = 0;

    /*!
     * \brief Returns the GNSS signal assigned to this channel.
     * \return The Gnss_Signal object for this channel.
     */
    virtual Gnss_Signal get_signal() = 0;

    /*!
     * \brief Initiates the acquisition process on this channel.
     */
    virtual void start_acquisition() = 0;

    /*!
     * \brief Provides Doppler assistance to the acquisition process.
     * \param[in] Carrier_Doppler_hz Doppler frequency in Hz.
     */
    virtual void assist_acquisition_doppler(double Carrier_Doppler_hz) = 0;

    /*!
     * \brief Stops the channel operation.
     */
    virtual void stop_channel() = 0;

    /*!
     * \brief Assigns a GNSS signal to this channel.
     * \param[in] gnss_signal Reference to the Gnss_Signal to assign.
     */
    virtual void set_signal(const Gnss_Signal& gnss_signal) = 0;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_CHANNEL_INTERFACE_H
