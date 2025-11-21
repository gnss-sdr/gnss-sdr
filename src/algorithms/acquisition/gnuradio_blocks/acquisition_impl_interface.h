/*!
 * \file acquisition_impl_interface.h
 * \brief Header file of the interface to an acquisition implementation GNSS block.
 * \author Mathieu Favraeu, 2025. favreau.mathieu(at)hotmail.com
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
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ACQUISITION_IMPL_INTERFACE_H
#define GNSS_SDR_ACQUISITION_IMPL_INTERFACE_H

#include "gnss_block_interface.h"
#include "gnss_synchro.h"
#include <gnuradio/block.h>
#include <complex>
#include <memory>

/** \addtogroup Core
 * \{ */
/** \addtogroup GNSS_Block_Interfaces GNSS block interfaces
 * GNSS block interfaces.
 * \{ */

class ChannelFsm;
class acquisition_impl_interface;

using acquisition_impl_interface_sptr = gnss_shared_ptr<acquisition_impl_interface>;

/*! \brief This abstract class represents an interface to an acquisition GNSS block.
 *
 * Abstract class for acquisition algorithms. Since all its methods are virtual,
 * this class cannot be instantiated directly, and a subclass can only be
 * instantiated directly if all inherited pure virtual methods have been
 * implemented by that class or a parent class.
 */
class acquisition_impl_interface : public gr::block
{
public:
    acquisition_impl_interface(const std::string& name,
        gr::io_signature::sptr input_signature,
        gr::io_signature::sptr output_signature) : gr::block(name, std::move(input_signature), std::move(output_signature)) {}

    virtual void set_gnss_synchro(Gnss_Synchro* gnss_synchro) = 0;
    virtual void set_channel(uint32_t channel_id) = 0;
    virtual void set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm) = 0;
    virtual void set_threshold(float threshold) = 0;
    virtual void set_local_code(std::complex<float>* /*code*/) {};
    virtual void set_local_code(std::complex<float>* /*code_data*/, std::complex<float>* /*code_pilot*/) {};
    virtual uint32_t mag() const = 0;
    virtual void set_active(bool active) = 0;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_ACQUISITION_INTERFACE */
