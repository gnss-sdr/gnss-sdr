/*!
 * \file signal_source_base.h
 * \brief Header file of the base class to signal_source GNSS blocks.
 * \author Jim Melton, 2020. jim.melton(at)sncorp.com
 *
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

#ifndef GNSS_SDR_SIGNAL_SOURCE_BASE_H
#define GNSS_SDR_SIGNAL_SOURCE_BASE_H

#include "signal_source_interface.h"

#include <cstddef>
#include <string>


class ConfigurationInterface;

class SignalSourceBase : public SignalSourceInterface
{
public:
    std::string role() final;
    std::string implementation() final;

    size_t getRfChannels() const override;

protected:
    //! Constructor
    SignalSourceBase(ConfigurationInterface const* configuration, std::string role, std::string impl);

private:
    std::string const role_;
    std::string const implementation_;
    bool connected_;
    size_t rfChannels_;
};


#endif
