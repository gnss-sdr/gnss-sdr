/*!
 * \file signal_source_base.cc
 * \brief Base class for signal sources
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

#include "signal_source_base.h"
#include "configuration_interface.h"

using namespace std::string_literals;

std::string SignalSourceBase::role()
{
    return role_;
}

std::string SignalSourceBase::implementation()
{
    return implementation_;
}

size_t SignalSourceBase::getRfChannels() const
{
    return rfChannels_;
}

SignalSourceBase::SignalSourceBase(ConfigurationInterface const* configuration, std::string role, std::string impl)
    : SignalSourceInterface(), role_(role), implementation_(impl), rfChannels_(configuration->property(role + ".RF_channels"s, 1u))
{
}
