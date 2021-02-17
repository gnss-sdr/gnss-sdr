/*!
 * \file signal_source_base.cc
 * \brief Base class for signal sources
 * \author Jim Melton, 2020. jim.melton(at)sncorp.com
 *
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

#include "signal_source_base.h"
#include "configuration_interface.h"
#include "gnss_sdr_string_literals.h"
#include <utility>  // move

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
    : role_(std::move(role)), implementation_(std::move(impl)), rfChannels_(configuration->property(role_ + ".RF_channels"s, 1U))
{
}
