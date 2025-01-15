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
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
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

gr::basic_block_sptr SignalSourceBase::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return {};
}

SignalSourceBase::SignalSourceBase(ConfigurationInterface const* configuration, std::string role, std::string impl)
    : role_(std::move(role)), implementation_(std::move(impl))
{
    // because clang-tidy insists on using std::move for the string arguments, and to avoid
    // depending on the order of initialization, assign rfChannels_ in the body
    rfChannels_ = configuration->property(role_ + ".RF_channels"s, uint64_t(1U));
}

size_t SignalSourceBase::decode_item_type(std::string const& item_type, bool* is_interleaved, bool throw_on_error)
{
    size_t item_size = 0;

    // The default is for samples not to be interleaved
    if (is_interleaved) *is_interleaved = false;  // NOLINT

    if (item_type == "gr_complex"s)
        {
            item_size = sizeof(gr_complex);
        }
    else if (item_type == "float"s)
        {
            item_size = sizeof(float);
        }
    else if (item_type == "short"s)
        {
            item_size = sizeof(int16_t);
        }
    else if (item_type == "ishort"s)
        {
            item_size = sizeof(int16_t);
            if (is_interleaved) *is_interleaved = true;  // NOLINT
        }
    else if (item_type == "byte"s)
        {
            item_size = sizeof(int8_t);
        }
    else if (item_type == "ibyte"s)
        {
            item_size = sizeof(int8_t);
            if (is_interleaved) *is_interleaved = true;  // NOLINT
        }
    else
        {
            if (throw_on_error)
                {
                    throw std::invalid_argument(item_type + " is not a recognized item type"s);
                }
            else
                {
                    LOG(WARNING) << item_type
                                 << " unrecognized item type. Using gr_complex.";
                    item_size = sizeof(gr_complex);
                }
        }
    return item_size;
}
