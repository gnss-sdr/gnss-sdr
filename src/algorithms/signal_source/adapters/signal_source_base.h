/*!
 * \file signal_source_base.h
 * \brief Header file of the base class to signal_source GNSS blocks.
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
    gr::basic_block_sptr get_left_block() override;  // non-sensical; implement once

protected:
    //! Constructor
    SignalSourceBase(ConfigurationInterface const* configuration, std::string role, std::string impl);

    //! utility for decoding passed ".item_type" values
    //!  @param[in]  item_type - user provided string, should be one of the known types
    //!  @param[out] is_interleaved - if non-null, the pointed to memory is updated with
    //!                               whether the data is interleaved I/Q (e.g., ishort)
    //!  @param[in] throw_on_error  - if true, throw an exception if the string does not
    //!                               represent a known type
    //!  @return the size in bytes of the passed type
    size_t decode_item_type(std::string const& item_type, bool* is_interleaved = nullptr, bool throw_on_error = false);

private:
    std::string const role_;
    std::string const implementation_;
    size_t rfChannels_;
};


#endif
