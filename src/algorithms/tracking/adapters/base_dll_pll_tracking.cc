/*!
 * \file base_dll_pll_tracking.cc
 * \brief Base class providing shared logic for DLL+PLL VEML tracking adapters.
 * \authors Carles Fernandez, 2025. carles.fernandez(at)cttc.cat
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

#include "base_dll_pll_tracking.h"
#include "configuration_interface.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

BaseDllPllTracking::BaseDllPllTracking(
    const ConfigurationInterface* configuration,
    std::string role,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(std::move(role)),
      item_size_(sizeof(gr_complex)),
      channel_(0),
      in_streams_(in_streams),
      out_streams_(out_streams)
{
    trk_params_ = Dll_Pll_Conf();
    trk_params_.SetFromConfiguration(configuration, role_);

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "Only one input stream is supported.";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "Only one output stream is supported.";
        }

    DLOG(INFO) << "role " << role_;
}


void BaseDllPllTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* no connection needed */
        }
}


void BaseDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* no disconnection needed */
        }
}


gr::basic_block_sptr BaseDllPllTracking::get_left_block()
{
    return tracking_sptr_;
}


gr::basic_block_sptr BaseDllPllTracking::get_right_block()
{
    return tracking_sptr_;
}


void BaseDllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_sptr_->set_channel(channel);
}


void BaseDllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_sptr_->set_gnss_synchro(p_gnss_synchro);
}


void BaseDllPllTracking::start_tracking()
{
    tracking_sptr_->start_tracking();
}


void BaseDllPllTracking::stop_tracking()
{
    tracking_sptr_->stop_tracking();
}
