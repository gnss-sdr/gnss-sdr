/*!
 * \file gps_l2_m_dll_pll_tracking.cc
 * \brief Implementation of an adapter of a DLL+PLL tracking loop block
 * for GPS L1 C/A to a TrackingInterface
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
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

#include "gps_l2_m_dll_pll_tracking.h"
#include "GPS_L2C.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <algorithm>
#include <array>

GpsL2MDllPllTracking::GpsL2MDllPllTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(role),
      item_size_(sizeof(gr_complex)),
      channel_(0),
      in_streams_(in_streams),
      out_streams_(out_streams)
{
    Dll_Pll_Conf trk_params = Dll_Pll_Conf();
    trk_params.SetFromConfiguration(configuration, role_);

    const auto vector_length = static_cast<int>(std::round(static_cast<double>(trk_params.fs_in) / (static_cast<double>(GPS_L2_M_CODE_RATE_CPS) / static_cast<double>(GPS_L2_M_CODE_LENGTH_CHIPS))));
    trk_params.vector_length = vector_length;
    if (trk_params.extend_correlation_symbols != 1)
        {
            trk_params.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Extended coherent integration is not allowed in GPS L2. Coherent integration has been set to 20 ms (1 symbol)" << TEXT_RESET << '\n';
        }
    trk_params.track_pilot = configuration->property(role_ + ".track_pilot", false);
    if (trk_params.track_pilot)
        {
            trk_params.track_pilot = false;
            std::cout << TEXT_RED << "WARNING: GPS L2 does not have pilot signal. Data tracking has been enabled" << TEXT_RESET << '\n';
        }
    trk_params.system = 'G';
    const std::array<char, 3> sig{'2', 'S', '\0'};
    std::copy_n(sig.data(), 3, trk_params.signal);

    // ################# Make a GNU Radio Tracking block object ################
    DLOG(INFO) << "role " << role_;
    if (trk_params.item_type == "gr_complex")
        {
            tracking_sptr_ = dll_pll_veml_make_tracking(trk_params);
            DLOG(INFO) << "tracking(" << tracking_sptr_->unique_id() << ")";
        }
    else
        {
            item_size_ = 0;
            tracking_sptr_ = nullptr;
            LOG(WARNING) << trk_params.item_type << " unknown tracking item type.";
        }

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void GpsL2MDllPllTracking::stop_tracking()
{
    tracking_sptr_->stop_tracking();
}


void GpsL2MDllPllTracking::start_tracking()
{
    tracking_sptr_->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GpsL2MDllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_sptr_->set_channel(channel);
}


void GpsL2MDllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_sptr_->set_gnss_synchro(p_gnss_synchro);
}


void GpsL2MDllPllTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void GpsL2MDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GpsL2MDllPllTracking::get_left_block()
{
    return tracking_sptr_;
}


gr::basic_block_sptr GpsL2MDllPllTracking::get_right_block()
{
    return tracking_sptr_;
}
