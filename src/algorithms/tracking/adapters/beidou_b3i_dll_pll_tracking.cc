/*!
 * \file beidou_b3i_dll_pll_tracking.cc
 * \brief Implementation of an adapter of a DLL+PLL tracking loop block
 * for Beidou B3I to a TrackingInterface
 * \author Damian Miralles, 2019. dmiralles2009(at)gmail.com
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#include "beidou_b3i_dll_pll_tracking.h"
#include "Beidou_B3I.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <array>

using google::LogMessage;

BeidouB3iDllPllTracking::BeidouB3iDllPllTracking(
    ConfigurationInterface* configuration, const std::string& role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    Dll_Pll_Conf trk_params = Dll_Pll_Conf();
    DLOG(INFO) << "role " << role;
    trk_params.SetFromConfiguration(configuration, role);

    int vector_length = std::round(static_cast<double>(trk_params.fs_in) / (BEIDOU_B3I_CODE_RATE_CPS / BEIDOU_B3I_CODE_LENGTH_CHIPS));
    trk_params.vector_length = vector_length;
    trk_params.track_pilot = configuration->property(role + ".track_pilot", false);
    if (trk_params.extend_correlation_symbols < 1)
        {
            trk_params.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: BEIDOU B3I. extend_correlation_symbols must be bigger than 1. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
        }
    else if (trk_params.extend_correlation_symbols > 20)
        {
            trk_params.extend_correlation_symbols = 20;
            std::cout << TEXT_RED << "WARNING: BEIDOU B3I. extend_correlation_symbols must be lower than 21. Coherent integration has been set to 20 symbols (20 ms)" << TEXT_RESET << std::endl;
        }
    trk_params.system = 'C';
    std::array<char, 3> sig_{'B', '3', '\0'};
    std::memcpy(trk_params.signal, sig_.data(), 3);

    // ################# Make a GNU Radio Tracking block object ################
    if (trk_params.item_type == "gr_complex")
        {
            item_size_ = sizeof(gr_complex);
            tracking_ = dll_pll_veml_make_tracking(trk_params);
        }
    else
        {
            item_size_ = sizeof(gr_complex);
            LOG(WARNING) << trk_params.item_type << " unknown tracking item type.";
        }
    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


void BeidouB3iDllPllTracking::start_tracking()
{
    tracking_->start_tracking();
}


void BeidouB3iDllPllTracking::stop_tracking()
{
    tracking_->stop_tracking();
}


/*
 * Set tracking channel unique ID
 */
void BeidouB3iDllPllTracking::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_->set_channel(channel);
}


void BeidouB3iDllPllTracking::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_->set_gnss_synchro(p_gnss_synchro);
}


void BeidouB3iDllPllTracking::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void BeidouB3iDllPllTracking::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr BeidouB3iDllPllTracking::get_left_block()
{
    return tracking_;
}


gr::basic_block_sptr BeidouB3iDllPllTracking::get_right_block()
{
    return tracking_;
}
