/*!
 * \file gps_l2_m_dll_pll_tracking.cc
 * \brief Implementation of an adapter of a DLL+PLL tracking loop block
 * for GPS L2C(M) to a TrackingInterface
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
#include <algorithm>
#include <array>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GpsL2MDllPllTracking::GpsL2MDllPllTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void GpsL2MDllPllTracking::configure_tracking_parameters(
    const ConfigurationInterface* configuration)
{
    const auto vector_length = static_cast<int>(std::round(static_cast<double>(config_params().fs_in) / (static_cast<double>(GPS_L2_M_CODE_RATE_CPS) / static_cast<double>(GPS_L2_M_CODE_LENGTH_CHIPS))));
    config_params().vector_length = vector_length;
    if (config_params().extend_correlation_symbols != 1)
        {
            config_params().extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Extended coherent integration is not allowed in GPS L2. Coherent integration has been set to 20 ms (1 symbol)" << TEXT_RESET << '\n';
        }
    config_params().track_pilot = configuration->property(role() + ".track_pilot", false);
    if (config_params().track_pilot)
        {
            config_params().track_pilot = false;
            std::cout << TEXT_RED << "WARNING: GPS L2 does not have pilot signal. Data tracking has been enabled" << TEXT_RESET << '\n';
        }
    config_params().system = 'G';
    const std::array<char, 3> sig{'2', 'S', '\0'};
    std::copy_n(sig.data(), 3, config_params().signal);
}


void GpsL2MDllPllTracking::create_tracking_block()
{
    // ################# Make a GNU Radio Tracking block object ################
    if (config_params().item_type == "gr_complex")
        {
            tracking_sptr_ = dll_pll_veml_make_tracking(config_params());
            DLOG(INFO) << "tracking(" << tracking_sptr_->unique_id() << ")";
        }
    else
        {
            set_item_size(0);
            tracking_sptr_ = nullptr;
            LOG(WARNING) << config_params().item_type << " unknown tracking item type.";
        }
}
