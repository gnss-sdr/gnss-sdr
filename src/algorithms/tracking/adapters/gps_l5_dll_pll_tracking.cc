/*!
 * \file gps_l5_dll_pll_tracking.cc
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L5 to a TrackingInterface
 * \author Javier Arribas, 2017. jarribas(at)cttc.es
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

#include "gps_l5_dll_pll_tracking.h"
#include "GPS_L5.h"
#include "configuration_interface.h"
#include "display.h"
#include <algorithm>
#include <array>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GpsL5DllPllTracking::GpsL5DllPllTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void GpsL5DllPllTracking::configure_tracking_parameters(
    const ConfigurationInterface* configuration __attribute__((unused)))
{
    const auto vector_length = static_cast<int>(std::round(static_cast<double>(config_params().fs_in) / (static_cast<double>(GPS_L5I_CODE_RATE_CPS) / static_cast<double>(GPS_L5I_CODE_LENGTH_CHIPS))));
    config_params().vector_length = vector_length;
    if (config_params().extend_correlation_symbols < 1)
        {
            config_params().extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: GPS L5. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << '\n';
        }
    else if (!config_params().track_pilot and config_params().extend_correlation_symbols > GPS_L5I_NH_CODE_LENGTH)
        {
            config_params().extend_correlation_symbols = GPS_L5I_NH_CODE_LENGTH;
            std::cout << TEXT_RED << "WARNING: GPS L5. extend_correlation_symbols must be lower than 11 when tracking the data component. Coherent integration has been set to 10 symbols (10 ms)" << TEXT_RESET << '\n';
        }
    if ((config_params().extend_correlation_symbols > 1) and (config_params().pll_bw_narrow_hz > config_params().pll_bw_hz or config_params().dll_bw_narrow_hz > config_params().dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: GPS L5. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << '\n';
        }
    config_params().system = 'G';
    const std::array<char, 3> sig{'L', '5', '\0'};
    std::copy_n(sig.data(), 3, config_params().signal);
}


void GpsL5DllPllTracking::create_tracking_block()
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
