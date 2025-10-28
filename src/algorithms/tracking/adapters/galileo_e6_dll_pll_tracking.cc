/*!
 * \file galileo_e6_dll_pll_tracking.cc
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E6 signals
 * \author Carles Fernandez-Prades, 2020. cfernandez(at)cttc.es
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

#include "galileo_e6_dll_pll_tracking.h"
#include "Galileo_E6.h"
#include "configuration_interface.h"
#include "display.h"
#include <algorithm>
#include <array>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


GalileoE6DllPllTracking::GalileoE6DllPllTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void GalileoE6DllPllTracking::configure_tracking_parameters(
    const ConfigurationInterface* configuration [[maybe_unused]])
{
    const auto vector_length = static_cast<int>(std::round(config_params().fs_in / (GALILEO_E6_B_CODE_CHIP_RATE_CPS / GALILEO_E6_B_CODE_LENGTH_CHIPS)));
    config_params().vector_length = vector_length;
    config_params().system = 'E';
    const std::array<char, 3> sig{'5', 'X', '\0'};
    std::copy_n(sig.data(), 3, config_params().signal);
    if (config_params().extend_correlation_symbols < 1)
        {
            config_params().extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E6. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << '\n';
        }
    else if (!config_params().track_pilot and config_params().extend_correlation_symbols > 1)
        {
            config_params().extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E6. Extended coherent integration is not allowed when tracking the data component. Coherent integration has been set to 1 ms (1 symbol)" << TEXT_RESET << '\n';
        }
    if ((config_params().extend_correlation_symbols > 1) and (config_params().pll_bw_narrow_hz > config_params().pll_bw_hz or config_params().dll_bw_narrow_hz > config_params().dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: Galileo E5b. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << '\n';
        }
}


void GalileoE6DllPllTracking::create_tracking_block()
{
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
