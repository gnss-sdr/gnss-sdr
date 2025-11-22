/*!
 * \file galileo_e5b_dll_pll_tracking.cc
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E5b signals
 * \author Piyush Gupta, 2020. piyush04111999@gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas@cttc.es
 *          <li> Luis Esteve, 2012. luis@epsilon-formacion.com
 *          <li> Marc Sales, 2014. marcsales92@gmail.com
 *          </ul>
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

#include "galileo_e5b_dll_pll_tracking.h"
#include "Galileo_E5b.h"
#include "configuration_interface.h"
#include "display.h"
#include <algorithm>
#include <array>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GalileoE5bDllPllTracking::GalileoE5bDllPllTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void GalileoE5bDllPllTracking::configure_tracking_parameters(
    const ConfigurationInterface* configuration [[maybe_unused]])
{
    const auto vector_length = static_cast<int>(std::round(config_params().fs_in / (GALILEO_E5B_CODE_CHIP_RATE_CPS / GALILEO_E5B_CODE_LENGTH_CHIPS)));
    config_params().vector_length = vector_length;
    if (config_params().extend_correlation_symbols < 1)
        {
            config_params().extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E5b. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << '\n';
        }
    else if (!config_params().track_pilot and config_params().extend_correlation_symbols > GALILEO_E5B_I_SECONDARY_CODE_LENGTH)
        {
            config_params().extend_correlation_symbols = GALILEO_E5B_I_SECONDARY_CODE_LENGTH;
            std::cout << TEXT_RED << "WARNING: Galileo E5b. extend_correlation_symbols must be lower than 5 when tracking the data component. Coherent integration has been set to 4 symbols (4 ms)" << TEXT_RESET << '\n';
        }
    if ((config_params().extend_correlation_symbols > 1) and (config_params().pll_bw_narrow_hz > config_params().pll_bw_hz or config_params().dll_bw_narrow_hz > config_params().dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: Galileo E5b. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << '\n';
        }
    config_params().system = 'E';
    const std::array<char, 3> sig{'7', 'X', '\0'};
    std::copy_n(sig.data(), 3, config_params().signal);
}


void GalileoE5bDllPllTracking::create_tracking_block()
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
