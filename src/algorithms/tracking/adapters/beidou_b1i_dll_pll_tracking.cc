/*!
 * \file beidou_b1i_dll_pll_tracking.cc
 * \brief Implementation of an adapter of a DLL+PLL tracking loop block
 * for Beidou B1I to a TrackingInterface
 * \author Sergi Segura, 2018. sergi.segura.munoz@gmail.com
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

#include "beidou_b1i_dll_pll_tracking.h"
#include "Beidou_B1I.h"
#include "configuration_interface.h"
#include "display.h"
#include <algorithm>
#include <array>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

BeidouB1iDllPllTracking::BeidouB1iDllPllTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void BeidouB1iDllPllTracking::configure_tracking_parameters(
    const ConfigurationInterface* configuration)
{
    const auto vector_length = static_cast<int>(std::round(config_params().fs_in / (BEIDOU_B1I_CODE_RATE_CPS / BEIDOU_B1I_CODE_LENGTH_CHIPS)));
    config_params().vector_length = vector_length;
    config_params().system = 'C';
    const std::array<char, 3> sig{'B', '1', '\0'};
    std::copy_n(sig.data(), 3, config_params().signal);
    if (config_params().extend_correlation_symbols < 1)
        {
            config_params().extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: BEIDOU B1I. extend_correlation_symbols must be bigger than 1. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << '\n';
        }
    else if (config_params().extend_correlation_symbols > 20)
        {
            config_params().extend_correlation_symbols = 20;
            std::cout << TEXT_RED << "WARNING: BEIDOU B1I. extend_correlation_symbols must be lower than 21. Coherent integration has been set to 20 symbols (20 ms)" << TEXT_RESET << '\n';
        }
    config_params().track_pilot = configuration->property(role() + ".track_pilot", false);
    if (config_params().track_pilot)
        {
            std::cout << TEXT_RED << "WARNING: BEIDOU B1I does not have pilot signal. Data tracking has been enabled" << TEXT_RESET << '\n';
            config_params().track_pilot = false;
        }
    if ((config_params().extend_correlation_symbols > 1) && (config_params().pll_bw_narrow_hz > config_params().pll_bw_hz or config_params().dll_bw_narrow_hz > config_params().dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: BEIDOU B1I. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << '\n';
        }
}


void BeidouB1iDllPllTracking::create_tracking_block()
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
