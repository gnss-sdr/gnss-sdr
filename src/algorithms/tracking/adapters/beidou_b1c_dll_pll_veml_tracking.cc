/*!
 * \file beidou_b1c_dll_pll_veml_tracking.cc
 * \brief Adapts a DLL+PLL VEML tracking loop block to a TrackingInterface
 *   for BeiDou B1C signals
 * \author GNSS-SDR contributors
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

#include "beidou_b1c_dll_pll_veml_tracking.h"
#include "Beidou_B1C.h"
#include "configuration_interface.h"
#include "display.h"
#include <algorithm>
#include <array>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

BeidouB1cDllPllVemlTracking::BeidouB1cDllPllVemlTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void BeidouB1cDllPllVemlTracking::configure_tracking_parameters(
    const ConfigurationInterface* configuration [[maybe_unused]])
{
    config_params().vector_length = static_cast<int>(std::round(
        config_params().fs_in / (BEIDOU_B1C_CODE_RATE_CPS / BEIDOU_B1C_CODE_LENGTH_CHIPS)));
    config_params().system = 'C';
    const std::array<char, 3> sig{'1', 'D', '\0'};
    std::copy_n(sig.data(), 3, config_params().signal);
    const bool acq_qmboc = configuration->property("Acquisition_1D.qmboc", config_params().b1c_qmboc_tracking);
    // Keep tracking-side local replica modulation aligned with acquisition by default.
    config_params().b1c_qmboc_tracking = configuration->property(role() + ".b1c_qmboc_tracking", acq_qmboc);

    if (config_params().extend_correlation_symbols < 1)
        {
            config_params().extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: BeiDou B1C. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (10 ms)" << TEXT_RESET << '\n';
        }
    else if (!config_params().track_pilot && config_params().extend_correlation_symbols > 1)
        {
            config_params().extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: BeiDou B1C. Extended coherent integration is not allowed when tracking the data component. Coherent integration has been set to 10 ms (1 symbol)" << TEXT_RESET << '\n';
        }

    // B1C pilot: 1800-chip secondary sync spans one B-CNAV1 frame (18 s).
    // Keep coherent integration at 1 symbol (10 ms); extended correlator is not used.
    if (config_params().track_pilot)
        {
            config_params().extend_correlation_symbols = 1;
            const auto min_bit_sync_limit_s = static_cast<uint32_t>(
                BEIDOU_B1C_FRAME_PERIOD_S + 30);
            if (config_params().bit_synchronization_time_limit_s < min_bit_sync_limit_s)
                {
                    config_params().bit_synchronization_time_limit_s = min_bit_sync_limit_s;
                }
        }

    if ((config_params().extend_correlation_symbols > 1) &&
        (config_params().pll_bw_narrow_hz > config_params().pll_bw_hz ||
            config_params().dll_bw_narrow_hz > config_params().dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: BeiDou B1C. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << '\n';
        }
}


void BeidouB1cDllPllVemlTracking::create_tracking_block()
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
