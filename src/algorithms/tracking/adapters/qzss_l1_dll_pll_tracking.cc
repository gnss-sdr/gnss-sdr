/*!
 * \file qzss_l1_dll_pll_tracking.cc
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for QZSS L1 signals to a TrackingInterface
 * \author Carles Fernandez, 2026. cfernandez(at)cttc.es
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
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "qzss_l1_dll_pll_tracking.h"
#include "configuration_interface.h"
#include "display.h"
#include "qzss.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

QzssL1DllPllTracking::QzssL1DllPllTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void QzssL1DllPllTracking::configure_tracking_parameters(
    const ConfigurationInterface* configuration __attribute__((unused)))
{
    // Set basic signal identifiers
    config_params().system = 'J';
    const std::array<char, 3> sig{'J', '1', '\0'};
    std::copy_n(sig.data(), 3, config_params().signal);

    const auto vector_length = static_cast<int>(std::round(config_params().fs_in / (QZSS_L1_CHIP_RATE / QZSS_L1_CODE_LENGTH)));
    config_params().vector_length = vector_length;

    // Sanity checks and warnings
    if (config_params().extend_correlation_symbols < 1)
        {
            config_params().extend_correlation_symbols = 1;
            std::cout << TEXT_RED
                      << "WARNING: QZSS L1 C/A: extend_correlation_symbols must be > 0. "
                      << "Coherent integration set to 1 ms."
                      << TEXT_RESET << std::endl;
        }
    else if (config_params().extend_correlation_symbols > 20)
        {
            config_params().extend_correlation_symbols = 20;
            std::cout << TEXT_RED
                      << "WARNING: QZSS L1 C/A: extend_correlation_symbols limited to 20 (20 ms)."
                      << TEXT_RESET << std::endl;
        }

    // QZSS L1 C/A does not have a pilot component
    config_params().track_pilot = configuration->property(this->role() + ".track_pilot", false);
    if (config_params().track_pilot)
        {
            config_params().track_pilot = false;
            std::cout << TEXT_RED
                      << "WARNING: QZSS L1 C/A does not have pilot signal. "
                      << "Data tracking enabled instead."
                      << TEXT_RESET << std::endl;
        }

    // Ensure bandwidth sanity when narrow-band is enabled
    if ((config_params().extend_correlation_symbols > 1) &&
        (config_params().pll_bw_narrow_hz > config_params().pll_bw_hz ||
            config_params().dll_bw_narrow_hz > config_params().dll_bw_hz))
        {
            std::cout << TEXT_RED
                      << "WARNING: QZSS L1 C/A: Narrow tracking bandwidth is higher than wide bandwidth."
                      << TEXT_RESET << std::endl;
        }
}


void QzssL1DllPllTracking::create_tracking_block()
{
    // Create GNU Radio block
    if (config_params().item_type == "gr_complex")
        {
            tracking_sptr_ = dll_pll_veml_make_tracking(config_params());
            DLOG(INFO) << "Tracking block (" << tracking_sptr_->unique_id() << ")";
        }
    else
        {
            set_item_size(0);
            tracking_sptr_ = nullptr;
            LOG(WARNING) << config_params().item_type << " unknown tracking item type.";
        }
}
