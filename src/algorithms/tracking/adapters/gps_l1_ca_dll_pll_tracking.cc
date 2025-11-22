/*!
 * \file gps_l1_ca_dll_pll_tracking.cc
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L1 C/A to a TrackingInterface
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
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

#include "gps_l1_ca_dll_pll_tracking.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "display.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GpsL1CaDllPllTracking::GpsL1CaDllPllTracking(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTracking(configuration, role, in_streams, out_streams)
{
    configure_tracking_parameters(configuration);
    create_tracking_block();
}


void GpsL1CaDllPllTracking::configure_tracking_parameters(
    const ConfigurationInterface* configuration __attribute__((unused)))
{
    // Set basic signal identifiers
    config_params().system = 'G';
    const std::array<char, 3> sig{'1', 'C', '\0'};
    std::copy_n(sig.data(), 3, config_params().signal);

    const auto vector_length = static_cast<int>(std::round(config_params().fs_in / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));
    config_params().vector_length = vector_length;

    // Sanity checks and warnings
    if (config_params().extend_correlation_symbols < 1)
        {
            config_params().extend_correlation_symbols = 1;
            std::cout << TEXT_RED
                      << "WARNING: GPS L1 C/A: extend_correlation_symbols must be > 0. "
                      << "Coherent integration set to 1 ms."
                      << TEXT_RESET << std::endl;
        }
    else if (config_params().extend_correlation_symbols > 20)
        {
            config_params().extend_correlation_symbols = 20;
            std::cout << TEXT_RED
                      << "WARNING: GPS L1 C/A: extend_correlation_symbols limited to 20 (20 ms)."
                      << TEXT_RESET << std::endl;
        }

    // GPS L1 C/A does not have a pilot component
    config_params().track_pilot = configuration->property(this->role() + ".track_pilot", false);
    if (config_params().track_pilot)
        {
            config_params().track_pilot = false;
            std::cout << TEXT_RED
                      << "WARNING: GPS L1 C/A does not have pilot signal. "
                      << "Data tracking enabled instead."
                      << TEXT_RESET << std::endl;
        }

    // Ensure bandwidth sanity when narrow-band is enabled
    if ((config_params().extend_correlation_symbols > 1) &&
        (config_params().pll_bw_narrow_hz > config_params().pll_bw_hz ||
            config_params().dll_bw_narrow_hz > config_params().dll_bw_hz))
        {
            std::cout << TEXT_RED
                      << "WARNING: GPS L1 C/A: Narrow tracking bandwidth is higher than wide bandwidth."
                      << TEXT_RESET << std::endl;
        }
}


void GpsL1CaDllPllTracking::create_tracking_block()
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
