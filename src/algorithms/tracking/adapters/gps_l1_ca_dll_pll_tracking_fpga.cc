/*!
 * \file gps_l1_ca_dll_pll_tracking_fpga.cc
 * \brief Implementation of an adapter of a DLL+PLL tracking loop block
 * for GPS L1 C/A to a TrackingInterface for the FPGA
 * \author Marc Majoral, 2019, mmajoral(at)cttc.es
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

#include "gps_l1_ca_dll_pll_tracking_fpga.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "display.h"
#include "gps_sdr_signal_replica.h"
#include "uio_fpga.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>
#include <array>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GpsL1CaDllPllTrackingFpga::GpsL1CaDllPllTrackingFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTrackingFpga(configuration, role, in_streams, out_streams)
{
    const uint32_t NUM_PRNs = 32;
    const int32_t GPS_CA_BIT_DURATION_MS = 20;
    // -------------------------------------------------------------------------
    // Adjust configuration parameters
    // -------------------------------------------------------------------------
    const auto vector_length = static_cast<int32_t>(
        std::round(config_params_fpga().fs_in / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));
    config_params_fpga().vector_length = vector_length;

    if (config_params_fpga().extend_correlation_symbols < 1)
        {
            config_params_fpga().extend_correlation_symbols = 1;
            std::cout << TEXT_RED
                      << "WARNING: GPS L1 C/A. extend_correlation_symbols must be >= 1. Set to 1 ms."
                      << TEXT_RESET << '\n';
        }
    else if (config_params_fpga().extend_correlation_symbols > GPS_CA_BIT_DURATION_MS)
        {
            config_params_fpga().extend_correlation_symbols = GPS_CA_BIT_DURATION_MS;
            std::cout << TEXT_RED
                      << "WARNING: GPS L1 C/A. extend_correlation_symbols limited to 20 ms."
                      << TEXT_RESET << '\n';
        }

    config_params_fpga().track_pilot = false;  // GPS L1 has no pilot

    config_params_fpga().system = 'G';
    const std::array<char, 3> sig{'1', 'C', '\0'};
    std::copy_n(sig.data(), 3, config_params_fpga().signal);

    // -------------------------------------------------------------------------
    // Configure FPGA tracking channel mapping to hardware accelerator devices
    // -------------------------------------------------------------------------
    configure_fpga_tracking_channel_mapping("1C");

    // Precompute CA codes
    ca_codes_ptr_ = static_cast<int32_t*>(
        volk_gnsssdr_malloc(GPS_L1_CA_CODE_LENGTH_CHIPS * NUM_PRNs * sizeof(int32_t),
            volk_gnsssdr_get_alignment()));

    for (uint32_t prn = 1; prn <= NUM_PRNs; ++prn)
        {
            gps_l1_ca_code_gen_int(
                own::span<int32_t>(&ca_codes_ptr_[static_cast<size_t>(GPS_L1_CA_CODE_LENGTH_CHIPS) * (prn - 1)],
                    &ca_codes_ptr_[static_cast<size_t>(GPS_L1_CA_CODE_LENGTH_CHIPS * prn)]),
                prn, 0);

            for (uint32_t k = 0; k < GPS_L1_CA_CODE_LENGTH_CHIPS; ++k)
                {
                    int32_t tmp = ca_codes_ptr_[static_cast<size_t>(GPS_L1_CA_CODE_LENGTH_CHIPS) * (prn - 1) + k];
                    if (tmp < 0)
                        {
                            tmp = 0;
                        }
                    tmp |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                    ca_codes_ptr_[static_cast<size_t>(GPS_L1_CA_CODE_LENGTH_CHIPS) * (prn - 1) + k] = tmp;
                }
        }

    config_params_fpga().ca_codes = ca_codes_ptr_;
    config_params_fpga().code_length_chips = GPS_L1_CA_CODE_LENGTH_CHIPS;
    config_params_fpga().code_samples_per_chip = 1;
    config_params_fpga().extended_correlation_in_fpga = false;
    config_params_fpga().extend_fpga_integration_periods = 1;
    config_params_fpga().fpga_integration_period = 1;

    if (config_params_fpga().extend_correlation_symbols > 1 &&
        config_params_fpga().extend_correlation_symbols <= GPS_CA_BIT_DURATION_MS &&
        (GPS_CA_BIT_DURATION_MS % config_params_fpga().extend_correlation_symbols) == 0)
        {
            config_params_fpga().extended_correlation_in_fpga = true;
            config_params_fpga().fpga_integration_period = config_params_fpga().extend_correlation_symbols;
        }

    tracking_fpga_sc_sptr_ = dll_pll_veml_make_tracking_fpga(config_params_fpga());
}


GpsL1CaDllPllTrackingFpga::~GpsL1CaDllPllTrackingFpga()
{
    volk_gnsssdr_free(ca_codes_ptr_);
}
