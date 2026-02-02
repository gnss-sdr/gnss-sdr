/*!
 * \file gps_l2_m_dll_pll_tracking_fpga.cc
 * \brief Implementation of an adapter of a DLL+PLL tracking loop block
 * for GPS L2C to a TrackingInterface for the FPGA
 * \author Javier Arribas, 2019. jarribas(at)cttc.es
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


#include "gps_l2_m_dll_pll_tracking_fpga.h"
#include "GPS_L2C.h"
#include "configuration_interface.h"
#include "display.h"
#include "gps_l2c_signal_replica.h"
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>
#include <array>
#include <cmath>  // for round
#include <iostream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GpsL2MDllPllTrackingFpga::GpsL2MDllPllTrackingFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTrackingFpga(configuration, role, in_streams, out_streams)
{
    // -------------------------------------------------------------------------
    // Adjust configuration parameters
    // -------------------------------------------------------------------------
    const auto vector_length = static_cast<int>(
        std::round(static_cast<double>(config_params_fpga().fs_in) /
                   (static_cast<double>(GPS_L2_M_CODE_RATE_CPS) /
                       static_cast<double>(GPS_L2_M_CODE_LENGTH_CHIPS))));
    config_params_fpga().vector_length = vector_length;

    config_params_fpga().extend_correlation_symbols =
        configuration->property(role + ".extend_correlation_symbols", 1);

    if (config_params_fpga().extend_correlation_symbols != 1)
        {
            config_params_fpga().extend_correlation_symbols = 1;
            std::cout << TEXT_RED
                      << "WARNING: Extended coherent integration is not supported in GPS L2. "
                      << "Coherent integration set to 20 ms (1 symbol)."
                      << TEXT_RESET << '\n';
        }

    config_params_fpga().track_pilot =
        configuration->property(role + ".track_pilot", false);
    if (config_params_fpga().track_pilot)
        {
            config_params_fpga().track_pilot = false;
            std::cout << TEXT_RED
                      << "WARNING: GPS L2M does not have a pilot signal. Data tracking enabled."
                      << TEXT_RESET << '\n';
        }

    config_params_fpga().system = 'G';
    const std::array<char, 3> sig{'2', 'S', '\0'};
    std::copy_n(sig.data(), 3, config_params_fpga().signal);

    // -------------------------------------------------------------------------
    // Configure FPGA tracking channel mapping to hardware accelerator devices
    // -------------------------------------------------------------------------
    configure_fpga_tracking_channel_mapping("2S");

    // -------------------------------------------------------------------------
    // Precompute local PRN codes
    // -------------------------------------------------------------------------
    const uint32_t NUM_PRNs = 32;
    const auto code_length_chips = static_cast<uint32_t>(GPS_L2_M_CODE_LENGTH_CHIPS);

    volk_gnsssdr::vector<float> ca_codes_f(code_length_chips, 0.0F);
    prn_codes_ptr_ = static_cast<int32_t*>(
        volk_gnsssdr_malloc(code_length_chips * NUM_PRNs * sizeof(int32_t),
            volk_gnsssdr_get_alignment()));

    for (uint32_t prn = 1; prn <= NUM_PRNs; ++prn)
        {
            gps_l2c_m_code_gen_float(ca_codes_f, prn);
            for (uint32_t s = 0; s < code_length_chips; ++s)
                {
                    int32_t bit_val = (ca_codes_f[s] < 0.0F) ? 0 : 1;
                    bit_val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                    prn_codes_ptr_[code_length_chips * (prn - 1) + s] = bit_val;
                }
        }

    // -------------------------------------------------------------------------
    // Assign FPGA tracking configuration
    // -------------------------------------------------------------------------
    config_params_fpga().ca_codes = prn_codes_ptr_;
    config_params_fpga().data_codes = nullptr;
    config_params_fpga().code_length_chips = GPS_L2_M_CODE_LENGTH_CHIPS;
    config_params_fpga().code_samples_per_chip = 1;
    config_params_fpga().extended_correlation_in_fpga = false;
    config_params_fpga().extend_fpga_integration_periods = 1;
    config_params_fpga().fpga_integration_period = 1;

    // -------------------------------------------------------------------------
    // Create GNU Radio FPGA tracking block
    // -------------------------------------------------------------------------
    tracking_fpga_sc_sptr_ = dll_pll_veml_make_tracking_fpga(config_params_fpga());
    DLOG(INFO) << "tracking(" << tracking_fpga_sc_sptr_->unique_id() << ")";
}


GpsL2MDllPllTrackingFpga::~GpsL2MDllPllTrackingFpga()
{
    volk_gnsssdr_free(prn_codes_ptr_);
}
