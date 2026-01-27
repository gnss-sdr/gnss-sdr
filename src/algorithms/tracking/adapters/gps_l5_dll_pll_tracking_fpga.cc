/*!
 * \file gps_l5_dll_pll_tracking_fpga.cc
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L5 to a TrackingInterface for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
 *         Javier Arribas, 2019. jarribas(at)cttc.es
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


#include "gps_l5_dll_pll_tracking_fpga.h"
#include "GPS_L5.h"
#include "configuration_interface.h"
#include "display.h"
#include "gps_l5_signal_replica.h"
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>
#include <array>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


GpsL5DllPllTrackingFpga::GpsL5DllPllTrackingFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTrackingFpga(configuration, role, in_streams, out_streams),
      data_codes_ptr_(nullptr),
      track_pilot_(false)
{
    // -------------------------------------------------------------------------
    // Adjust configuration parameters
    // -------------------------------------------------------------------------
    const auto vector_length = static_cast<int32_t>(
        std::round(static_cast<double>(config_params_fpga().fs_in) /
                   (static_cast<double>(GPS_L5I_CODE_RATE_CPS) /
                       static_cast<double>(GPS_L5I_CODE_LENGTH_CHIPS))));
    config_params_fpga().vector_length = vector_length;

    if (config_params_fpga().extend_correlation_symbols < 1)
        {
            config_params_fpga().extend_correlation_symbols = 1;
            std::cout << TEXT_RED
                      << "WARNING: GPS L5. extend_correlation_symbols must be >= 1. "
                      << "Coherent integration set to 1 symbol (1 ms)."
                      << TEXT_RESET << '\n';
        }
    else if (!config_params_fpga().track_pilot &&
             config_params_fpga().extend_correlation_symbols > GPS_L5I_NH_CODE_LENGTH)
        {
            config_params_fpga().extend_correlation_symbols = GPS_L5I_NH_CODE_LENGTH;
            std::cout << TEXT_RED
                      << "WARNING: GPS L5. extend_correlation_symbols must be <= 10 when tracking "
                      << "the data component. Set to 10 symbols (10 ms)."
                      << TEXT_RESET << '\n';
        }

    if ((config_params_fpga().extend_correlation_symbols > 1) &&
        (config_params_fpga().pll_bw_narrow_hz > config_params_fpga().pll_bw_hz ||
            config_params_fpga().dll_bw_narrow_hz > config_params_fpga().dll_bw_hz))
        {
            std::cout << TEXT_RED
                      << "WARNING: GPS L5. PLL/DLL narrow bandwidth larger than wide one."
                      << TEXT_RESET << '\n';
        }

    track_pilot_ = config_params_fpga().track_pilot;
    config_params_fpga().system = 'G';
    const std::array<char, 3> sig{'L', '5', '\0'};
    std::copy_n(sig.data(), 3, config_params_fpga().signal);

    // -------------------------------------------------------------------------
    // Configure FPGA tracking channel mapping to hardware accelerator devices
    // -------------------------------------------------------------------------
    configure_fpga_tracking_channel_mapping("L5");

    // -------------------------------------------------------------------------
    // Precompute local PRN codes (pilot/data)
    // -------------------------------------------------------------------------
    const uint32_t NUM_PRNs = 32;
    const auto code_length_chips = static_cast<uint32_t>(GPS_L5I_CODE_LENGTH_CHIPS);

    volk_gnsssdr::vector<float> tracking_code(code_length_chips, 0.0F);
    volk_gnsssdr::vector<float> data_code;

    if (track_pilot_)
        {
            data_code.resize(code_length_chips, 0.0F);
        }

    prn_codes_ptr_ = static_cast<int32_t*>(
        volk_gnsssdr_malloc(code_length_chips * NUM_PRNs * sizeof(int32_t),
            volk_gnsssdr_get_alignment()));

    if (track_pilot_)
        {
            data_codes_ptr_ = static_cast<int32_t*>(
                volk_gnsssdr_malloc(code_length_chips * NUM_PRNs * sizeof(int32_t),
                    volk_gnsssdr_get_alignment()));
        }

    for (uint32_t prn = 1; prn <= NUM_PRNs; ++prn)
        {
            if (track_pilot_)
                {
                    gps_l5q_code_gen_float(tracking_code, prn);
                    gps_l5i_code_gen_float(data_code, prn);

                    for (uint32_t s = 0; s < code_length_chips; ++s)
                        {
                            int32_t pilot_val = (tracking_code[s] < 0.0F) ? 0 : 1;
                            pilot_val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            prn_codes_ptr_[code_length_chips * (prn - 1) + s] = pilot_val;

                            int32_t data_val = (data_code[s] < 0.0F) ? 0 : 1;
                            data_val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY |
                                        LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
                            data_codes_ptr_[code_length_chips * (prn - 1) + s] = data_val;
                        }
                }
            else
                {
                    gps_l5i_code_gen_float(tracking_code, prn);
                    for (uint32_t s = 0; s < code_length_chips; ++s)
                        {
                            int32_t tmp_val = (tracking_code[s] < 0.0F) ? 0 : 1;
                            tmp_val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            prn_codes_ptr_[code_length_chips * (prn - 1) + s] = tmp_val;
                        }
                }
        }

    // -------------------------------------------------------------------------
    // Assign FPGA tracking configuration
    // -------------------------------------------------------------------------
    config_params_fpga().ca_codes = prn_codes_ptr_;
    config_params_fpga().data_codes = data_codes_ptr_;
    config_params_fpga().code_length_chips = code_length_chips;
    config_params_fpga().code_samples_per_chip = 1;
    config_params_fpga().extended_correlation_in_fpga = false;
    config_params_fpga().extend_fpga_integration_periods = 1;
    config_params_fpga().fpga_integration_period = 1;

    if (track_pilot_)
        {
            const auto extend = config_params_fpga().extend_correlation_symbols;
            if (extend > 1)
                {
                    if (extend <= GPS_L5I_NH_CODE_LENGTH &&
                        (GPS_L5I_NH_CODE_LENGTH % extend) == 0)
                        {
                            config_params_fpga().extended_correlation_in_fpga = true;
                            config_params_fpga().fpga_integration_period = extend;
                        }
                    else if (extend % GPS_L5I_NH_CODE_LENGTH == 0)
                        {
                            config_params_fpga().extended_correlation_in_fpga = true;
                            config_params_fpga().extend_fpga_integration_periods =
                                extend / GPS_L5I_NH_CODE_LENGTH;
                            config_params_fpga().fpga_integration_period = GPS_L5I_NH_CODE_LENGTH;
                        }
                }
        }

    // -------------------------------------------------------------------------
    // Create GNU Radio FPGA tracking block
    // -------------------------------------------------------------------------
    tracking_fpga_sc_sptr_ = dll_pll_veml_make_tracking_fpga(config_params_fpga());
    DLOG(INFO) << "tracking(" << tracking_fpga_sc_sptr_->unique_id() << ")";
}


GpsL5DllPllTrackingFpga::~GpsL5DllPllTrackingFpga()
{
    volk_gnsssdr_free(prn_codes_ptr_);
    if (track_pilot_)
        {
            volk_gnsssdr_free(data_codes_ptr_);
        }
}
