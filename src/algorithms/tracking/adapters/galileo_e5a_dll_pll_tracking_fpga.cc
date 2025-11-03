/*!
 * \file galileo_e5a_dll_pll_tracking_fpga.cc
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E5a signals for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
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

#include "galileo_e5a_dll_pll_tracking_fpga.h"
#include "Galileo_E5a.h"
#include "configuration_interface.h"
#include "display.h"
#include "galileo_e5_signal_replica.h"
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>
#include <array>
#include <cmath>  // for std::round

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GalileoE5aDllPllTrackingFpga::GalileoE5aDllPllTrackingFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : BaseDllPllTrackingFpga(configuration, role, in_streams, out_streams),
      data_codes_ptr_(nullptr),
      track_pilot_(config_params_fpga().track_pilot)
{
    // -------------------------------------------------------------------------
    // Adjust configuration parameters
    // -------------------------------------------------------------------------
    const auto vector_length = static_cast<int32_t>(
        std::round(config_params_fpga().fs_in /
                   (GALILEO_E5A_CODE_CHIP_RATE_CPS / GALILEO_E5A_CODE_LENGTH_CHIPS)));
    config_params_fpga().vector_length = vector_length;

    if (config_params_fpga().extend_correlation_symbols < 1)
        {
            config_params_fpga().extend_correlation_symbols = 1;
            std::cout << TEXT_RED
                      << "WARNING: Galileo E5a. extend_correlation_symbols must be >= 1. "
                      << "Coherent integration set to 1 symbol (1 ms)."
                      << TEXT_RESET << '\n';
        }
    else if (!config_params_fpga().track_pilot &&
             config_params_fpga().extend_correlation_symbols > GALILEO_E5A_I_SECONDARY_CODE_LENGTH)
        {
            config_params_fpga().extend_correlation_symbols = GALILEO_E5A_I_SECONDARY_CODE_LENGTH;
            std::cout << TEXT_RED
                      << "WARNING: Galileo E5a. extend_correlation_symbols cannot exceed 20 "
                      << "when tracking the data component. Set to 20 ms (20 symbols)."
                      << TEXT_RESET << '\n';
        }

    if ((config_params_fpga().extend_correlation_symbols > 1) &&
        (config_params_fpga().pll_bw_narrow_hz > config_params_fpga().pll_bw_hz ||
            config_params_fpga().dll_bw_narrow_hz > config_params_fpga().dll_bw_hz))
        {
            std::cout << TEXT_RED
                      << "WARNING: Galileo E5a. PLL/DLL narrow bandwidth is greater than wide one."
                      << TEXT_RESET << '\n';
        }

    config_params_fpga().system = 'E';
    const std::array<char, 3> sig{'5', 'X', '\0'};
    std::copy_n(sig.data(), 3, config_params_fpga().signal);

    // -------------------------------------------------------------------------
    // Configure FPGA device name
    // -------------------------------------------------------------------------
    device_name_ = configuration->property(role + ".devicename", default_device_name_Galileo_E5a);

    // Compute number of previously assigned channels (L1 + L2 + L5)
    const uint32_t num_prev_assigned_ch_1C = configuration->property("Channels_1C.count", 0);
    uint32_t num_prev_assigned_ch_2S = 0;
    if (configuration->property("Tracking_2S.devicename", std::string("")) != device_name_)
        {
            num_prev_assigned_ch_2S = configuration->property("Channels_2S.count", 0);
        }
    const uint32_t num_prev_assigned_ch_1B = configuration->property("Channels_1B.count", 0);
    set_num_prev_assigned_ch(num_prev_assigned_ch_1C + num_prev_assigned_ch_2S + num_prev_assigned_ch_1B);

    // -------------------------------------------------------------------------
    // Precompute local PRN codes
    // -------------------------------------------------------------------------
    const uint32_t code_samples_per_chip = 1;
    const auto code_length_chips = static_cast<uint32_t>(GALILEO_E5A_CODE_LENGTH_CHIPS);

    volk_gnsssdr::vector<gr_complex> aux_code(code_length_chips * code_samples_per_chip, gr_complex(0.0F, 0.0F));

    prn_codes_ptr_ = static_cast<int32_t*>(
        volk_gnsssdr_malloc(code_length_chips * code_samples_per_chip * GALILEO_E5A_NUMBER_OF_CODES * sizeof(int32_t),
            volk_gnsssdr_get_alignment()));

    if (track_pilot_)
        {
            data_codes_ptr_ = static_cast<int32_t*>(
                volk_gnsssdr_malloc(code_length_chips * code_samples_per_chip * GALILEO_E5A_NUMBER_OF_CODES * sizeof(int32_t),
                    volk_gnsssdr_get_alignment()));
        }

    for (uint32_t PRN = 1; PRN <= GALILEO_E5A_NUMBER_OF_CODES; ++PRN)
        {
            const std::array<char, 3> sig_a = {'5', 'X', '\0'};
            galileo_e5_a_code_gen_complex_primary(aux_code, PRN, sig_a);

            if (track_pilot_)
                {
                    for (uint32_t s = 0; s < code_length_chips; ++s)
                        {
                            int32_t imag_val = (aux_code[s].imag() < 0.0F) ? 0 : 1;
                            imag_val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            prn_codes_ptr_[code_length_chips * (PRN - 1) + s] = imag_val;

                            int32_t real_val = (aux_code[s].real() < 0.0F) ? 0 : 1;
                            real_val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY |
                                        LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
                            data_codes_ptr_[code_length_chips * (PRN - 1) + s] = real_val;
                        }
                }
            else
                {
                    for (uint32_t s = 0; s < code_length_chips; ++s)
                        {
                            int32_t val = (aux_code[s].real() < 0.0F) ? 0 : 1;
                            val |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            prn_codes_ptr_[code_length_chips * (PRN - 1) + s] = val;
                        }
                }
        }

    // -------------------------------------------------------------------------
    // Assign FPGA tracking configuration
    // -------------------------------------------------------------------------
    config_params_fpga().ca_codes = prn_codes_ptr_;
    config_params_fpga().data_codes = data_codes_ptr_;
    config_params_fpga().code_length_chips = code_length_chips;
    config_params_fpga().code_samples_per_chip = code_samples_per_chip;
    config_params_fpga().extended_correlation_in_fpga = false;
    config_params_fpga().extend_fpga_integration_periods = 1;
    config_params_fpga().fpga_integration_period = 1;

    if (track_pilot_)
        {
            const auto extend = config_params_fpga().extend_correlation_symbols;
            if (extend > 1)
                {
                    if (extend <= GALILEO_E5A_I_SECONDARY_CODE_LENGTH &&
                        (GALILEO_E5A_I_SECONDARY_CODE_LENGTH % extend) == 0)
                        {
                            config_params_fpga().extended_correlation_in_fpga = true;
                            config_params_fpga().fpga_integration_period = extend;
                        }
                    else if (extend % GALILEO_E5A_I_SECONDARY_CODE_LENGTH == 0)
                        {
                            config_params_fpga().extended_correlation_in_fpga = true;
                            config_params_fpga().extend_fpga_integration_periods =
                                extend / GALILEO_E5A_I_SECONDARY_CODE_LENGTH;
                            config_params_fpga().fpga_integration_period = GALILEO_E5A_I_SECONDARY_CODE_LENGTH;
                        }
                }
        }

    // -------------------------------------------------------------------------
    // Create GNU Radio FPGA tracking block
    // -------------------------------------------------------------------------
    tracking_fpga_sc_sptr_ = dll_pll_veml_make_tracking_fpga(config_params_fpga());
    DLOG(INFO) << "tracking(" << tracking_fpga_sc_sptr_->unique_id() << ")";
}


GalileoE5aDllPllTrackingFpga::~GalileoE5aDllPllTrackingFpga()
{
    volk_gnsssdr_free(prn_codes_ptr_);
    if (track_pilot_)
        {
            volk_gnsssdr_free(data_codes_ptr_);
        }
}
