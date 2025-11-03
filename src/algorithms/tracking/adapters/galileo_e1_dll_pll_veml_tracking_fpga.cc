/*!
 * \file galileo_e1_dll_pll_veml_tracking_fpga.cc
 * \brief  Adapts a DLL+PLL VEML (Very Early Minus Late) tracking loop block
 *   to a TrackingInterface for Galileo E1 signals for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
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


#include "galileo_e1_dll_pll_veml_tracking_fpga.h"
#include "Galileo_E1.h"
#include "configuration_interface.h"
#include "display.h"
#include "galileo_e1_signal_replica.h"
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>
#include <array>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

GalileoE1DllPllVemlTrackingFpga::GalileoE1DllPllVemlTrackingFpga(
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
    if (config_params_fpga().extend_correlation_symbols < 1)
        {
            config_params_fpga().extend_correlation_symbols = 1;
            std::cout << TEXT_RED
                      << "WARNING: Galileo E1. extend_correlation_symbols must be >= 1. "
                      << "Set to 1 symbol (4 ms)." << TEXT_RESET << '\n';
        }
    else if (!track_pilot_ && config_params_fpga().extend_correlation_symbols > 1)
        {
            config_params_fpga().extend_correlation_symbols = 1;
            std::cout << TEXT_RED
                      << "WARNING: Galileo E1. Extended coherent integration is only supported "
                      << "when tracking the pilot component. Set to 4 ms (1 symbol)."
                      << TEXT_RESET << '\n';
        }

    if ((config_params_fpga().extend_correlation_symbols > 1) &&
        (config_params_fpga().pll_bw_narrow_hz > config_params_fpga().pll_bw_hz ||
            config_params_fpga().dll_bw_narrow_hz > config_params_fpga().dll_bw_hz))
        {
            std::cout << TEXT_RED
                      << "WARNING: Galileo E1. Narrow tracking bandwidths are larger than wide ones."
                      << TEXT_RESET << '\n';
        }

    config_params_fpga().system = 'E';
    const std::array<char, 3> sig{'1', 'B', '\0'};
    std::copy_n(sig.data(), 3, config_params_fpga().signal);

    const auto vector_length = static_cast<int32_t>(
        std::round(config_params_fpga().fs_in /
                   (GALILEO_E1_CODE_CHIP_RATE_CPS / GALILEO_E1_B_CODE_LENGTH_CHIPS)));
    config_params_fpga().vector_length = vector_length;

    device_name_ = configuration->property(role + ".devicename", default_device_name_Galileo_E1);

    // -------------------------------------------------------------------------
    // Precompute all local codes (pilot and/or data)
    // -------------------------------------------------------------------------
    const uint32_t code_samples_per_chip = 2;
    const uint32_t num_codes = GALILEO_E1_NUMBER_OF_CODES;
    const uint32_t code_length = GALILEO_E1_B_CODE_LENGTH_CHIPS;

    prn_codes_ptr_ = static_cast<int32_t*>(
        volk_gnsssdr_malloc(code_length * code_samples_per_chip * num_codes * sizeof(int32_t),
            volk_gnsssdr_get_alignment()));

    volk_gnsssdr::vector<float> ca_codes_f(code_length * code_samples_per_chip);
    volk_gnsssdr::vector<float> data_codes_f;

    if (track_pilot_)
        {
            data_codes_ptr_ = static_cast<int32_t*>(
                volk_gnsssdr_malloc(code_length * code_samples_per_chip * num_codes * sizeof(int32_t),
                    volk_gnsssdr_get_alignment()));
            data_codes_f.resize(code_length * code_samples_per_chip);
        }

    for (uint32_t prn = 1; prn <= num_codes; prn++)
        {
            if (track_pilot_)
                {
                    galileo_e1_code_gen_sinboc11_float(ca_codes_f, {'1', 'C', '\0'}, prn);
                    galileo_e1_code_gen_sinboc11_float(data_codes_f, {'1', 'B', '\0'}, prn);

                    for (uint32_t s = 0; s < 2 * code_length; s++)
                        {
                            int32_t pilot = (ca_codes_f[s] < 0) ? 0 : 1;
                            pilot |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            prn_codes_ptr_[code_length * 2 * (prn - 1) + s] = pilot;

                            int32_t data = (data_codes_f[s] < 0) ? 0 : 1;
                            data |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY |
                                    LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
                            data_codes_ptr_[code_length * 2 * (prn - 1) + s] = data;
                        }
                }
            else
                {
                    galileo_e1_code_gen_sinboc11_float(ca_codes_f, {'1', 'B', '\0'}, prn);

                    for (uint32_t s = 0; s < 2 * code_length; s++)
                        {
                            int32_t tmp = (ca_codes_f[s] < 0) ? 0 : 1;
                            tmp |= LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            prn_codes_ptr_[code_length * 2 * (prn - 1) + s] = tmp;
                        }
                }
        }

    config_params_fpga().ca_codes = prn_codes_ptr_;
    config_params_fpga().data_codes = data_codes_ptr_;
    config_params_fpga().code_length_chips = code_length;
    config_params_fpga().code_samples_per_chip = code_samples_per_chip;
    config_params_fpga().extended_correlation_in_fpga = false;
    config_params_fpga().extend_fpga_integration_periods = 1;
    config_params_fpga().fpga_integration_period = 1;

    // -------------------------------------------------------------------------
    // Create GNU Radio FPGA tracking block
    // -------------------------------------------------------------------------
    tracking_fpga_sc_sptr_ = dll_pll_veml_make_tracking_fpga(config_params_fpga());
    DLOG(INFO) << "tracking(" << tracking_fpga_sc_sptr_->unique_id() << ")";
}


GalileoE1DllPllVemlTrackingFpga::~GalileoE1DllPllVemlTrackingFpga()
{
    volk_gnsssdr_free(prn_codes_ptr_);
    if (track_pilot_)
        {
            volk_gnsssdr_free(data_codes_ptr_);
        }
}
