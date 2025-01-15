/*!
 * \file dll_pll_conf_fpga.h
 * \brief Class that contains all the configuration parameters for generic
 * tracking block based on a DLL and a PLL for the FPGA.
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
 * \author Javier Arribas, 2018. jarribas(at)cttc.es
 *
 * Class that contains all the configuration parameters for generic tracking block based on a DLL and a PLL.
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

#ifndef GNSS_SDR_DLL_PLL_CONF_FPGA_H
#define GNSS_SDR_DLL_PLL_CONF_FPGA_H

#include "configuration_interface.h"
#include <cstdint>
#include <string>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


class Dll_Pll_Conf_Fpga
{
public:
    Dll_Pll_Conf_Fpga();
    void SetFromConfiguration(const ConfigurationInterface* configuration, const std::string& role);

    /* DLL/PLL tracking configuration */
    std::string device_name{"/dev/uio"};
    std::string dump_filename{"./dll_pll_dump.dat"};

    double fs_in{12500000.0};
    double carrier_lock_th{0.0};

    float pll_pull_in_bw_hz{50.0};
    float dll_pull_in_bw_hz{3.0};
    float fll_bw_hz{35.0};
    float pll_bw_hz{5.0};
    float dll_bw_hz{0.5};
    float pll_bw_narrow_hz{2.0};
    float dll_bw_narrow_hz{0.25};
    float early_late_space_chips{0.25};
    float very_early_late_space_chips{0.5};
    float early_late_space_narrow_chips{0.15};
    float very_early_late_space_narrow_chips{0.5};
    float slope{1.0};
    float spc{0.5};
    float y_intercept{1.0};
    float cn0_smoother_alpha{0.002};
    float carrier_lock_test_smoother_alpha{0.002};

    uint32_t pull_in_time_s{10U};  // signed integer, when pull in time is not yet reached it has to be compared against a negative number
    uint32_t bit_synchronization_time_limit_s{70U};
    uint32_t vector_length{0U};
    uint32_t smoother_length{10U};
    uint32_t code_length_chips{0U};
    uint32_t code_samples_per_chip{0U};
    uint32_t extend_fpga_integration_periods{1};
    uint32_t fpga_integration_period{0};

    int32_t fll_filter_order{1};
    int32_t pll_filter_order{3};
    int32_t dll_filter_order{2};
    int32_t extend_correlation_symbols{1};
    int32_t cn0_samples{0};
    int32_t cn0_min{0};
    int32_t max_code_lock_fail{0};
    int32_t max_carrier_lock_fail{0};
    int32_t cn0_smoother_samples{200};
    int32_t carrier_lock_test_smoother_samples{25};
    // int32_t max_lock_fail;

    int32_t* ca_codes{nullptr};
    int32_t* data_codes{nullptr};

    char signal[3]{};
    char system{'G'};

    bool extended_correlation_in_fpga{false};
    bool track_pilot{true};
    bool enable_doppler_correction{false};
    bool enable_fll_pull_in{false};
    bool enable_fll_steady_state{false};
    bool carrier_aiding{true};
    bool high_dyn{false};
    bool dump{false};
    bool dump_mat{true};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_DLL_PLL_CONF_FPGA_H
