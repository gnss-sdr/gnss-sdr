/*!
 * \file dll_pll_conf.h
 * \brief Class that contains all the configuration parameters for generic tracking block based on a DLL and a PLL.
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

#ifndef GNSS_SDR_DLL_PLL_CONF_H
#define GNSS_SDR_DLL_PLL_CONF_H

#include "configuration_interface.h"
#include <cstdint>
#include <string>

/** \addtogroup Tracking
 * \{ */
/** \addtogroup Tracking_libs
 * \{ */


class Dll_Pll_Conf
{
public:
    Dll_Pll_Conf();
    void SetFromConfiguration(const ConfigurationInterface *configuration, const std::string &role);

    /* DLL/PLL tracking configuration */
    std::string item_type{"gr_complex"};
    std::string dump_filename{"./dll_pll_dump.dat"};
    std::string f_error_dump_filename{"./f_error_dump.csv"};
    double fs_in{2000000.0};
    double carrier_lock_th{0.0};
    double bs_dominance_ratio{0.5};
    double bs_runner_up_margin{0.3};
    double bs_transition_confidence{0.6};
    float pll_pull_in_bw_hz{50.0};
    float dll_pull_in_bw_hz{3.0};
    float fll_bw_hz{35.0};
    float pll_bw_hz{35.0};
    float dll_bw_hz{2.0};
    float pll_bw_narrow_hz{5.0};
    float dll_bw_narrow_hz{0.75};
    float early_late_space_chips{0.25};
    float very_early_late_space_chips{0.5};
    float early_late_space_narrow_chips{0.15};
    float very_early_late_space_narrow_chips{0.5};
    float slope{1.0};
    float spc{0.5};
    float y_intercept{1.0};
    float cn0_smoother_alpha{0.002};
    float carrier_lock_test_smoother_alpha{0.002};
    float bs_min_prompt_mag{0.0};
    uint32_t pull_in_time_s{5U};
    uint32_t bit_synchronization_time_limit_s{20U};
    uint32_t vector_length{0U};
    uint32_t smoother_length{10U};
    uint32_t f_error_accumulation{20U};
    uint32_t f_error_step_num{0U};
    double f_error_doppler_step{250.0};
    int32_t fll_filter_order{1};
    int32_t pll_filter_order{3};
    int32_t dll_filter_order{2};
    int32_t extend_correlation_symbols{1};
    int32_t cn0_samples{0};
    int32_t cn0_smoother_samples{200};
    int32_t carrier_lock_test_smoother_samples{25};
    int32_t cn0_min{0};
    int32_t max_code_lock_fail{0};
    int32_t max_carrier_lock_fail{0};
    int32_t bs_stable_best_required{3};
    int32_t bs_min_events_for_lock{6};
    int32_t bs_transition_window_epochs{4};
    int32_t bs_tentative_events_required{2};
    char signal[3]{};
    char system{'G'};
    bool enable_fll_pull_in{false};
    bool enable_fll_steady_state{false};
    bool track_pilot{true};
    bool enable_doppler_correction{false};
    bool carrier_aiding{true};
    bool high_dyn{false};
    bool dump{false};
    bool dump_mat{true};
    bool tow_to_trk{false};
    bool bs_use_phase_dot_detector{true};
    // BeiDou B1C-specific options.
    bool qmboc{true};  //!< QMBOC local replica (same key as Acquisition_*.qmboc).
    bool b1c_prompt_use_data_q{true};
    bool b1c_prompt_normalize_power{true};
    float b1c_data_prompt_scale{1.7320508F};  // sqrt(3), compensates 1:3 data:pilot power.
    float b1c_pilot_prompt_scale{1.0F};
    float b1c_secondary_lock_ratio{0.88F};
};


/** \} */
/** \} */
#endif  // GNSS_SDR_DLL_PLL_CONF_H
