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
    std::string item_type;
    std::string dump_filename;
    double fs_in;
    double carrier_lock_th;
    float pll_pull_in_bw_hz;
    float dll_pull_in_bw_hz;
    float fll_bw_hz;
    float pll_bw_hz;
    float dll_bw_hz;
    float pll_bw_narrow_hz;
    float dll_bw_narrow_hz;
    float early_late_space_chips;
    float very_early_late_space_chips;
    float early_late_space_narrow_chips;
    float very_early_late_space_narrow_chips;
    float slope;
    float spc;
    float y_intercept;
    float cn0_smoother_alpha;
    float carrier_lock_test_smoother_alpha;
    uint32_t pull_in_time_s;
    uint32_t bit_synchronization_time_limit_s;
    uint32_t vector_length;
    uint32_t smoother_length;
    int32_t fll_filter_order;
    int32_t pll_filter_order;
    int32_t dll_filter_order;
    int32_t extend_correlation_symbols;
    int32_t cn0_samples;
    int32_t cn0_smoother_samples;
    int32_t carrier_lock_test_smoother_samples;
    int32_t cn0_min;
    int32_t max_code_lock_fail;
    int32_t max_carrier_lock_fail;
    char signal[3]{};
    char system;
    bool enable_fll_pull_in;
    bool enable_fll_steady_state;
    bool track_pilot;
    bool enable_doppler_correction;
    bool carrier_aiding;
    bool high_dyn;
    bool dump;
    bool dump_mat;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_DLL_PLL_CONF_H
