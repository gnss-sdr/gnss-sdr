/*!
 * \file Kf_conf.h
 * \brief Class that contains all the configuration parameters for generic tracking block based on a Kalman Filter.
 * \author Javier Arribas, 2020. jarribas(at)cttc.es
 *
 * Class that contains all the configuration parameters for generic tracking block based on a DLL and a PLL.
 *
 * -----------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_Kf_CONF_H
#define GNSS_SDR_Kf_CONF_H

#include "configuration_interface.h"
#include <cstdint>
#include <string>

class Kf_Conf
{
public:
    Kf_Conf();
    void SetFromConfiguration(const ConfigurationInterface *configuration, const std::string &role);

    std::string item_type;
    std::string dump_filename;
    double fs_in;
    double carrier_lock_th;

    // KF statistics
    // Measurement covariances (R)
    double code_disc_sd_chips;
    double carrier_disc_sd_rads;

    // System covariances (Q)
    double code_phase_sd_chips;
    double carrier_phase_sd_rad;
    double carrier_freq_sd_hz;
    double carrier_freq_rate_sd_hz_s;

    // initial Kalman covariance matrix (P)
    double init_code_phase_sd_chips;
    double init_carrier_phase_sd_rad;
    double init_carrier_freq_sd_hz;
    double init_carrier_freq_rate_sd_hz_s;

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
    int32_t extend_correlation_symbols;
    int32_t cn0_samples;
    int32_t cn0_smoother_samples;
    int32_t carrier_lock_test_smoother_samples;
    int32_t cn0_min;
    int32_t max_code_lock_fail;
    int32_t max_carrier_lock_fail;
    char signal[3]{};
    char system;
    bool track_pilot;
    bool enable_doppler_correction;
    bool high_dyn;
    bool dump;
    bool dump_mat;
};

#endif
