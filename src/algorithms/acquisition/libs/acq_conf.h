/*!
 * \file acq_conf.h
 * \brief Class that contains all the configuration parameters for generic
 * acquisition block based on the PCPS algorithm.
 * \author Carles Fernandez, 2018. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ACQ_CONF_H
#define GNSS_SDR_ACQ_CONF_H

#include "configuration_interface.h"
#include <cstdint>
#include <string>

class Acq_Conf
{
public:
    Acq_Conf();

    void SetFromConfiguration(ConfigurationInterface *configuration, const std::string &role, double chip_rate, double opt_freq);

    /* PCPS Acquisition configuration */
    std::string item_type;
    std::string dump_filename;

    int64_t fs_in;
    int64_t resampled_fs;

    size_t it_size;

    float doppler_step;
    float samples_per_ms;
    float doppler_step2;
    float pfa;
    float pfa2;
    float samples_per_code;
    float resampler_ratio;

    uint32_t sampled_ms;
    uint32_t ms_per_code;
    uint32_t samples_per_chip;
    uint32_t chips_per_second;
    uint32_t max_dwells;
    uint32_t num_doppler_bins_step2;
    uint32_t resampler_latency_samples;
    uint32_t dump_channel;
    int32_t doppler_max;
    int32_t doppler_min;

    bool bit_transition_flag;
    bool use_CFAR_algorithm_flag;
    bool dump;
    bool blocking;
    bool blocking_on_standby;  // enable it only for unit testing to avoid sample consume on idle status
    bool make_2_steps;
    bool use_automatic_resampler;

private:
    void SetDerivedParams();

    void ConfigureAutomaticResampler(double opt_freq);
};

#endif
