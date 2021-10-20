/*!
 * \file acq_conf.h
 * \brief Class that contains all the configuration parameters for generic
 * acquisition block based on the PCPS algorithm.
 * \author Carles Fernandez, 2018. cfernandez(at)cttc.es
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

#ifndef GNSS_SDR_ACQ_CONF_H
#define GNSS_SDR_ACQ_CONF_H

#include "configuration_interface.h"
#include <gnuradio/gr_complex.h>
#include <cstdint>
#include <string>

/** \addtogroup Acquisition
 * \{ */
/** \addtogroup acquisition_libs acquisition_libs
 * Library with utilities for GNSS signal acquisition
 * \{ */


class Acq_Conf
{
public:
    Acq_Conf() = default;

    void SetFromConfiguration(const ConfigurationInterface *configuration, const std::string &role, double chip_rate, double opt_freq);

    /* PCPS Acquisition configuration */
    std::string item_type{"gr_complex"};
    std::string dump_filename;

    int64_t fs_in{4000000LL};
    int64_t resampled_fs{0LL};

    size_t it_size{sizeof(gr_complex)};

    float doppler_step{250.0};
    float samples_per_ms{0.0};
    float doppler_step2{125.0};
    float pfa{0.0};
    float pfa2{0.0};
    float samples_per_code{0.0};
    float resampler_ratio{1.0};

    uint32_t sampled_ms{1U};
    uint32_t ms_per_code{1U};
    uint32_t samples_per_chip{2U};
    uint32_t chips_per_second{1023000U};
    uint32_t max_dwells{1U};
    uint32_t num_doppler_bins_step2{4U};
    uint32_t resampler_latency_samples{0U};
    uint32_t dump_channel{0U};
    int32_t doppler_max{5000};
    int32_t doppler_min{-5000};

    bool bit_transition_flag{false};
    bool use_CFAR_algorithm_flag{true};
    bool dump{false};
    bool blocking{true};
    bool blocking_on_standby{false};  // enable it only for unit testing to avoid sample consume on idle status
    bool make_2_steps{false};
    bool use_automatic_resampler{false};
    bool enable_monitor_output{false};

private:
    void SetDerivedParams();

    void ConfigureAutomaticResampler(double opt_freq);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_ACQ_CONF_H
