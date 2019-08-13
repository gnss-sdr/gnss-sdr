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
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_ACQ_CONF_H_
#define GNSS_SDR_ACQ_CONF_H_

#include <cstddef>
#include <cstdint>
#include <string>

class Acq_Conf
{
public:
    /* PCPS Acquisition configuration */
    uint32_t sampled_ms;
    uint32_t ms_per_code;
    uint32_t samples_per_chip;
    uint32_t max_dwells;
    uint32_t doppler_max;
    uint32_t num_doppler_bins_step2;
    float doppler_step2;
    int64_t fs_in;
    float samples_per_ms;
    float samples_per_code;
    bool bit_transition_flag;
    bool use_CFAR_algorithm_flag;
    bool dump;
    bool blocking;
    bool blocking_on_standby;  // enable it only for unit testing to avoid sample consume on idle status
    bool make_2_steps;
    bool use_automatic_resampler;
    float resampler_ratio;
    int64_t resampled_fs;
    uint32_t resampler_latency_samples;
    std::string dump_filename;
    uint32_t dump_channel;
    size_t it_size;

    Acq_Conf();
};

#endif
