/*!
 * \file acq_conf.cc
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

#include "acq_conf.h"
#include "item_type_helpers.h"
#include <glog/logging.h>
#include <gnuradio/gr_complex.h>
#include <cmath>

Acq_Conf::Acq_Conf()
{
    /* PCPS acquisition configuration */
    sampled_ms = 1U;
    ms_per_code = 1U;
    max_dwells = 1U;
    samples_per_chip = 2U;
    chips_per_second = 1023000;
    doppler_max = 5000;
    doppler_min = -5000;
    doppler_step = 250.0;
    num_doppler_bins_step2 = 4U;
    doppler_step2 = 125.0;
    pfa = 0.0;
    pfa2 = 0.0;
    fs_in = 4000000;
    samples_per_ms = 0.0;
    samples_per_code = 0.0;
    bit_transition_flag = false;
    use_CFAR_algorithm_flag = true;
    dump = false;
    blocking = true;
    make_2_steps = false;
    dump_channel = 0U;
    it_size = sizeof(gr_complex);
    item_type = std::string("gr_complex");
    blocking_on_standby = false;
    use_automatic_resampler = false;
    resampler_ratio = 1.0;
    resampled_fs = 0LL;
    resampler_latency_samples = 0U;
    enable_monitor_output = false;
}


void Acq_Conf::SetFromConfiguration(const ConfigurationInterface *configuration,
    const std::string &role, double chip_rate, double opt_freq)
{
    item_type = configuration->property(role + ".item_type", item_type);
    if (!item_type_valid(item_type))
        {
            throw std::invalid_argument("Unknown item type: " + item_type);
        }

    chips_per_second = chip_rate;

    const int64_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", fs_in);
    fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    doppler_max = configuration->property(role + ".doppler_max", doppler_max);
    sampled_ms = configuration->property(role + ".coherent_integration_time_ms", sampled_ms);
    bit_transition_flag = configuration->property(role + ".bit_transition_flag", bit_transition_flag);
    max_dwells = configuration->property(role + ".max_dwells", max_dwells);
    dump = configuration->property(role + ".dump", dump);
    dump_channel = configuration->property(role + ".dump_channel", dump_channel);
    blocking = configuration->property(role + ".blocking", blocking);
    dump_filename = configuration->property(role + ".dump_filename", dump_filename);

    use_automatic_resampler = configuration->property("GNSS-SDR.use_acquisition_resampler", use_automatic_resampler);

    if ((sampled_ms % ms_per_code) != 0)
        {
            LOG(WARNING) << "Parameter coherent_integration_time_ms should be a multiple of "
                         << ms_per_code << ". Setting it to " << ms_per_code;
            sampled_ms = ms_per_code;
        }

    resampled_fs = fs_in;

    if (use_automatic_resampler)
        {
            ConfigureAutomaticResampler(opt_freq);
        }

    it_size = item_type_size(item_type);
    num_doppler_bins_step2 = configuration->property(role + ".second_nbins", num_doppler_bins_step2);
    doppler_step2 = configuration->property(role + ".second_doppler_step", doppler_step2);
    doppler_step = configuration->property(role + ".doppler_step", doppler_step);
    pfa = configuration->property(role + ".pfa", pfa);
    if ((pfa < 0.0) or (pfa > 1.0))
        {
            LOG(WARNING) << "Parameter pfa should between 0.0 and 1.0. Setting it to 0.0";
            pfa = 0.0;
        }
    pfa2 = configuration->property(role + ".pfa_second_step", pfa2);
    if ((pfa2 <= 0.0) or (pfa2 > 1.0))
        {
            pfa2 = pfa;
        }
    make_2_steps = configuration->property(role + ".make_two_steps", make_2_steps);
    blocking_on_standby = configuration->property(role + ".blocking_on_standby", blocking_on_standby);

    if (pfa <= 0.0)
        {
            // if pfa is not set, we use the first_vs_second_peak_statistic metric
            use_CFAR_algorithm_flag = false;
        }

    enable_monitor_output = configuration->property("AcquisitionMonitor.enable_monitor", false);

    SetDerivedParams();
}


void Acq_Conf::ConfigureAutomaticResampler(double opt_freq)
{
    if (use_automatic_resampler)
        {
            if (fs_in > opt_freq)
                {
                    resampler_ratio = floor(static_cast<float>(fs_in) / opt_freq);
                    uint32_t decimation = fs_in / opt_freq;
                    while (fs_in % decimation > 0)
                        {
                            decimation--;
                        };
                    resampler_ratio = decimation;
                    resampled_fs = fs_in / static_cast<int>(resampler_ratio);
                }
            // --- Find number of samples per spreading code -------------------
            SetDerivedParams();
        }
}


void Acq_Conf::SetDerivedParams()
{
    samples_per_ms = static_cast<float>(resampled_fs) * 0.001F;
    samples_per_chip = static_cast<unsigned int>(std::ceil(static_cast<float>(resampled_fs) / chips_per_second));
    samples_per_code = samples_per_ms * ms_per_code;
}
