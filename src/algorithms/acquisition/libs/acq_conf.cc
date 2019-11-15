/*!
 * \file acq_conf.cc
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

#include "acq_conf.h"
#include "item_type_helpers.h"
#include <glog/logging.h>
#include <gnuradio/gr_complex.h>

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
    dump_filename = "";
    dump_channel = 0U;
    it_size = sizeof(gr_complex);
    item_type = "gr_complex";
    blocking_on_standby = false;
    use_automatic_resampler = false;
    resampler_ratio = 1.0;
    resampled_fs = 0LL;
    resampler_latency_samples = 0U;
}

void Acq_Conf::SetFromConfiguration(ConfigurationInterface *configuration,
    const std::string &role, double chip_rate, double opt_freq)
{
    item_type = configuration->property(role + ".item_type", item_type);
    if (!item_type_valid(item_type))
        {
            throw std::invalid_argument("Unknown item type: " + item_type);
        }

    chips_per_second = chip_rate;

    int64_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", fs_in);
    fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    doppler_max = configuration->property(role + ".doppler_max", doppler_max);
    sampled_ms = configuration->property(role + ".coherent_integration_time_ms", sampled_ms);
    bit_transition_flag = configuration->property(role + ".bit_transition_flag", bit_transition_flag);
    use_CFAR_algorithm_flag = configuration->property(role + ".use_CFAR_algorithm", use_CFAR_algorithm_flag);  //will be false in future versions
    //acquire_pilot = configuration->property(role + ".acquire_pilot", acquire_pilot);  //will be true in future versions
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
    pfa2 = configuration->property(role + ".pfa_second_step", pfa2);
    if (pfa2 <= 0.0)
        pfa2 = pfa;
    make_2_steps = configuration->property(role + ".make_two_steps", make_2_steps);
    blocking_on_standby = configuration->property(role + ".blocking_on_standby", blocking_on_standby);

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
            //--- Find number of samples per spreading code -------------------------
            SetDerivedParams();
        }
}

void Acq_Conf::SetDerivedParams()
{
    samples_per_ms = static_cast<float>(resampled_fs) * 0.001;
    samples_per_chip = static_cast<unsigned int>(ceil(static_cast<float>(resampled_fs) / chips_per_second));
    samples_per_code = samples_per_ms * ms_per_code;
}
