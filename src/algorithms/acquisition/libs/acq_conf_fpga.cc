/*!
 * \file acq_conf_fpga.cc
 * \brief Class that contains all the configuration parameters for generic
 * acquisition block based on the PCPS algorithm running in the FPGA.
 * \author Marc Majoral, 2022. mmajoral(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2022  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "acq_conf_fpga.h"
#include "item_type_helpers.h"
#include "uio_fpga.h"
#include <cmath>
#include <iostream>
#include <utility>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

void Acq_Conf_Fpga::SetFromConfiguration(const ConfigurationInterface *configuration,
    const std::string &role, uint32_t blk_exp, double code_chips_per_sec, double num_chips_per_code)
{
    // sampling frequency
    const int64_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", fs_in);
    fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);

    // resampling frequency
    resampled_fs = fs_in;

    // max doppler
    doppler_max = configuration->property(role + ".doppler_max", doppler_max);

    // code chips per second
    code_rate_cps = code_chips_per_sec;

    // code length chips
    code_length_chips = num_chips_per_code;

    // code length in samples
    code_length = static_cast<uint32_t>(std::round(static_cast<double>(fs_in) / (code_rate_cps / code_length_chips)));

    // zero padding
    if (((code_length & (code_length - 1)) != 0))
        {
            // If the code length (in samples) is not a power of two, zero padding is required.
            enable_zero_padding = true;
        }

    // FFT length
    float zero_padding_adjustment = enable_zero_padding ? 2.0F : 1.0F;
    fft_size = pow(2, ceilf(log2f(static_cast<float>(code_length) * zero_padding_adjustment)));

    // Exclude limit value for the second maxima search
    excludelimit = static_cast<unsigned int>(1 + ceil((1.0 / code_rate_cps) * static_cast<float>(resampled_fs)));

    // repeat satellite
    repeat_satellite = configuration->property(role + ".repeat_satellite", false);

    // UIO device file
    std::string device_io_name;

    // find the uio device file corresponding to the acquisition
    if (find_uio_dev_file_name(device_io_name, acquisition_device_name, 0) < 0)
        {
            std::cout << "Cannot find the FPGA uio device file corresponding to device name " << acquisition_device_name << std::endl;
            throw std::exception();
        }
    device_name = std::move(device_io_name);

    // parameters for the second acquisition step
    num_doppler_bins_step2 = configuration->property(role + ".second_nbins", num_doppler_bins_step2);
    doppler_step2 = configuration->property(role + ".second_doppler_step", doppler_step2);
    doppler_step = configuration->property(role + ".doppler_step", doppler_step);
    make_2_steps = configuration->property(role + ".make_two_steps", make_2_steps);
    max_num_acqs = configuration->property(role + ".max_num_acqs", 2);

    // reference for the FPGA FFT-IFFT attenuation factor
    total_block_exp = configuration->property(role + ".total_block_exp", blk_exp);
}

bool Acq_Conf_Fpga::ConfigureAutomaticResampler(std::vector<std::pair<uint32_t, uint32_t>> downsampling_filter_specs, uint32_t max_FFT_size, double opt_freq)
{
    bool acq_configuration_valid = false;
    uint32_t optimal_downsampling_factor = fs_in / opt_freq;

    if (optimal_downsampling_factor > 1)
        {
            for (uint32_t k = 0; k < downsampling_filter_specs.size(); k++)
                {
                    uint32_t dec_factor = downsampling_filter_specs[k].first;
                    uint32_t filter_delay = downsampling_filter_specs[k].second;
                    // Select the FPGA-supported downsampling factor that is the closest to, but smaller than, the target downsampling factor
                    if (optimal_downsampling_factor >= dec_factor)
                        {
                            // check if the required FFT size is supported in the FPGA
                            uint32_t fft_size_downsampled = (fft_size / dec_factor);
                            if (fft_size_downsampled <= max_FFT_size)
                                {
                                    downsampling_filter_num = k + 1;
                                    downsampling_factor = dec_factor;
                                    downsampling_filter_delay = filter_delay;

                                    // update the resampling frequency, the code length, the FFT size, and the exclude limit parameter taking into account the downsampling factor
                                    resampled_fs = fs_in / downsampling_factor;
                                    code_length = static_cast<uint32_t>(std::round(static_cast<double>(resampled_fs) / (code_rate_cps / code_length_chips)));
                                    float zero_padding_adjustment = enable_zero_padding ? 2.0F : 1.0F;
                                    fft_size = pow(2, ceilf(log2f(static_cast<float>(code_length) * zero_padding_adjustment)));
                                    excludelimit = static_cast<unsigned int>(1 + ceil((1.0 / code_rate_cps) * static_cast<float>(resampled_fs)));
                                    acq_configuration_valid = true;
                                }
                            break;
                        }
                }
        }
    else
        {
            // the buffer containing L2 or L5/E5a frequency band samples by default does not implement a downsampling filter
            if (fft_size <= max_FFT_size)
                {
                    acq_configuration_valid = true;
                }
        }

    return acq_configuration_valid;
}

bool Acq_Conf_Fpga::Is_acq_config_valid(uint32_t max_FFT_size)
{
    if (fft_size <= max_FFT_size)
        {
            return true;
        }
    else
        {
            return false;
        }
}
