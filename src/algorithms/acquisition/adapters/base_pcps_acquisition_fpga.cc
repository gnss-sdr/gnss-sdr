/*!
 * \file base_pcps_acquisition_fpga.cc
 * \brief Shared implementation for FPGA-based PCPS acquisition adapters
 * \authors Carles Fernandez, 2025. carles.fernandez(at)cttc.cat
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2025  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "base_pcps_acquisition_fpga.h"
#include "configuration_interface.h"
#include "gnss_sdr_flags.h"
#include <vector>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

BasePcpsAcquisitionFpga::BasePcpsAcquisitionFpga(
    const ConfigurationInterface* configuration,
    std::string role,
    double code_rate_cps,
    double code_length_chips,
    uint32_t opt_acq_fs_sps,
    uint32_t default_fpga_blk_exp,
    uint32_t acq_buff,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(std::move(role))
{
    if (in_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams > 0)
        {
            LOG(ERROR) << "This implementation does not provide an output stream";
        }

    // set acquisition parameters
    acq_parameters_.SetFromConfiguration(configuration, role_, default_fpga_blk_exp, code_rate_cps, code_length_chips);

    // Query the capabilities of the instantiated FPGA Acquisition IP Core
    std::vector<std::pair<uint32_t, uint32_t>> downsampling_filter_specs;
    uint32_t max_FFT_size;
    acquisition_fpga_ = pcps_make_acquisition_fpga(&acq_parameters_, acq_buff, downsampling_filter_specs, max_FFT_size);

    // Configure the automatic resampler according to the capabilities of the instantiated FPGA Acquisition IP Core.
    // When the FPGA is in use, the acquisition resampler operates only in the L1/E1 frequency band.
    if (acq_buff == 0)
        {
            bool acq_configuration_valid = acq_parameters_.ConfigureAutomaticResampler(downsampling_filter_specs, max_FFT_size, opt_acq_fs_sps);
            if (!acq_configuration_valid)
                {
                    std::cout << "The FPGA acquisition IP does not support the required sampling frequency of " << acq_parameters_.fs_in << " SPS for the L1/E1 band. Please update the sampling frequency in the configuration file." << std::endl;
                    exit(0);
                }
        }
    else
        {
            bool acq_configuration_valid = acq_parameters_.Is_acq_config_valid(max_FFT_size);
            if (!acq_configuration_valid)
                {
                    std::cout << "The FPGA acquisition IP does not support the required sampling frequency of " << acq_parameters_.fs_in << " SPS for the L2/L5 band. Please update the sampling frequency in the configuration file." << std::endl;
                    exit(0);
                }
        }

    DLOG(INFO) << "role " << role_;

#if USE_GLOG_AND_GFLAGS
    if (FLAGS_doppler_max != 0)
        {
            acq_parameters_.doppler_max = FLAGS_doppler_max;
        }
    if (FLAGS_doppler_step != 0)
        {
            acq_parameters_.doppler_step = static_cast<float>(FLAGS_doppler_step);
        }
#else
    if (absl::GetFlag(FLAGS_doppler_max) != 0)
        {
            acq_parameters_.doppler_max = absl::GetFlag(FLAGS_doppler_max);
        }
    if (absl::GetFlag(FLAGS_doppler_step) != 0)
        {
            acq_parameters_.doppler_step = static_cast<float>(absl::GetFlag(FLAGS_doppler_step));
        }
#endif
}


void BasePcpsAcquisitionFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            // FPGA acquisition blocks usually don’t connect to GNU Radio flowgraphs
        }
}


void BasePcpsAcquisitionFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
            // No connections to remove
        }
}


gr::basic_block_sptr BasePcpsAcquisitionFpga::get_left_block()
{
    return nullptr;
}


gr::basic_block_sptr BasePcpsAcquisitionFpga::get_right_block()
{
    return nullptr;
}


void BasePcpsAcquisitionFpga::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_gnss_synchro(gnss_synchro);
        }
}


void BasePcpsAcquisitionFpga::set_channel(unsigned int channel)
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_channel(channel);
        }
}


void BasePcpsAcquisitionFpga::set_channel_fsm(std::weak_ptr<ChannelFsm> channel_fsm)
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_channel_fsm(channel_fsm);
        }
}


void BasePcpsAcquisitionFpga::set_threshold(float threshold)
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_threshold(threshold);
        }
}


void BasePcpsAcquisitionFpga::set_doppler_center(int doppler_center)
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_doppler_center(doppler_center);
        }
}


void BasePcpsAcquisitionFpga::set_state(int state)
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_state(state);
        }
}


void BasePcpsAcquisitionFpga::reset()
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->set_active(true);
        }
}


void BasePcpsAcquisitionFpga::stop_acquisition()
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->stop_acquisition();
        }
}


void BasePcpsAcquisitionFpga::init()
{
    if (acquisition_fpga_)
        {
            acquisition_fpga_->init();
        }
}


signed int BasePcpsAcquisitionFpga::mag()
{
    if (acquisition_fpga_)
        {
            return acquisition_fpga_->mag();
        }
    return 0;
}


void BasePcpsAcquisitionFpga::set_local_code()
{
    acquisition_fpga_->set_local_code();
}
