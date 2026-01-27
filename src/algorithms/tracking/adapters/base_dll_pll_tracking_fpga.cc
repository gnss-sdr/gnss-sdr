/*!
 * \file base_dll_pll_tracking_fpga.cc
 * \brief Base class providing shared logic for DLL+PLL VEML tracking adapters
 * for FPGA-based devices
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

#include "base_dll_pll_tracking_fpga.h"
#include "configuration_interface.h"
#include "uio_fpga.h"

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

BaseDllPllTrackingFpga::BaseDllPllTrackingFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(role),
      channel_(0),
      signal_base_channel_index_(0)
{
    trk_params_.SetFromConfiguration(configuration, role_);
    if (in_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
    DLOG(INFO) << "role " << role_;
}


void BaseDllPllTrackingFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
        }
}


void BaseDllPllTrackingFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        {
        }
}


gr::basic_block_sptr BaseDllPllTrackingFpga::get_left_block()
{
    return tracking_fpga_sc_sptr_;
}


gr::basic_block_sptr BaseDllPllTrackingFpga::get_right_block()
{
    return tracking_fpga_sc_sptr_;
}


void BaseDllPllTrackingFpga::start_tracking()
{
    tracking_fpga_sc_sptr_->start_tracking();
}


void BaseDllPllTrackingFpga::stop_tracking()
{
    tracking_fpga_sc_sptr_->stop_tracking();
}

void BaseDllPllTrackingFpga::configure_fpga_tracking_channel_mapping(std::string signal)
{
    set_signal(signal);
    set_device_name();
    set_signal_channel_base_index();
}

void BaseDllPllTrackingFpga::set_signal(std::string signal)
{
    signal_ = signal;
}

void BaseDllPllTrackingFpga::set_device_name()
{
    auto it = signal_to_device_.find(signal_);
    if (it != signal_to_device_.end())
        {
            device_name_ = it->second;
        }
}

void BaseDllPllTrackingFpga::set_signal_channel_base_index()
{
    // update channel mapping count
    std::lock_guard<std::mutex> lock(channel_counts_mtx_);
    channel_counts_.at(signal_)++;

    // compute signal_base_channel_index
    uint32_t signal_base_channel_index = 0;
    for (const auto& [label, num_correlators] : channel_counts_)
        {
            if (label != signal_)
                {
                    // Check whether any preceding signal in the initialization order uses the same FPGA tracking multicorrelator device type as signal_
                    bool same_tracking_multicorrelator_type = false;
                    auto signal_to_device = signal_to_device_.find(label);
                    if (signal_to_device->second == device_name_)
                        {
                            same_tracking_multicorrelator_type = true;
                        }
                    // If no preceding signal shares the same multicorrelator device type, include signal_ channels in the base index computation.
                    if (!same_tracking_multicorrelator_type)
                        {
                            signal_base_channel_index += num_correlators;
                        }
                }
        }

    // Check for tracking multicorrelators assigned as an alternative
    uint32_t num_alternative_devices = get_num_alternative_devices_locked_();
    signal_base_channel_index -= num_alternative_devices;

    // Update signal_base_channel_index_
    signal_base_channel_index_ = signal_base_channel_index;
}

uint32_t BaseDllPllTrackingFpga::get_num_alternative_devices_locked_() const
{
    // Get the default FPGA hardware correlator device name for signal_
    std::string default_device_name;
    auto device_name_it = signal_to_device_.find(signal_);
    if (device_name_it != signal_to_device_.end())
        {
            default_device_name = device_name_it->second;
        }

    // Check if this FPGA hardware correlator name is assigned as an alternative to another signal
    bool alternative_assignment = false;
    std::string alternative_signal_type;
    for (const auto& [signal_type, alternative_device_name] : signal_to_alternative_device_)
        {
            if (alternative_device_name == default_device_name)
                {
                    alternative_signal_type = signal_type;
                    alternative_assignment = true;
                    break;  // only one match is possible
                }
        }

    if (alternative_assignment)
        {
            // Get the number of channels for the signal that uses this FPGA hardware correlator name as an alternative
            uint32_t num_channels = channel_counts_.at(alternative_signal_type);

            // Get the default FPGA device name for the signal that uses this hardware correlator name as an alternative
            std::string device_name;
            auto device_it = signal_to_device_.find(alternative_signal_type);
            if (device_it != signal_to_device_.end())
                {
                    device_name = device_it->second;
                }

            // Get the number of devices available by default for the signal that uses this hardware correlator name as an alternative
            uint32_t num_devices_available = get_num_devices(device_name);

            // return the actual number of FPGA tracking multicorrelator devices used as an alternative
            if (num_devices_available < num_channels)
                {
                    return num_channels - num_devices_available;
                }
            return 0;
        }
    return 0;
}

void BaseDllPllTrackingFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    std::string device_io_name;

    if (find_uio_dev_file_name(device_io_name, device_name_, channel_ - signal_base_channel_index_) < 0)
        {
            // If insufficient FPGA tracking multicorrelators are available for a GNSS signal, check whether compatible multicorrelators from another signal can be used as an alternative.
            auto it = signal_to_alternative_device_.find(signal_);
            if (it != signal_to_alternative_device_.end())
                {
                    uint32_t alternate_device_channel_index = channel_ - signal_base_channel_index_ - get_num_devices(it->second);  // Channel number in the alternative tracking multicorrelator devices
                    if (find_uio_dev_file_name(device_io_name, it->second, alternate_device_channel_index) >= 0)
                        {
                            tracking_fpga_sc_sptr_->set_channel(channel_, device_io_name);
                        }
                    else
                        {
                            std::cout << "Cannot find FPGA UIO device file for " << device_name_ << std::endl;
                            throw std::runtime_error("UIO device not found");
                        }
                }
        }
    else
        {
            tracking_fpga_sc_sptr_->set_channel(channel_, device_io_name);
        }
}


void BaseDllPllTrackingFpga::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_fpga_sc_sptr_->set_gnss_synchro(p_gnss_synchro);
}
