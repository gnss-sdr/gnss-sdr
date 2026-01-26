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
      num_channels_({{"1C", configuration->property("Channels_1C.count", 0)},
          {"2S", configuration->property("Channels_2S.count", 0)},
          {"L5", configuration->property("Channels_L5.count", 0)},
          {"1B", configuration->property("Channels_1B.count", 0)}}),
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
    // Compute the base channel index for signal_ based on the channel initialization order in the receiver and the number of channels assigned to each signal.
    uint32_t signal_base_channel_index = 0;
    for (const auto& label : channel_ordering)
        {
            if (label != signal_)
                {
                    auto it = num_channels_.find(label);
                    if (it != num_channels_.end())
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
                                    signal_base_channel_index += it->second;
                                }
                        }
                }
            else
                {
                    break;
                }
        }

    // Check for tracking multicorrelators assigned as a fallback
    uint32_t num_fallback_devices = get_num_fallback_devices();
    signal_base_channel_index -= num_fallback_devices;

    // Update signal_base_channel_index_
    signal_base_channel_index_ = signal_base_channel_index;
}

uint32_t BaseDllPllTrackingFpga::get_num_fallback_devices()
{
    // Return the number of FPGA tracking multicorrelator devices mapped to signal_ that are also assigned as fallback for other signals.

    // Get the default FPGA hardware correlator device name for signal_
    std::string default_device_name;
    auto device_name_it = signal_to_device_.find(signal_);
    if (device_name_it != signal_to_device_.end())
        {
            default_device_name = device_name_it->second;
        }

    // Check if this FPGA hardware correlator name is assigned as fallback to another signal
    bool fallback_assignment = false;
    std::string fallback_signal_type;
    for (const auto& [signal_type, fallback_device_name] : signal_to_fallback_device_)
        {
            if (fallback_device_name == default_device_name)
                {
                    fallback_signal_type = signal_type;
                    fallback_assignment = true;
                    break;  // only one match is possible
                }
        }

    if (fallback_assignment)
        {
            // Get the number of channels for the signal that uses this FPGA hardware correlator name as a fallback
            uint32_t num_channels = 0;
            auto num_channels_it = num_channels_.find(fallback_signal_type);
            if (num_channels_it != num_channels_.end())
                {
                    num_channels = num_channels_it->second;
                }

            // Get the default FPGA device name for the signal that uses this hardware correlator name as a fallback
            std::string device_name;
            auto device_it = signal_to_device_.find(fallback_signal_type);
            if (device_it != signal_to_device_.end())
                {
                    device_name = device_it->second;
                }

            // Get the number of devices available by default for the signal that uses this hardware correlator name as a fallback
            uint32_t num_devices_available = get_num_devices(device_name);

            // return the actual number of FPGA tracking multicorrelator devices used as fallback
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
            // If insufficient FPGA tracking multicorrelators are available for a GNSS signal, check whether compatible multicorrelators from another signal can be used as fallback.
            auto it = signal_to_fallback_device_.find(signal_);
            if (it != signal_to_fallback_device_.end())
                {
                    uint32_t alternate_device_channel_index = channel_ - signal_base_channel_index_ - get_num_devices(it->second);  // channel number within the fallback tracking multicorrelators.
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
