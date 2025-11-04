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
      num_prev_assigned_ch_(0),
      in_streams_(in_streams),
      out_streams_(out_streams)
{
    trk_params_ = Dll_Pll_Conf_Fpga();
    trk_params_.SetFromConfiguration(configuration, role_);
    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
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


void BaseDllPllTrackingFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    std::string device_io_name;

    if (find_uio_dev_file_name(device_io_name, device_name_, channel_ - num_prev_assigned_ch_) < 0)
        {
            if (!find_alternative_device(device_io_name))
                {
                    std::cout << "Cannot find FPGA UIO device file for " << device_name_ << std::endl;
                    throw std::runtime_error("UIO device not found");
                }
        }

    tracking_fpga_sc_sptr_->set_channel(channel_, device_io_name);
}


void BaseDllPllTrackingFpga::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_fpga_sc_sptr_->set_gnss_synchro(p_gnss_synchro);
}
