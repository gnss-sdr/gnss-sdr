/*!
 * \file gps_l1_ca_dll_pll_tracking_fpga.cc
 * \brief Implementation of an adapter of a DLL+PLL tracking loop block
 * for GPS L1 C/A to a TrackingInterface for the FPGA
 * \author Marc Majoral, 2019, mmajoral(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
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

#include "gps_l1_ca_dll_pll_tracking_fpga.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf_fpga.h"
#include "gnss_sdr_flags.h"
#include "gps_sdr_signal_replica.h"
#include "uio_fpga.h"
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>
#include <array>

GpsL1CaDllPllTrackingFpga::GpsL1CaDllPllTrackingFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(role),
      num_prev_assigned_ch_(0),
      channel_(0),
      in_streams_(in_streams),
      out_streams_(out_streams)
{
    Dll_Pll_Conf_Fpga trk_params_fpga = Dll_Pll_Conf_Fpga();
    trk_params_fpga.SetFromConfiguration(configuration, role_);

    const auto vector_length = static_cast<int32_t>(std::round(trk_params_fpga.fs_in / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));
    trk_params_fpga.vector_length = vector_length;
    if (trk_params_fpga.extend_correlation_symbols < 1)
        {
            trk_params_fpga.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A. extend_correlation_symbols must be bigger than 1. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << '\n';
        }
    else if (trk_params_fpga.extend_correlation_symbols > GPS_CA_BIT_DURATION_MS)
        {
            trk_params_fpga.extend_correlation_symbols = GPS_CA_BIT_DURATION_MS;
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A. extend_correlation_symbols must be lower than 21. Coherent integration has been set to 20 symbols (20 ms)" << TEXT_RESET << '\n';
        }
    trk_params_fpga.track_pilot = configuration->property(role_ + ".track_pilot", false);
    if (trk_params_fpga.track_pilot)
        {
            trk_params_fpga.track_pilot = false;
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A does not have pilot signal. Data tracking has been enabled" << TEXT_RESET << '\n';
        }
    if ((trk_params_fpga.extend_correlation_symbols > 1) and (trk_params_fpga.pll_bw_narrow_hz > trk_params_fpga.pll_bw_hz or trk_params_fpga.dll_bw_narrow_hz > trk_params_fpga.dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << '\n';
        }
    trk_params_fpga.system = 'G';
    const std::array<char, 3> sig{'1', 'C', '\0'};
    std::copy_n(sig.data(), 3, trk_params_fpga.signal);

    // UIO device file
    device_name_ = configuration->property(role_ + ".devicename", default_device_name_GPS_L1);

    // ################# PRE-COMPUTE ALL THE CODES #################
    ca_codes_ptr_ = static_cast<int32_t*>(volk_gnsssdr_malloc(static_cast<int32_t>(GPS_L1_CA_CODE_LENGTH_CHIPS * NUM_PRNs) * sizeof(int32_t), volk_gnsssdr_get_alignment()));
    for (uint32_t PRN = 1; PRN <= NUM_PRNs; PRN++)
        {
            gps_l1_ca_code_gen_int(own::span<int32_t>(&ca_codes_ptr_[static_cast<int32_t>(GPS_L1_CA_CODE_LENGTH_CHIPS) * (PRN - 1)], &ca_codes_ptr_[static_cast<int32_t>(GPS_L1_CA_CODE_LENGTH_CHIPS) * (PRN)]), PRN, 0);

            // The code is generated as a series of 1s and -1s. In order to store the values using only one bit, a -1 is stored as a 0 in the FPGA
            for (uint32_t k = 0; k < GPS_L1_CA_CODE_LENGTH_CHIPS; k++)
                {
                    int32_t tmp_value = ca_codes_ptr_[(int32_t(GPS_L1_CA_CODE_LENGTH_CHIPS)) * (PRN - 1) + k];
                    if (tmp_value < 0)
                        {
                            tmp_value = 0;
                        }
                    tmp_value = tmp_value | LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                    ca_codes_ptr_[(int32_t(GPS_L1_CA_CODE_LENGTH_CHIPS)) * (PRN - 1) + k] = tmp_value;
                }
        }
    trk_params_fpga.ca_codes = ca_codes_ptr_;
    trk_params_fpga.code_length_chips = GPS_L1_CA_CODE_LENGTH_CHIPS;
    trk_params_fpga.code_samples_per_chip = 1;  // 1 sample per chip

    trk_params_fpga.extended_correlation_in_fpga = false;  // by default
    trk_params_fpga.extend_fpga_integration_periods = 1;   // (number of FPGA integrations that are combined in the SW)
    trk_params_fpga.fpga_integration_period = 1;           // (number of symbols that are effectively integrated in the FPGA)
    if (trk_params_fpga.extend_correlation_symbols > 1)
        {
            if (trk_params_fpga.extend_correlation_symbols <= GPS_CA_BIT_DURATION_MS)
                {
                    if ((GPS_CA_BIT_DURATION_MS % trk_params_fpga.extend_correlation_symbols) == 0)
                        {
                            trk_params_fpga.extended_correlation_in_fpga = true;
                            trk_params_fpga.fpga_integration_period = trk_params_fpga.extend_correlation_symbols;
                        }
                }
        }

    // ################# MAKE TRACKING GNU Radio object ###################
    DLOG(INFO) << "role " << role_;
    tracking_fpga_sc_sptr_ = dll_pll_veml_make_tracking_fpga(trk_params_fpga);
    DLOG(INFO) << "tracking(" << tracking_fpga_sc_sptr_->unique_id() << ")";

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


GpsL1CaDllPllTrackingFpga::~GpsL1CaDllPllTrackingFpga()
{
    volk_gnsssdr_free(ca_codes_ptr_);
}


void GpsL1CaDllPllTrackingFpga::start_tracking()
{
    tracking_fpga_sc_sptr_->start_tracking();
}


void GpsL1CaDllPllTrackingFpga::stop_tracking()
{
    tracking_fpga_sc_sptr_->stop_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GpsL1CaDllPllTrackingFpga::set_channel(unsigned int channel)
{
    channel_ = channel;

    // UIO device file
    std::string device_io_name;

    // find the uio device file corresponding to the tracking multicorrelator
    if (find_uio_dev_file_name(device_io_name, device_name_, channel_ - num_prev_assigned_ch_) < 0)
        {
            bool alt_device_found = false;  // alternative compatible HW accelerator device not found by default

            // If the HW accelerator is the default one in the L1 band then look for an alternative hardware accelerator
            if (device_name_ == default_device_name_GPS_L1)
                {
                    if (find_uio_dev_file_name(device_io_name, default_device_name_Galileo_E1, channel_ - num_prev_assigned_ch_) < 0)
                        {
                            std::cout << "Cannot find the FPGA uio device file corresponding to device names " << device_name_ << " or " << default_device_name_Galileo_E1 << std::endl;
                            throw std::exception();
                        }
                    else
                        {
                            alt_device_found = true;  // alternative compatible HW accelerator device has been found
                        }
                }

            if (!alt_device_found)
                {
                    std::cout << "Cannot find the FPGA uio device file corresponding to device name " << device_name_ << std::endl;
                    throw std::exception();
                }
        }

    tracking_fpga_sc_sptr_->set_channel(channel_, device_io_name);
}


void GpsL1CaDllPllTrackingFpga::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_fpga_sc_sptr_->set_gnss_synchro(p_gnss_synchro);
}


void GpsL1CaDllPllTrackingFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect
}


void GpsL1CaDllPllTrackingFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect
}


gr::basic_block_sptr GpsL1CaDllPllTrackingFpga::get_left_block()
{
    return tracking_fpga_sc_sptr_;
}


gr::basic_block_sptr GpsL1CaDllPllTrackingFpga::get_right_block()
{
    return tracking_fpga_sc_sptr_;
}
