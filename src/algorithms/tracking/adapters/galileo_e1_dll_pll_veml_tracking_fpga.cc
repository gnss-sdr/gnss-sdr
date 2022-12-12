/*!
 * \file galileo_e1_dll_pll_veml_tracking_fpga.cc
 * \brief  Adapts a DLL+PLL VEML (Very Early Minus Late) tracking loop block
 *   to a TrackingInterface for Galileo E1 signals for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
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

#include "galileo_e1_dll_pll_veml_tracking_fpga.h"
#include "Galileo_E1.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf_fpga.h"
#include "galileo_e1_signal_replica.h"
#include "gnss_sdr_flags.h"
#include "uio_fpga.h"
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>
#include <array>

GalileoE1DllPllVemlTrackingFpga::GalileoE1DllPllVemlTrackingFpga(
    const ConfigurationInterface* configuration,
    const std::string& role,
    unsigned int in_streams,
    unsigned int out_streams)
    : role_(role),
      data_codes_ptr_(nullptr),
      channel_(0),
      in_streams_(in_streams),
      out_streams_(out_streams)
{
    Dll_Pll_Conf_Fpga trk_params_fpga = Dll_Pll_Conf_Fpga();
    trk_params_fpga.SetFromConfiguration(configuration, role_);

    if (trk_params_fpga.extend_correlation_symbols < 1)
        {
            trk_params_fpga.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E1. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (4 ms)" << TEXT_RESET << '\n';
        }
    else if (!trk_params_fpga.track_pilot and trk_params_fpga.extend_correlation_symbols > 1)
        {
            trk_params_fpga.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E1. Extended coherent integration is not allowed when tracking the data component. Coherent integration has been set to 4 ms (1 symbol)" << TEXT_RESET << '\n';
        }
    if ((trk_params_fpga.extend_correlation_symbols > 1) and (trk_params_fpga.pll_bw_narrow_hz > trk_params_fpga.pll_bw_hz or trk_params_fpga.dll_bw_narrow_hz > trk_params_fpga.dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: Galileo E1. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << '\n';
        }
    track_pilot_ = trk_params_fpga.track_pilot;
    const auto vector_length = static_cast<int32_t>(std::round(trk_params_fpga.fs_in / (GALILEO_E1_CODE_CHIP_RATE_CPS / GALILEO_E1_B_CODE_LENGTH_CHIPS)));
    trk_params_fpga.vector_length = vector_length;
    trk_params_fpga.system = 'E';
    const std::array<char, 3> sig{'1', 'B', '\0'};
    std::copy_n(sig.data(), 3, trk_params_fpga.signal);

    // UIO device file
    device_name_ = configuration->property(role_ + ".devicename", default_device_name_Galileo_E1);

    // compute the number of tracking channels that have already been instantiated. The order in which
    // GNSS-SDR instantiates the tracking channels i L1, L2, L5, E1, E5a

    uint32_t num_prev_assigned_ch_1C = 0;
    std::string device_io_name;

    if (configuration->property("Tracking_1C.devicename", default_device_name_GPS_L1) == default_device_name_GPS_L1)
        {
            for (uint32_t k = 0; k < configuration->property("Channels_1C.count", 0U); k++)
                {
                    if (find_uio_dev_file_name(device_io_name, default_device_name_GPS_L1, k) == 0)
                        {
                            num_prev_assigned_ch_1C = num_prev_assigned_ch_1C + 1;
                        }
                }
        }
    else
        {
            if (configuration->property("Tracking_1C.devicename", std::string("")) != device_name_)
                {
                    num_prev_assigned_ch_1C = configuration->property("Channels_1C.count", 0);
                }
        }

    uint32_t num_prev_assigned_ch_2S = configuration->property("Channels_2S.count", 0);
    uint32_t num_prev_assigned_ch_L5 = configuration->property("Channels_L5.count", 0);
    num_prev_assigned_ch_ = num_prev_assigned_ch_1C + num_prev_assigned_ch_2S + num_prev_assigned_ch_L5;

    // ################# PRE-COMPUTE ALL THE CODES #################
    uint32_t code_samples_per_chip = 2;
    prn_codes_ptr_ = static_cast<int32_t*>(volk_gnsssdr_malloc(static_cast<int32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS) * code_samples_per_chip * GALILEO_E1_NUMBER_OF_CODES * sizeof(int32_t), volk_gnsssdr_get_alignment()));
    volk_gnsssdr::vector<float> ca_codes_f(static_cast<uint32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS) * code_samples_per_chip);
    volk_gnsssdr::vector<float> data_codes_f;

    if (track_pilot_)
        {
            data_codes_ptr_ = static_cast<int32_t*>(volk_gnsssdr_malloc((static_cast<uint32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS)) * code_samples_per_chip * GALILEO_E1_NUMBER_OF_CODES * sizeof(int32_t), volk_gnsssdr_get_alignment()));
            data_codes_f.resize(static_cast<uint32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS) * code_samples_per_chip, 0.0);
        }

    for (uint32_t PRN = 1; PRN <= GALILEO_E1_NUMBER_OF_CODES; PRN++)
        {
            std::array<char, 3> data_signal = {'1', 'B', '\0'};
            if (track_pilot_)
                {
                    std::array<char, 3> pilot_signal = {'1', 'C', '\0'};
                    galileo_e1_code_gen_sinboc11_float(ca_codes_f, pilot_signal, PRN);
                    galileo_e1_code_gen_sinboc11_float(data_codes_f, data_signal, PRN);

                    // The code is generated as a series of 1s and -1s. In order to store the values using only one bit, a -1 is stored as a 0 in the FPGA
                    for (uint32_t s = 0; s < 2 * GALILEO_E1_B_CODE_LENGTH_CHIPS; s++)
                        {
                            auto tmp_value = static_cast<int32_t>(ca_codes_f[s]);
                            if (tmp_value < 0)
                                {
                                    tmp_value = 0;
                                }
                            tmp_value = tmp_value | LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            prn_codes_ptr_[static_cast<int32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS) * 2 * (PRN - 1) + s] = tmp_value;
                            tmp_value = static_cast<int32_t>(data_codes_f[s]);
                            if (tmp_value < 0)
                                {
                                    tmp_value = 0;
                                }
                            tmp_value = tmp_value | LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY | LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
                            data_codes_ptr_[static_cast<int32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS) * 2 * (PRN - 1) + s] = tmp_value;
                        }
                }
            else
                {
                    galileo_e1_code_gen_sinboc11_float(ca_codes_f, data_signal, PRN);

                    // The code is generated as a series of 1s and -1s. In order to store the values using only one bit, a -1 is stored as a 0 in the FPGA
                    for (uint32_t s = 0; s < 2 * GALILEO_E1_B_CODE_LENGTH_CHIPS; s++)
                        {
                            auto tmp_value = static_cast<int32_t>(ca_codes_f[s]);
                            if (tmp_value < 0)
                                {
                                    tmp_value = 0;
                                }
                            tmp_value = tmp_value | LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            prn_codes_ptr_[static_cast<int32_t>(GALILEO_E1_B_CODE_LENGTH_CHIPS) * 2 * (PRN - 1) + s] = tmp_value;
                        }
                }
        }

    trk_params_fpga.ca_codes = prn_codes_ptr_;
    trk_params_fpga.data_codes = data_codes_ptr_;
    trk_params_fpga.code_length_chips = GALILEO_E1_B_CODE_LENGTH_CHIPS;
    trk_params_fpga.code_samples_per_chip = code_samples_per_chip;  // 2 sample per chip
    trk_params_fpga.extended_correlation_in_fpga = false;
    trk_params_fpga.extend_fpga_integration_periods = 1;  // (number of FPGA integrations that are combined in the SW)
    trk_params_fpga.fpga_integration_period = 1;          // (number of symbols that are effectively integrated in the FPGA)

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


GalileoE1DllPllVemlTrackingFpga::~GalileoE1DllPllVemlTrackingFpga()
{
    volk_gnsssdr_free(prn_codes_ptr_);
    if (track_pilot_)
        {
            volk_gnsssdr_free(data_codes_ptr_);
        }
}


void GalileoE1DllPllVemlTrackingFpga::stop_tracking()
{
    tracking_fpga_sc_sptr_->stop_tracking();
}


void GalileoE1DllPllVemlTrackingFpga::start_tracking()
{
    tracking_fpga_sc_sptr_->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GalileoE1DllPllVemlTrackingFpga::set_channel(unsigned int channel)
{
    channel_ = channel;

    // UIO device file
    std::string device_io_name;
    // find the uio device file corresponding to the tracking multicorrelator
    if (find_uio_dev_file_name(device_io_name, device_name_, channel_ - num_prev_assigned_ch_) < 0)
        {
            std::cout << "Cannot find the FPGA uio device file corresponding to device name " << device_name_ << std::endl;
            throw std::exception();
        }
    tracking_fpga_sc_sptr_->set_channel(channel_, device_io_name);
}


void GalileoE1DllPllVemlTrackingFpga::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_fpga_sc_sptr_->set_gnss_synchro(p_gnss_synchro);
}


void GalileoE1DllPllVemlTrackingFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void GalileoE1DllPllVemlTrackingFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GalileoE1DllPllVemlTrackingFpga::get_left_block()
{
    return tracking_fpga_sc_sptr_;
}


gr::basic_block_sptr GalileoE1DllPllVemlTrackingFpga::get_right_block()
{
    return tracking_fpga_sc_sptr_;
}
