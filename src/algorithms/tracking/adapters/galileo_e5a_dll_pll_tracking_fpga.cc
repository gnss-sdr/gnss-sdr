/*!
 * \file galileo_e5a_dll_pll_tracking_fpga.cc
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E5a signals for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
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

#include "galileo_e5a_dll_pll_tracking_fpga.h"
#include "Galileo_E5a.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf_fpga.h"
#include "galileo_e5_signal_replica.h"
#include "gnss_sdr_flags.h"
#include "uio_fpga.h"
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <algorithm>
#include <array>

GalileoE5aDllPllTrackingFpga::GalileoE5aDllPllTrackingFpga(
    const ConfigurationInterface *configuration,
    const std::string &role,
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

    const auto vector_length = static_cast<int32_t>(std::round(trk_params_fpga.fs_in / (GALILEO_E5A_CODE_CHIP_RATE_CPS / GALILEO_E5A_CODE_LENGTH_CHIPS)));
    trk_params_fpga.vector_length = vector_length;
    track_pilot_ = trk_params_fpga.track_pilot;
    if (trk_params_fpga.extend_correlation_symbols < 1)
        {
            trk_params_fpga.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E5a. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << '\n';
        }
    else if (!trk_params_fpga.track_pilot and trk_params_fpga.extend_correlation_symbols > GALILEO_E5A_I_SECONDARY_CODE_LENGTH)
        {
            trk_params_fpga.extend_correlation_symbols = GALILEO_E5A_I_SECONDARY_CODE_LENGTH;
            std::cout << TEXT_RED << "WARNING: Galileo E5a. extend_correlation_symbols must be lower than 21 when tracking the data component. Coherent integration has been set to 20 symbols (20 ms)" << TEXT_RESET << '\n';
        }
    if ((trk_params_fpga.extend_correlation_symbols > 1) and (trk_params_fpga.pll_bw_narrow_hz > trk_params_fpga.pll_bw_hz or trk_params_fpga.dll_bw_narrow_hz > trk_params_fpga.dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: Galileo E5a. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << '\n';
        }
    trk_params_fpga.system = 'E';
    const std::array<char, 3> sig{'5', 'X', '\0'};
    std::copy_n(sig.data(), 3, trk_params_fpga.signal);

    // UIO device file
    device_name_ = configuration->property(role_ + ".devicename", default_device_name_Galileo_E5a);

    // compute the number of tracking channels that have already been instantiated. The order in which
    // GNSS-SDR instantiates the tracking channels i L1, L2, L5, E1, E5a
    // However E5a can use the same tracking HW accelerators as L5 (but not simultaneously).
    // Therefore for the proper assignment of the FPGA tracking device file numbers to the E5a tracking channels,
    // the number of channels that have already been assigned to L5 must not be substracted to this channel number,
    // so they are not counted here.

    uint32_t num_prev_assigned_ch_1C = configuration->property("Channels_1C.count", 0);
    uint32_t num_prev_assigned_ch_2S = 0;
    if (configuration->property("Tracking_2S.devicename", std::string("")) != device_name_)
        {
            num_prev_assigned_ch_2S = configuration->property("Channels_2S.count", 0);
        }
    uint32_t num_prev_assigned_ch_1B = configuration->property("Channels_1B.count", 0);
    num_prev_assigned_ch_ = num_prev_assigned_ch_1C + num_prev_assigned_ch_2S + num_prev_assigned_ch_1B;

    // ################# PRE-COMPUTE ALL THE CODES #################
    uint32_t code_samples_per_chip = 1;
    auto code_length_chips = static_cast<uint32_t>(GALILEO_E5A_CODE_LENGTH_CHIPS);

    volk_gnsssdr::vector<gr_complex> aux_code(code_length_chips * code_samples_per_chip, gr_complex(0.0, 0.0));

    prn_codes_ptr_ = static_cast<int32_t *>(volk_gnsssdr_malloc(static_cast<int32_t>(code_length_chips) * code_samples_per_chip * GALILEO_E5A_NUMBER_OF_CODES * sizeof(int32_t), volk_gnsssdr_get_alignment()));

    if (trk_params_fpga.track_pilot)
        {
            data_codes_ptr_ = static_cast<int32_t *>(volk_gnsssdr_malloc((static_cast<uint32_t>(code_length_chips)) * code_samples_per_chip * GALILEO_E5A_NUMBER_OF_CODES * sizeof(int32_t), volk_gnsssdr_get_alignment()));
        }

    for (uint32_t PRN = 1; PRN <= GALILEO_E5A_NUMBER_OF_CODES; PRN++)
        {
            const std::array<char, 3> sig_a = {'5', 'X', '\0'};
            galileo_e5_a_code_gen_complex_primary(aux_code, PRN, sig_a);

            if (trk_params_fpga.track_pilot)
                {
                    // The code is generated as a series of 1s and -1s. In order to store the values using only one bit, a -1 is stored as a 0 in the FPGA
                    for (uint32_t s = 0; s < code_length_chips; s++)
                        {
                            auto tmp_value = static_cast<int32_t>(aux_code[s].imag());
                            if (tmp_value < 0)
                                {
                                    tmp_value = 0;
                                }
                            tmp_value = tmp_value | LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            prn_codes_ptr_[static_cast<int32_t>(code_length_chips) * (PRN - 1) + s] = tmp_value;

                            tmp_value = static_cast<int32_t>(aux_code[s].real());
                            if (tmp_value < 0)
                                {
                                    tmp_value = 0;
                                }
                            tmp_value = tmp_value | LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY | LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
                            data_codes_ptr_[static_cast<int32_t>(code_length_chips) * (PRN - 1) + s] = tmp_value;
                        }
                }
            else
                {
                    // The code is generated as a series of 1s and -1s. In order to store the values using only one bit, a -1 is stored as a 0 in the FPGA
                    for (uint32_t s = 0; s < code_length_chips; s++)
                        {
                            auto tmp_value = static_cast<int32_t>(aux_code[s].real());
                            if (tmp_value < 0)
                                {
                                    tmp_value = 0;
                                }
                            tmp_value = tmp_value | LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            prn_codes_ptr_[static_cast<int32_t>(code_length_chips) * (PRN - 1) + s] = tmp_value;
                        }
                }
        }

    trk_params_fpga.ca_codes = prn_codes_ptr_;
    trk_params_fpga.data_codes = data_codes_ptr_;
    trk_params_fpga.code_length_chips = code_length_chips;
    trk_params_fpga.code_samples_per_chip = code_samples_per_chip;  // 2 sample per chip

    trk_params_fpga.extended_correlation_in_fpga = false;  // by default
    trk_params_fpga.extend_fpga_integration_periods = 1;   // (number of FPGA integrations that are combined in the SW)
    trk_params_fpga.fpga_integration_period = 1;           // (number of symbols that are effectively integrated in the FPGA)
    if (track_pilot_)
        {
            if (trk_params_fpga.extend_correlation_symbols > 1)
                {
                    if (trk_params_fpga.extend_correlation_symbols <= GALILEO_E5A_I_SECONDARY_CODE_LENGTH)
                        {
                            if ((GALILEO_E5A_I_SECONDARY_CODE_LENGTH % trk_params_fpga.extend_correlation_symbols) == 0)
                                {
                                    trk_params_fpga.extended_correlation_in_fpga = true;
                                    trk_params_fpga.fpga_integration_period = trk_params_fpga.extend_correlation_symbols;
                                }
                        }
                    else
                        {
                            if (trk_params_fpga.extend_correlation_symbols % GALILEO_E5A_I_SECONDARY_CODE_LENGTH == 0)
                                {
                                    trk_params_fpga.extended_correlation_in_fpga = true;
                                    trk_params_fpga.extend_fpga_integration_periods = trk_params_fpga.extend_correlation_symbols / GALILEO_E5A_I_SECONDARY_CODE_LENGTH;
                                    trk_params_fpga.fpga_integration_period = GALILEO_E5A_I_SECONDARY_CODE_LENGTH;
                                }
                        }
                }
        }

    // ################# MAKE TRACKING GNURadio object ###################
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


GalileoE5aDllPllTrackingFpga::~GalileoE5aDllPllTrackingFpga()
{
    volk_gnsssdr_free(prn_codes_ptr_);
    if (track_pilot_)
        {
            volk_gnsssdr_free(data_codes_ptr_);
        }
}


void GalileoE5aDllPllTrackingFpga::start_tracking()
{
    tracking_fpga_sc_sptr_->start_tracking();
}


void GalileoE5aDllPllTrackingFpga::stop_tracking()
{
    tracking_fpga_sc_sptr_->stop_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GalileoE5aDllPllTrackingFpga::set_channel(unsigned int channel)
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


void GalileoE5aDllPllTrackingFpga::set_gnss_synchro(Gnss_Synchro *p_gnss_synchro)
{
    tracking_fpga_sc_sptr_->set_gnss_synchro(p_gnss_synchro);
}


void GalileoE5aDllPllTrackingFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void GalileoE5aDllPllTrackingFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GalileoE5aDllPllTrackingFpga::get_left_block()
{
    return tracking_fpga_sc_sptr_;
}


gr::basic_block_sptr GalileoE5aDllPllTrackingFpga::get_right_block()
{
    return tracking_fpga_sc_sptr_;
}
