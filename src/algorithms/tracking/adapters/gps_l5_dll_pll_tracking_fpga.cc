/*!
 * \file gps_l5_dll_pll_tracking_fpga.cc
 * \brief  Interface of an adapter of a DLL+PLL tracking loop block
 * for GPS L5 to a TrackingInterface for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
 *         Javier Arribas, 2019. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
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
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */


#include "gps_l5_dll_pll_tracking_fpga.h"
#include "GPS_L5.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf_fpga.h"
#include "gnss_sdr_flags.h"
#include "gps_l5_signal.h"
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr_alloc.h>
#include <array>

GpsL5DllPllTrackingFpga::GpsL5DllPllTrackingFpga(
    ConfigurationInterface *configuration, const std::string &role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    Dll_Pll_Conf_Fpga trk_params_fpga = Dll_Pll_Conf_Fpga();
    DLOG(INFO) << "role " << role;
    trk_params_fpga.SetFromConfiguration(configuration, role);

    int32_t vector_length = std::round(static_cast<double>(trk_params_fpga.fs_in) / (static_cast<double>(GPS_L5I_CODE_RATE_CPS) / static_cast<double>(GPS_L5I_CODE_LENGTH_CHIPS)));
    trk_params_fpga.vector_length = vector_length;
    if (trk_params_fpga.extend_correlation_symbols < 1)
        {
            trk_params_fpga.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: GPS L5. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
        }
    else if (!trk_params_fpga.track_pilot and trk_params_fpga.extend_correlation_symbols > GPS_L5I_NH_CODE_LENGTH)
        {
            trk_params_fpga.extend_correlation_symbols = GPS_L5I_NH_CODE_LENGTH;
            std::cout << TEXT_RED << "WARNING: GPS L5. extend_correlation_symbols must be lower than 11 when tracking the data component. Coherent integration has been set to 10 symbols (10 ms)" << TEXT_RESET << std::endl;
        }
    if ((trk_params_fpga.extend_correlation_symbols > 1) and (trk_params_fpga.pll_bw_narrow_hz > trk_params_fpga.pll_bw_hz or trk_params_fpga.dll_bw_narrow_hz > trk_params_fpga.dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: GPS L5. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << std::endl;
        }
    d_track_pilot = trk_params_fpga.track_pilot;
    trk_params_fpga.system = 'G';
    std::array<char, 3> sig_{'L', '5', '\0'};
    std::memcpy(trk_params_fpga.signal, sig_.data(), 3);

    // FPGA configuration parameters
    // obtain the number of the first uio device corresponding to a HW accelerator in the FPGA
    // that can be assigned to the tracking of the L5 signal
    trk_params_fpga.dev_file_num = configuration->property(role + ".dev_file_num", 27);
    // compute the number of tracking channels that have already been instantiated. The order in which
    // GNSS-SDR instantiates the tracking channels i L1, L2, L5, E1, E5a
    trk_params_fpga.num_prev_assigned_ch = configuration->property("Channels_1C.count", 0) +
                                           configuration->property("Channels_2S.count", 0);

    // ################# PRE-COMPUTE ALL THE CODES #################
    uint32_t code_samples_per_chip = 1;
    auto code_length_chips = static_cast<uint32_t>(GPS_L5I_CODE_LENGTH_CHIPS);

    volk_gnsssdr::vector<float> data_code;
    volk_gnsssdr::vector<float> tracking_code(code_length_chips, 0.0);

    if (d_track_pilot)
        {
            data_code.resize(code_length_chips, 0.0);
        }

    d_ca_codes = static_cast<int32_t *>(volk_gnsssdr_malloc(static_cast<int32_t>(code_length_chips * NUM_PRNs) * sizeof(int32_t), volk_gnsssdr_get_alignment()));

    d_data_codes = nullptr;
    if (d_track_pilot)
        {
            d_data_codes = static_cast<int32_t *>(volk_gnsssdr_malloc((static_cast<uint32_t>(code_length_chips)) * NUM_PRNs * sizeof(int32_t), volk_gnsssdr_get_alignment()));
        }

    for (uint32_t PRN = 1; PRN <= NUM_PRNs; PRN++)
        {
            if (d_track_pilot)
                {
                    gps_l5q_code_gen_float(tracking_code, PRN);
                    gps_l5i_code_gen_float(data_code, PRN);

                    // The code is generated as a series of 1s and -1s. In order to store the values using only one bit, a -1 is stored as a 0 in the FPGA
                    for (uint32_t s = 0; s < code_length_chips; s++)
                        {
                            auto tmp_value = static_cast<int32_t>(tracking_code[s]);
                            if (tmp_value < 0)
                                {
                                    tmp_value = 0;
                                }
                            tmp_value = tmp_value | LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            d_ca_codes[static_cast<int32_t>(code_length_chips) * (PRN - 1) + s] = tmp_value;

                            tmp_value = static_cast<int32_t>(data_code[s]);
                            if (tmp_value < 0)
                                {
                                    tmp_value = 0;
                                }
                            tmp_value = tmp_value | LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY | LOCAL_CODE_FPGA_CORRELATOR_SELECT_COUNT;
                            d_data_codes[static_cast<int32_t>(code_length_chips) * (PRN - 1) + s] = tmp_value;
                        }
                }
            else
                {
                    gps_l5i_code_gen_float(tracking_code, PRN);

                    // The code is generated as a series of 1s and -1s. In order to store the values using only one bit, a -1 is stored as a 0 in the FPGA
                    for (uint32_t s = 0; s < code_length_chips; s++)
                        {
                            auto tmp_value = static_cast<int32_t>(tracking_code[s]);
                            if (tmp_value < 0)
                                {
                                    tmp_value = 0;
                                }
                            tmp_value = tmp_value | LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                            d_ca_codes[static_cast<int32_t>(code_length_chips) * (PRN - 1) + s] = tmp_value;
                        }
                }
        }

    trk_params_fpga.ca_codes = d_ca_codes;
    trk_params_fpga.data_codes = d_data_codes;
    trk_params_fpga.code_length_chips = code_length_chips;
    trk_params_fpga.code_samples_per_chip = code_samples_per_chip;  // 2 sample per chip

    trk_params_fpga.extended_correlation_in_fpga = false;  // by default
    trk_params_fpga.extend_fpga_integration_periods = 1;   // (number of FPGA integrations that are combined in the SW)
    trk_params_fpga.fpga_integration_period = 1;           // (number of symbols that are effectively integrated in the FPGA)
    if (d_track_pilot)
        {
            if (trk_params_fpga.extend_correlation_symbols > 1)
                {
                    if (trk_params_fpga.extend_correlation_symbols <= GPS_L5I_NH_CODE_LENGTH)
                        {
                            if ((GPS_L5I_NH_CODE_LENGTH % trk_params_fpga.extend_correlation_symbols) == 0)
                                {
                                    trk_params_fpga.extended_correlation_in_fpga = true;
                                    trk_params_fpga.fpga_integration_period = trk_params_fpga.extend_correlation_symbols;
                                }
                        }
                    else
                        {
                            if (trk_params_fpga.extend_correlation_symbols % GPS_L5I_NH_CODE_LENGTH == 0)
                                {
                                    trk_params_fpga.extended_correlation_in_fpga = true;
                                    trk_params_fpga.extend_fpga_integration_periods = trk_params_fpga.extend_correlation_symbols / GPS_L5I_NH_CODE_LENGTH;
                                    trk_params_fpga.fpga_integration_period = GPS_L5I_NH_CODE_LENGTH;
                                }
                        }
                }
        }

    // ################# MAKE TRACKING GNU Radio object ###################
    tracking_fpga_sc = dll_pll_veml_make_tracking_fpga(trk_params_fpga);
    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_fpga_sc->unique_id() << ")";

    if (in_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one input stream";
        }
    if (out_streams_ > 1)
        {
            LOG(ERROR) << "This implementation only supports one output stream";
        }
}


GpsL5DllPllTrackingFpga::~GpsL5DllPllTrackingFpga()
{
    volk_gnsssdr_free(d_ca_codes);
    if (d_track_pilot)
        {
            volk_gnsssdr_free(d_data_codes);
        }
}


void GpsL5DllPllTrackingFpga::start_tracking()
{
    tracking_fpga_sc->start_tracking();
}


void GpsL5DllPllTrackingFpga::stop_tracking()
{
    tracking_fpga_sc->stop_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GpsL5DllPllTrackingFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_fpga_sc->set_channel(channel);
}


void GpsL5DllPllTrackingFpga::set_gnss_synchro(Gnss_Synchro *p_gnss_synchro)
{
    tracking_fpga_sc->set_gnss_synchro(p_gnss_synchro);
}


void GpsL5DllPllTrackingFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to connect, now the tracking uses gr_sync_decimator
}


void GpsL5DllPllTrackingFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    // nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GpsL5DllPllTrackingFpga::get_left_block()
{
    return tracking_fpga_sc;
}


gr::basic_block_sptr GpsL5DllPllTrackingFpga::get_right_block()
{
    return tracking_fpga_sc;
}
