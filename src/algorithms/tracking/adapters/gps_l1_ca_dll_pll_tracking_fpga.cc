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

#include "gps_l1_ca_dll_pll_tracking_fpga.h"
#include "GPS_L1_CA.h"
#include "configuration_interface.h"
#include "display.h"
#include "dll_pll_conf_fpga.h"
#include "gnss_sdr_flags.h"
#include "gps_sdr_signal_processing.h"
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <array>

GpsL1CaDllPllTrackingFpga::GpsL1CaDllPllTrackingFpga(
    ConfigurationInterface* configuration, const std::string& role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    Dll_Pll_Conf_Fpga trk_params_fpga = Dll_Pll_Conf_Fpga();
    DLOG(INFO) << "role " << role;
    trk_params_fpga.SetFromConfiguration(configuration, role);

    int32_t vector_length = std::round(trk_params_fpga.fs_in / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS));
    trk_params_fpga.vector_length = vector_length;
    if (trk_params_fpga.extend_correlation_symbols < 1)
        {
            trk_params_fpga.extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A. extend_correlation_symbols must be bigger than 1. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
        }
    else if (trk_params_fpga.extend_correlation_symbols > GPS_CA_BIT_DURATION_MS)
        {
            trk_params_fpga.extend_correlation_symbols = GPS_CA_BIT_DURATION_MS;
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A. extend_correlation_symbols must be lower than 21. Coherent integration has been set to 20 symbols (20 ms)" << TEXT_RESET << std::endl;
        }
    trk_params_fpga.track_pilot = configuration->property(role + ".track_pilot", false);
    if (trk_params_fpga.track_pilot)
        {
            trk_params_fpga.track_pilot = false;
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A does not have pilot signal. Data tracking has been enabled" << TEXT_RESET << std::endl;
        }
    if ((trk_params_fpga.extend_correlation_symbols > 1) and (trk_params_fpga.pll_bw_narrow_hz > trk_params_fpga.pll_bw_hz or trk_params_fpga.dll_bw_narrow_hz > trk_params_fpga.dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << std::endl;
        }
    trk_params_fpga.system = 'G';
    std::array<char, 3> sig_{'1', 'C', '\0'};
    std::memcpy(trk_params_fpga.signal, sig_.data(), 3);

    // FPGA configuration parameters
    // obtain the number of the first uio device corresponding to a HW accelerator in the FPGA
    // that can be assigned to the tracking of the L1 signal
    trk_params_fpga.dev_file_num = configuration->property(role + ".dev_file_num", 3);
    // compute the number of tracking channels that have already been instantiated. The order in which
    // GNSS-SDR instantiates the tracking channels i L1, l2, L5, E1, E5a
    trk_params_fpga.num_prev_assigned_ch = 0;

    // ################# PRE-COMPUTE ALL THE CODES #################
    d_ca_codes = static_cast<int32_t*>(volk_gnsssdr_malloc(static_cast<int32_t>(GPS_L1_CA_CODE_LENGTH_CHIPS * NUM_PRNs) * sizeof(int32_t), volk_gnsssdr_get_alignment()));
    for (uint32_t PRN = 1; PRN <= NUM_PRNs; PRN++)
        {
            gps_l1_ca_code_gen_int(gsl::span<int32_t>(&d_ca_codes[static_cast<int32_t>(GPS_L1_CA_CODE_LENGTH_CHIPS) * (PRN - 1)], &d_ca_codes[static_cast<int32_t>(GPS_L1_CA_CODE_LENGTH_CHIPS) * (PRN)]), PRN, 0);

            // The code is generated as a series of 1s and -1s. In order to store the values using only one bit, a -1 is stored as a 0 in the FPGA
            for (uint32_t k = 0; k < GPS_L1_CA_CODE_LENGTH_CHIPS; k++)
                {
                    int32_t tmp_value = d_ca_codes[(int32_t(GPS_L1_CA_CODE_LENGTH_CHIPS)) * (PRN - 1) + k];
                    if (tmp_value < 0)
                        {
                            tmp_value = 0;
                        }
                    tmp_value = tmp_value | LOCAL_CODE_FPGA_ENABLE_WRITE_MEMORY;
                    d_ca_codes[(int32_t(GPS_L1_CA_CODE_LENGTH_CHIPS)) * (PRN - 1) + k] = tmp_value;
                }
        }
    trk_params_fpga.ca_codes = d_ca_codes;
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


GpsL1CaDllPllTrackingFpga::~GpsL1CaDllPllTrackingFpga()
{
    volk_gnsssdr_free(d_ca_codes);
}


void GpsL1CaDllPllTrackingFpga::start_tracking()
{
    tracking_fpga_sc->start_tracking();
}


void GpsL1CaDllPllTrackingFpga::stop_tracking()
{
    tracking_fpga_sc->stop_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GpsL1CaDllPllTrackingFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_fpga_sc->set_channel(channel);
}


void GpsL1CaDllPllTrackingFpga::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    tracking_fpga_sc->set_gnss_synchro(p_gnss_synchro);
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
    return tracking_fpga_sc;
}


gr::basic_block_sptr GpsL1CaDllPllTrackingFpga::get_right_block()
{
    return tracking_fpga_sc;
}
