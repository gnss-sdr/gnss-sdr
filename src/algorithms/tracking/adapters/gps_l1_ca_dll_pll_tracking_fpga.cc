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
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
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
    Dll_Pll_Conf_Fpga trk_param_fpga = Dll_Pll_Conf_Fpga();
    DLOG(INFO) << "role " << role;
    // ################# CONFIGURATION PARAMETERS ########################
    int32_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    int32_t fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    trk_param_fpga.fs_in = fs_in;
    trk_param_fpga.high_dyn = configuration->property(role + ".high_dyn", false);
    if (configuration->property(role + ".smoother_length", 10) < 1)
        {
            trk_param_fpga.smoother_length = 1;
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A. smoother_length must be bigger than 0. It has been set to 1" << TEXT_RESET << std::endl;
        }
    else
        {
            trk_param_fpga.smoother_length = configuration->property(role + ".smoother_length", 10);
        }
    bool dump = configuration->property(role + ".dump", false);
    trk_param_fpga.dump = dump;
    std::string default_dump_filename = "./track_ch";
    std::string dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
    trk_param_fpga.dump_filename = dump_filename;
    bool dump_mat = configuration->property(role + ".dump_mat", true);
    trk_param_fpga.dump_mat = dump_mat;
    float pll_bw_hz = configuration->property(role + ".pll_bw_hz", 50.0);
    if (FLAGS_pll_bw_hz != 0.0)
        {
            pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
        }
    trk_param_fpga.pll_bw_hz = pll_bw_hz;
    float pll_bw_narrow_hz = configuration->property(role + ".pll_bw_narrow_hz", 20.0);
    trk_param_fpga.pll_bw_narrow_hz = pll_bw_narrow_hz;
    float dll_bw_narrow_hz = configuration->property(role + ".dll_bw_narrow_hz", 2.0);
    trk_param_fpga.dll_bw_narrow_hz = dll_bw_narrow_hz;
    float dll_bw_hz = configuration->property(role + ".dll_bw_hz", 2.0);
    if (FLAGS_dll_bw_hz != 0.0)
        {
            dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
        }
    trk_param_fpga.dll_bw_hz = dll_bw_hz;

    int dll_filter_order = configuration->property(role + ".dll_filter_order", 2);
    if (dll_filter_order < 1)
        {
            LOG(WARNING) << "dll_filter_order parameter must be 1, 2 or 3. Set to 1.";
            dll_filter_order = 1;
        }
    if (dll_filter_order > 3)
        {
            LOG(WARNING) << "dll_filter_order parameter must be 1, 2 or 3. Set to 3.";
            dll_filter_order = 3;
        }
    trk_param_fpga.dll_filter_order = dll_filter_order;

    int pll_filter_order = configuration->property(role + ".pll_filter_order", 3);
    if (pll_filter_order < 2)
        {
            LOG(WARNING) << "pll_filter_order parameter must be 2 or 3. Set to 2.";
            pll_filter_order = 2;
        }
    if (pll_filter_order > 3)
        {
            LOG(WARNING) << "pll_filter_order parameter must be 2 or 3. Set to 3.";
            pll_filter_order = 3;
        }
    trk_param_fpga.pll_filter_order = pll_filter_order;

    if (pll_filter_order == 2)
        {
            trk_param_fpga.fll_filter_order = 1;
        }
    if (pll_filter_order == 3)
        {
            trk_param_fpga.fll_filter_order = 2;
        }

    bool enable_fll_pull_in = configuration->property(role + ".enable_fll_pull_in", false);
    trk_param_fpga.enable_fll_pull_in = enable_fll_pull_in;
    float fll_bw_hz = configuration->property(role + ".fll_bw_hz", 35.0);
    trk_param_fpga.fll_bw_hz = fll_bw_hz;
    float pull_in_time_s = configuration->property(role + ".pull_in_time_s", 2.0);
    trk_param_fpga.pull_in_time_s = pull_in_time_s;

    float early_late_space_chips = configuration->property(role + ".early_late_space_chips", 0.5);
    trk_param_fpga.early_late_space_chips = early_late_space_chips;
    float early_late_space_narrow_chips = configuration->property(role + ".early_late_space_narrow_chips", 0.5);
    trk_param_fpga.early_late_space_narrow_chips = early_late_space_narrow_chips;
    int32_t vector_length = std::round(fs_in / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));
    trk_param_fpga.vector_length = vector_length;
    int32_t symbols_extended_correlator = configuration->property(role + ".extend_correlation_symbols", 1);
    if (symbols_extended_correlator < 1)
        {
            symbols_extended_correlator = 1;
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A. extend_correlation_symbols must be bigger than 1. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
        }
    else if (symbols_extended_correlator > GPS_CA_BIT_DURATION_MS)
        {
            symbols_extended_correlator = GPS_CA_BIT_DURATION_MS;
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A. extend_correlation_symbols must be lower than 21. Coherent integration has been set to 20 symbols (20 ms)" << TEXT_RESET << std::endl;
        }
    trk_param_fpga.extend_correlation_symbols = symbols_extended_correlator;
    bool track_pilot = configuration->property(role + ".track_pilot", false);
    if (track_pilot)
        {
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A does not have pilot signal. Data tracking has been enabled" << TEXT_RESET << std::endl;
        }
    if ((symbols_extended_correlator > 1) and (pll_bw_narrow_hz > pll_bw_hz or dll_bw_narrow_hz > dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << std::endl;
        }
    trk_param_fpga.very_early_late_space_chips = 0.0;
    trk_param_fpga.very_early_late_space_narrow_chips = 0.0;
    trk_param_fpga.track_pilot = false;
    trk_param_fpga.system = 'G';
    std::array<char, 3> sig_{'1', 'C', '\0'};
    std::memcpy(trk_param_fpga.signal, sig_.data(), 3);
    trk_param_fpga.cn0_samples = configuration->property(role + ".cn0_samples", trk_param_fpga.cn0_samples);
    trk_param_fpga.cn0_min = configuration->property(role + ".cn0_min", trk_param_fpga.cn0_min);
    trk_param_fpga.max_code_lock_fail = configuration->property(role + ".max_lock_fail", trk_param_fpga.max_code_lock_fail);
    trk_param_fpga.max_carrier_lock_fail = configuration->property(role + ".max_carrier_lock_fail", trk_param_fpga.max_carrier_lock_fail);
    trk_param_fpga.carrier_lock_th = configuration->property(role + ".carrier_lock_th", trk_param_fpga.carrier_lock_th);

    // FPGA configuration parameters
    std::string default_device_name = "/dev/uio";
    std::string device_name = configuration->property(role + ".devicename", default_device_name);
    trk_param_fpga.device_name = device_name;
    uint32_t device_base = configuration->property(role + ".device_base", 3);
    trk_param_fpga.device_base = device_base;

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
    trk_param_fpga.ca_codes = d_ca_codes;
    trk_param_fpga.code_length_chips = GPS_L1_CA_CODE_LENGTH_CHIPS;
    trk_param_fpga.code_samples_per_chip = 1;  // 1 sample per chip

    trk_param_fpga.extended_correlation_in_fpga = false;  // by default
    trk_param_fpga.extend_fpga_integration_periods = 1;   // (number of FPGA integrations that are combined in the SW)
    trk_param_fpga.fpga_integration_period = 1;           // (number of symbols that are effectively integrated in the FPGA)
    if (symbols_extended_correlator > 1)
        {
            if (symbols_extended_correlator <= GPS_CA_BIT_DURATION_MS)
                {
                    if ((GPS_CA_BIT_DURATION_MS % symbols_extended_correlator) == 0)
                        {
                            trk_param_fpga.extended_correlation_in_fpga = true;
                            trk_param_fpga.fpga_integration_period = symbols_extended_correlator;
                        }
                }
        }

    // ################# MAKE TRACKING GNURadio object ###################
    tracking_fpga_sc = dll_pll_veml_make_tracking_fpga(trk_param_fpga);
    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_fpga_sc->unique_id() << ")";
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
