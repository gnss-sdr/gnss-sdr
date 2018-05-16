/*!
 * \file gps_l1_ca_dll_pll_tracking.cc
 * \brief Implementation of an adapter of a DLL+PLL tracking loop block
 * for GPS L1 C/A to a TrackingInterface that uses the FPGA
 * \author Marc Majoral, 2018, mmajoral(at)cttc.es
 *         Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
#include "configuration_interface.h"
#include "display.h"
#include "gnss_sdr_flags.h"
#include "GPS_L1_CA.h"
#include "gps_sdr_signal_processing.h"
#include <glog/logging.h>


#define NUM_PRNs 32

using google::LogMessage;

GpsL1CaDllPllTrackingFpga::GpsL1CaDllPllTrackingFpga(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    dllpllconf_fpga_t trk_param_fpga;
    DLOG(INFO) << "role " << role;

    //################# CONFIGURATION PARAMETERS ########################
    int fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    int fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    trk_param_fpga.fs_in = fs_in;
    bool dump = configuration->property(role + ".dump", false);
    trk_param_fpga.dump = dump;
    float pll_bw_hz = configuration->property(role + ".pll_bw_hz", 50.0);
    if (FLAGS_pll_bw_hz != 0.0) pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
    trk_param_fpga.pll_bw_hz = pll_bw_hz;
    float pll_bw_narrow_hz = configuration->property(role + ".pll_bw_narrow_hz", 20.0);
    trk_param_fpga.pll_bw_narrow_hz = pll_bw_narrow_hz;
    float dll_bw_narrow_hz = configuration->property(role + ".dll_bw_narrow_hz", 2.0);
    trk_param_fpga.dll_bw_narrow_hz = dll_bw_narrow_hz;
    float dll_bw_hz = configuration->property(role + ".dll_bw_hz", 2.0);
    if (FLAGS_dll_bw_hz != 0.0) dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
    trk_param_fpga.dll_bw_hz = dll_bw_hz;
    float early_late_space_chips = configuration->property(role + ".early_late_space_chips", 0.5);
    trk_param_fpga.early_late_space_chips = early_late_space_chips;
    float early_late_space_narrow_chips = configuration->property(role + ".early_late_space_narrow_chips", 0.5);
    trk_param_fpga.early_late_space_narrow_chips = early_late_space_narrow_chips;
    std::string default_dump_filename = "./track_ch";
    std::string dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
    trk_param_fpga.dump_filename = dump_filename;
    int vector_length = std::round(fs_in / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS));
    trk_param_fpga.vector_length = vector_length;
    int symbols_extended_correlator = configuration->property(role + ".extend_correlation_symbols", 1);
    if (symbols_extended_correlator < 1)
        {
            symbols_extended_correlator = 1;
            std::cout << TEXT_RED << "WARNING: GPS L1 C/A. extend_correlation_symbols must be bigger than 1. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
        }
    else if (symbols_extended_correlator > 20)
        {
            symbols_extended_correlator = 20;
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
    char sig_[3] = "1C";
    std::memcpy(trk_param_fpga.signal, sig_, 3);
    int cn0_samples = configuration->property(role + ".cn0_samples", 20);
    if (FLAGS_cn0_samples != 20) cn0_samples = FLAGS_cn0_samples;
    trk_param_fpga.cn0_samples = cn0_samples;
    int cn0_min = configuration->property(role + ".cn0_min", 25);
    if (FLAGS_cn0_min != 25) cn0_min = FLAGS_cn0_min;
    trk_param_fpga.cn0_min = cn0_min;
    int max_lock_fail = configuration->property(role + ".max_lock_fail", 50);
    if (FLAGS_max_lock_fail != 50) max_lock_fail = FLAGS_max_lock_fail;
    trk_param_fpga.max_lock_fail = max_lock_fail;
    double carrier_lock_th = configuration->property(role + ".carrier_lock_th", 0.85);
    if (FLAGS_carrier_lock_th != 0.85) carrier_lock_th = FLAGS_carrier_lock_th;
    trk_param_fpga.carrier_lock_th = carrier_lock_th;

    // FPGA configuration parameters
    std::string default_device_name = "/dev/uio";
    std::string device_name = configuration->property(role + ".devicename", default_device_name);
    trk_param_fpga.device_name = device_name;
    unsigned int device_base = configuration->property(role + ".device_base", 1);
    trk_param_fpga.device_base = device_base;

    //################# PRE-COMPUTE ALL THE CODES #################
    d_ca_codes = static_cast<int*>(volk_gnsssdr_malloc(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS * NUM_PRNs) * sizeof(int), volk_gnsssdr_get_alignment()));
    for (unsigned int PRN = 1; PRN <= NUM_PRNs; PRN++)
        {
            gps_l1_ca_code_gen_int(&d_ca_codes[(int(GPS_L1_CA_CODE_LENGTH_CHIPS)) * (PRN - 1)], PRN, 0);
        }
    trk_param_fpga.ca_codes = d_ca_codes;
    trk_param_fpga.code_length = GPS_L1_CA_CODE_LENGTH_CHIPS;

    //################# MAKE TRACKING GNURadio object ###################
    tracking_fpga_sc = dll_pll_veml_make_tracking_fpga(trk_param_fpga);
    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_fpga_sc->unique_id() << ")";
}


GpsL1CaDllPllTrackingFpga::~GpsL1CaDllPllTrackingFpga()
{
    delete[] d_ca_codes;
}


void GpsL1CaDllPllTrackingFpga::start_tracking()
{
    tracking_fpga_sc->start_tracking();
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
    //nothing to connect
}


void GpsL1CaDllPllTrackingFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to disconnect
}


gr::basic_block_sptr GpsL1CaDllPllTrackingFpga::get_left_block()
{
    return tracking_fpga_sc;
}


gr::basic_block_sptr GpsL1CaDllPllTrackingFpga::get_right_block()
{
    return tracking_fpga_sc;
}
