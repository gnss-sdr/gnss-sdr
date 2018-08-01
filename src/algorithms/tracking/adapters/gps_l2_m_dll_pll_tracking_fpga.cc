/*!
 * \file gps_l2_m_dll_pll_tracking.cc
 * \brief Implementation of an adapter of a DLL+PLL tracking loop block
 * for GPS L1 C/A to a TrackingInterface
 * \author Javier Arribas, 2015. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "gps_l2_m_dll_pll_tracking_fpga.h"
#include "configuration_interface.h"
#include "GPS_L2C.h"
#include "gps_l2c_signal.h"
#include "gnss_sdr_flags.h"
#include "display.h"
#include <glog/logging.h>

#define NUM_PRNs 32

using google::LogMessage;

GpsL2MDllPllTrackingFpga::GpsL2MDllPllTrackingFpga(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    //dllpllconf_t trk_param;
    dllpllconf_fpga_t trk_param_fpga;
    DLOG(INFO) << "role " << role;
    //################# CONFIGURATION PARAMETERS ########################
    //std::string default_item_type = "gr_complex";
    //std::string item_type = configuration->property(role + ".item_type", default_item_type);
    int fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    int fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    trk_param_fpga.fs_in = fs_in;
    bool dump = configuration->property(role + ".dump", false);
    trk_param_fpga.dump = dump;
    float pll_bw_hz = configuration->property(role + ".pll_bw_hz", 2.0);
    if (FLAGS_pll_bw_hz != 0.0) pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
    trk_param_fpga.pll_bw_hz = pll_bw_hz;
    float dll_bw_hz = configuration->property(role + ".dll_bw_hz", 0.75);
    if (FLAGS_dll_bw_hz != 0.0) dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
    trk_param_fpga.dll_bw_hz = dll_bw_hz;
    float early_late_space_chips = configuration->property(role + ".early_late_space_chips", 0.5);
    trk_param_fpga.early_late_space_chips = early_late_space_chips;
    trk_param_fpga.early_late_space_narrow_chips = 0.0;
    std::string default_dump_filename = "./track_ch";
    std::string dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
    trk_param_fpga.dump_filename = dump_filename;
    int vector_length = std::round(static_cast<double>(fs_in) / (static_cast<double>(GPS_L2_M_CODE_RATE_HZ) / static_cast<double>(GPS_L2_M_CODE_LENGTH_CHIPS)));
    trk_param_fpga.vector_length = vector_length;
    int symbols_extended_correlator = configuration->property(role + ".extend_correlation_symbols", 1);
    if (symbols_extended_correlator != 1)
        {
            std::cout << TEXT_RED << "WARNING: Extended coherent integration is not allowed in GPS L2. Coherent integration has been set to 20 ms (1 symbol)" << TEXT_RESET << std::endl;
        }
    trk_param_fpga.extend_correlation_symbols = 1;
    bool track_pilot = configuration->property(role + ".track_pilot", false);
    if (track_pilot)
        {
            std::cout << TEXT_RED << "WARNING: GPS L2 does not have pilot signal. Data tracking has been enabled" << TEXT_RESET << std::endl;
        }
    trk_param_fpga.track_pilot = false;
    trk_param_fpga.very_early_late_space_chips = 0.0;
    trk_param_fpga.very_early_late_space_narrow_chips = 0.0;
    trk_param_fpga.pll_bw_narrow_hz = 0.0;
    trk_param_fpga.dll_bw_narrow_hz = 0.0;
    trk_param_fpga.system = 'G';
    char sig_[3] = "2S";
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
    //unsigned int multicorr_type = configuration->property(role + ".multicorr_type", 0);
    trk_param_fpga.multicorr_type = 0; //multicorr_type : 0 -> 3 correlators, 1 -> 5 correlators

    //d_tracking_code = static_cast<float *>(volk_gnsssdr_malloc(2 * static_cast<unsigned int>(GPS_L2_M_CODE_LENGTH_CHIPS) * sizeof(float), volk_gnsssdr_get_alignment()));
    d_ca_codes = static_cast<int*>(volk_gnsssdr_malloc(static_cast<unsigned int>(GPS_L2_M_CODE_LENGTH_CHIPS)* NUM_PRNs * sizeof(int), volk_gnsssdr_get_alignment()));
    float* ca_codes_f = static_cast<float*>(volk_gnsssdr_malloc(static_cast<unsigned int>(GPS_L2_M_CODE_LENGTH_CHIPS)* sizeof(float), volk_gnsssdr_get_alignment()));

    //################# PRE-COMPUTE ALL THE CODES #################
    d_ca_codes = static_cast<int*>(volk_gnsssdr_malloc(static_cast<int>(GPS_L2_M_CODE_LENGTH_CHIPS*NUM_PRNs) * sizeof(int), volk_gnsssdr_get_alignment()));
    for (unsigned int PRN = 1; PRN <= NUM_PRNs; PRN++)
    {
        //gps_l1_ca_code_gen_int(&d_ca_codes[(int(GPS_L1_CA_CODE_LENGTH_CHIPS)) * (PRN - 1)], PRN, 0);
        gps_l2c_m_code_gen_float(ca_codes_f, PRN);
        for (unsigned int s = 0; s < 2*static_cast<unsigned int>(GPS_L2_M_CODE_LENGTH_CHIPS); s++)
            {
                d_ca_codes[static_cast<int>(GPS_L2_M_CODE_LENGTH_CHIPS)* (PRN - 1) + s] = static_cast<int>(ca_codes_f[s]);
            }
    }

    delete[] ca_codes_f;

    trk_param_fpga.ca_codes = d_ca_codes;
    trk_param_fpga.code_length_chips = GPS_L2_M_CODE_LENGTH_CHIPS;
    trk_param_fpga.code_samples_per_chip = 1; // 1 sample per chip

    //################# MAKE TRACKING GNURadio object ###################

//    //################# MAKE TRACKING GNURadio object ###################
//    if (item_type.compare("gr_complex") == 0)
//        {
//            item_size_ = sizeof(gr_complex);
//            tracking_ = dll_pll_veml_make_tracking(trk_param);
//        }
//    else
//        {
//            item_size_ = sizeof(gr_complex);
//            LOG(WARNING) << item_type << " unknown tracking item type.";
//        }

    //################# MAKE TRACKING GNURadio object ###################
    tracking_fpga_sc = dll_pll_veml_make_tracking_fpga(trk_param_fpga);

    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_fpga_sc->unique_id() << ")";
}


GpsL2MDllPllTrackingFpga::~GpsL2MDllPllTrackingFpga()
{
}


void GpsL2MDllPllTrackingFpga::start_tracking()
{
    //tracking_->start_tracking();
    tracking_fpga_sc->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GpsL2MDllPllTrackingFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    //tracking_->set_channel(channel);
    tracking_fpga_sc->set_channel(channel);
}


void GpsL2MDllPllTrackingFpga::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    //tracking_->set_gnss_synchro(p_gnss_synchro);
    tracking_fpga_sc->set_gnss_synchro(p_gnss_synchro);
}


void GpsL2MDllPllTrackingFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to connect, now the tracking uses gr_sync_decimator
}


void GpsL2MDllPllTrackingFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GpsL2MDllPllTrackingFpga::get_left_block()
{
    //return tracking_;
    return tracking_fpga_sc;
}


gr::basic_block_sptr GpsL2MDllPllTrackingFpga::get_right_block()
{
    //return tracking_;
    return tracking_fpga_sc;
}
