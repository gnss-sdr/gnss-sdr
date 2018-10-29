/*!
 * \file galileo_e1_dll_pll_veml_tracking.cc
 * \brief  Adapts a DLL+PLL VEML (Very Early Minus Late) tracking loop block
 *   to a TrackingInterface for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#include "galileo_e1_dll_pll_veml_tracking_fpga.h"
#include "configuration_interface.h"
#include "Galileo_E1.h"
#include "galileo_e1_signal_processing.h"
#include "gnss_sdr_flags.h"
#include "display.h"
#include <glog/logging.h>

//#define NUM_PRNs_GALILEO_E1 50

using google::LogMessage;

void GalileoE1DllPllVemlTrackingFpga::stop_tracking()
{
}

GalileoE1DllPllVemlTrackingFpga::GalileoE1DllPllVemlTrackingFpga(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    //dllpllconf_t trk_param;
    Dll_Pll_Conf_Fpga trk_param_fpga = Dll_Pll_Conf_Fpga();
    DLOG(INFO) << "role " << role;
    //################# CONFIGURATION PARAMETERS ########################
    std::string default_item_type = "gr_complex";
    std::string item_type = configuration->property(role + ".item_type", default_item_type);
    int fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 2048000);
    int fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    trk_param_fpga.fs_in = fs_in;
    bool dump = configuration->property(role + ".dump", false);
    trk_param_fpga.dump = dump;
    std::string default_dump_filename = "./track_ch";
    std::string dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
    trk_param_fpga.dump_filename = dump_filename;
    bool dump_mat = configuration->property(role + ".dump_mat", true);
    trk_param_fpga.dump_mat = dump_mat;
    float pll_bw_hz = configuration->property(role + ".pll_bw_hz", 5.0);
    if (FLAGS_pll_bw_hz != 0.0) pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
    trk_param_fpga.pll_bw_hz = pll_bw_hz;
    float dll_bw_hz = configuration->property(role + ".dll_bw_hz", 0.5);
    if (FLAGS_dll_bw_hz != 0.0) dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
    trk_param_fpga.dll_bw_hz = dll_bw_hz;
    float pll_bw_narrow_hz = configuration->property(role + ".pll_bw_narrow_hz", 2.0);
    trk_param_fpga.pll_bw_narrow_hz = pll_bw_narrow_hz;
    float dll_bw_narrow_hz = configuration->property(role + ".dll_bw_narrow_hz", 0.25);
    trk_param_fpga.dll_bw_narrow_hz = dll_bw_narrow_hz;
    int extend_correlation_symbols = configuration->property(role + ".extend_correlation_symbols", 1);
    float early_late_space_chips = configuration->property(role + ".early_late_space_chips", 0.15);
    trk_param_fpga.early_late_space_chips = early_late_space_chips;
    float very_early_late_space_chips = configuration->property(role + ".very_early_late_space_chips", 0.6);
    trk_param_fpga.very_early_late_space_chips = very_early_late_space_chips;
    float early_late_space_narrow_chips = configuration->property(role + ".early_late_space_narrow_chips", 0.15);
    trk_param_fpga.early_late_space_narrow_chips = early_late_space_narrow_chips;
    float very_early_late_space_narrow_chips = configuration->property(role + ".very_early_late_space_narrow_chips", 0.6);
    trk_param_fpga.very_early_late_space_narrow_chips = very_early_late_space_narrow_chips;
    bool track_pilot = configuration->property(role + ".track_pilot", false);
    if (extend_correlation_symbols < 1)
        {
            extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E1. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (4 ms)" << TEXT_RESET << std::endl;
        }
    else if (!track_pilot and extend_correlation_symbols > 1)
        {
            extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E1. Extended coherent integration is not allowed when tracking the data component. Coherent integration has been set to 4 ms (1 symbol)" << TEXT_RESET << std::endl;
        }
    if ((extend_correlation_symbols > 1) and (pll_bw_narrow_hz > pll_bw_hz or dll_bw_narrow_hz > dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: Galileo E1. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << std::endl;
        }
    trk_param_fpga.track_pilot = track_pilot;
    d_track_pilot = track_pilot;
    trk_param_fpga.extend_correlation_symbols = extend_correlation_symbols;
    int vector_length = std::round(fs_in / (Galileo_E1_CODE_CHIP_RATE_HZ / Galileo_E1_B_CODE_LENGTH_CHIPS));
    trk_param_fpga.vector_length = vector_length;
    trk_param_fpga.system = 'E';
    char sig_[3] = "1B";
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
    //unsigned int multicorr_type = configuration->property(role + ".multicorr_type", 1);
    trk_param_fpga.multicorr_type = 1;  // 0 -> 3 correlators, 1 -> 5 correlators

    //################# PRE-COMPUTE ALL THE CODES #################
    unsigned int code_samples_per_chip = 2;
    d_ca_codes = static_cast<int*>(volk_gnsssdr_malloc(static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS) * code_samples_per_chip * Galileo_E1_NUMBER_OF_CODES * sizeof(int), volk_gnsssdr_get_alignment()));
    float* ca_codes_f;
    float* data_codes_f;

    if (trk_param_fpga.track_pilot)
        {
            d_data_codes = static_cast<int*>(volk_gnsssdr_malloc((static_cast<unsigned int>(Galileo_E1_B_CODE_LENGTH_CHIPS)) * code_samples_per_chip * Galileo_E1_NUMBER_OF_CODES * sizeof(int), volk_gnsssdr_get_alignment()));
        }
    ca_codes_f = static_cast<float*>(volk_gnsssdr_malloc(static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS) * code_samples_per_chip * sizeof(float), volk_gnsssdr_get_alignment()));

    if (trk_param_fpga.track_pilot)
        {
            data_codes_f = static_cast<float*>(volk_gnsssdr_malloc((static_cast<unsigned int>(Galileo_E1_B_CODE_LENGTH_CHIPS)) * code_samples_per_chip * sizeof(float), volk_gnsssdr_get_alignment()));
        }

    //printf("pppppppp trk_param_fpga.track_pilot = %d\n", trk_param_fpga.track_pilot);

    //int kk;

    for (unsigned int PRN = 1; PRN <= Galileo_E1_NUMBER_OF_CODES; PRN++)
        {
            char data_signal[3] = "1B";
            if (trk_param_fpga.track_pilot)
                {
                    //printf("yes tracking pilot !!!!!!!!!!!!!!!\n");
                    char pilot_signal[3] = "1C";
                    galileo_e1_code_gen_sinboc11_float(ca_codes_f, pilot_signal, PRN);
                    galileo_e1_code_gen_sinboc11_float(data_codes_f, data_signal, PRN);

                    for (unsigned int s = 0; s < 2 * Galileo_E1_B_CODE_LENGTH_CHIPS; s++)
                        {
                            d_ca_codes[static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS) * 2 * (PRN - 1) + s] = static_cast<int>(ca_codes_f[s]);
                            d_data_codes[static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS) * 2 * (PRN - 1) + s] = static_cast<int>(data_codes_f[s]);
                            //printf("%f %d | ", data_codes_f[s], d_data_codes[static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS)* 2 * (PRN - 1) + s]);
                        }
                    //printf("\n next \n");
                    //scanf ("%d",&kk);
                }
            else
                {
                    //printf("no tracking pilot\n");
                    galileo_e1_code_gen_sinboc11_float(ca_codes_f, data_signal, PRN);

                    for (unsigned int s = 0; s < 2 * Galileo_E1_B_CODE_LENGTH_CHIPS; s++)
                        {
                            d_ca_codes[static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS) * 2 * (PRN - 1) + s] = static_cast<int>(ca_codes_f[s]);
                            //printf("%f %d | ", ca_codes_f[s], d_ca_codes[static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS)* 2 * (PRN - 1) + s]);
                        }
                    //printf("\n next \n");
                    //scanf ("%d",&kk);
                }
        }

    delete[] ca_codes_f;
    if (trk_param_fpga.track_pilot)
        {
            delete[] data_codes_f;
        }
    trk_param_fpga.ca_codes = d_ca_codes;
    trk_param_fpga.data_codes = d_data_codes;
    trk_param_fpga.code_length_chips = Galileo_E1_B_CODE_LENGTH_CHIPS;
    trk_param_fpga.code_samples_per_chip = code_samples_per_chip;  // 2 sample per chip
    //################# MAKE TRACKING GNURadio object ###################
    tracking_fpga_sc = dll_pll_veml_make_tracking_fpga(trk_param_fpga);
    channel_ = 0;
    DLOG(INFO) << "tracking(" << tracking_fpga_sc->unique_id() << ")";
}


GalileoE1DllPllVemlTrackingFpga::~GalileoE1DllPllVemlTrackingFpga()
{
    delete[] d_ca_codes;
    if (d_track_pilot)
        {
            delete[] d_data_codes;
        }
}

void GalileoE1DllPllVemlTrackingFpga::start_tracking()
{
    //tracking_->start_tracking();
    tracking_fpga_sc->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GalileoE1DllPllVemlTrackingFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    //tracking_->set_channel(channel);
    tracking_fpga_sc->set_channel(channel);
}


void GalileoE1DllPllVemlTrackingFpga::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    //tracking_->set_gnss_synchro(p_gnss_synchro);
    tracking_fpga_sc->set_gnss_synchro(p_gnss_synchro);
}


void GalileoE1DllPllVemlTrackingFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to connect, now the tracking uses gr_sync_decimator
}


void GalileoE1DllPllVemlTrackingFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GalileoE1DllPllVemlTrackingFpga::get_left_block()
{
    //return tracking_;
    return tracking_fpga_sc;
}


gr::basic_block_sptr GalileoE1DllPllVemlTrackingFpga::get_right_block()
{
    //return tracking_;
    return tracking_fpga_sc;
}
