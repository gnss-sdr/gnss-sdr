/*!
 * \file galileo_e5a_dll_pll_tracking_fpga.cc
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E5a signals for the FPGA
 * \author Marc Majoral, 2019. mmajoral(at)cttc.cat
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "galileo_e5a_dll_pll_tracking_fpga.h"
#include "Galileo_E5a.h"
#include "configuration_interface.h"
#include "display.h"
#include "galileo_e5_signal_processing.h"
#include "gnss_sdr_flags.h"
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr.h>

using google::LogMessage;


GalileoE5aDllPllTrackingFpga::GalileoE5aDllPllTrackingFpga(
    ConfigurationInterface *configuration, const std::string &role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    Dll_Pll_Conf_Fpga trk_param_fpga = Dll_Pll_Conf_Fpga();
    DLOG(INFO) << "role " << role;
    //################# CONFIGURATION PARAMETERS ########################
    std::string default_item_type = "gr_complex";
    std::string item_type = configuration->property(role + ".item_type", default_item_type);
    int32_t fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 12000000);
    int32_t fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    trk_param_fpga.fs_in = fs_in;
    bool dump = configuration->property(role + ".dump", false);
    trk_param_fpga.dump = dump;
    std::string default_dump_filename = "./track_ch";
    std::string dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
    trk_param_fpga.dump_filename = dump_filename;
    bool dump_mat = configuration->property(role + ".dump_mat", true);
    trk_param_fpga.dump_mat = dump_mat;
    float pll_bw_hz = configuration->property(role + ".pll_bw_hz", 20.0);
    if (FLAGS_pll_bw_hz != 0.0) pll_bw_hz = static_cast<float>(FLAGS_pll_bw_hz);
    trk_param_fpga.pll_bw_hz = pll_bw_hz;
    float dll_bw_hz = configuration->property(role + ".dll_bw_hz", 20.0);
    if (FLAGS_dll_bw_hz != 0.0) dll_bw_hz = static_cast<float>(FLAGS_dll_bw_hz);
    trk_param_fpga.dll_bw_hz = dll_bw_hz;
    float pll_bw_narrow_hz = configuration->property(role + ".pll_bw_narrow_hz", 5.0);
    trk_param_fpga.pll_bw_narrow_hz = pll_bw_narrow_hz;
    float dll_bw_narrow_hz = configuration->property(role + ".dll_bw_narrow_hz", 2.0);
    trk_param_fpga.dll_bw_narrow_hz = dll_bw_narrow_hz;
    float early_late_space_chips = configuration->property(role + ".early_late_space_chips", 0.5);
    trk_param_fpga.early_late_space_chips = early_late_space_chips;
    int32_t vector_length = std::round(fs_in / (GALILEO_E5A_CODE_CHIP_RATE_HZ / GALILEO_E5A_CODE_LENGTH_CHIPS));
    trk_param_fpga.vector_length = vector_length;
    int32_t extend_correlation_symbols = configuration->property(role + ".extend_correlation_symbols", 1);
    float early_late_space_narrow_chips = configuration->property(role + ".early_late_space_narrow_chips", 0.15);
    trk_param_fpga.early_late_space_narrow_chips = early_late_space_narrow_chips;
    bool track_pilot = configuration->property(role + ".track_pilot", false);
    d_track_pilot = track_pilot;
    if (extend_correlation_symbols < 1)
        {
            extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E5a. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
        }
    else if (!track_pilot and extend_correlation_symbols > GALILEO_E5A_I_SECONDARY_CODE_LENGTH)
        {
            extend_correlation_symbols = GALILEO_E5A_I_SECONDARY_CODE_LENGTH;
            std::cout << TEXT_RED << "WARNING: Galileo E5a. extend_correlation_symbols must be lower than 21 when tracking the data component. Coherent integration has been set to 20 symbols (20 ms)" << TEXT_RESET << std::endl;
        }
    if ((extend_correlation_symbols > 1) and (pll_bw_narrow_hz > pll_bw_hz or dll_bw_narrow_hz > dll_bw_hz))
        {
            std::cout << TEXT_RED << "WARNING: Galileo E5a. PLL or DLL narrow tracking bandwidth is higher than wide tracking one" << TEXT_RESET << std::endl;
        }
    trk_param_fpga.extend_correlation_symbols = extend_correlation_symbols;
    trk_param_fpga.track_pilot = track_pilot;
    trk_param_fpga.very_early_late_space_chips = 0.0;
    trk_param_fpga.very_early_late_space_narrow_chips = 0.0;
    trk_param_fpga.system = 'E';
    char sig_[3] = "5X";
    std::memcpy(trk_param_fpga.signal, sig_, 3);
    int32_t cn0_samples = configuration->property(role + ".cn0_samples", 20);
    if (FLAGS_cn0_samples != 20) cn0_samples = FLAGS_cn0_samples;
    trk_param_fpga.cn0_samples = cn0_samples;
    int32_t cn0_min = configuration->property(role + ".cn0_min", 25);
    if (FLAGS_cn0_min != 25) cn0_min = FLAGS_cn0_min;
    trk_param_fpga.cn0_min = cn0_min;
    int32_t max_lock_fail = configuration->property(role + ".max_lock_fail", 50);
    if (FLAGS_max_lock_fail != 50) max_lock_fail = FLAGS_max_lock_fail;
    trk_param_fpga.max_lock_fail = max_lock_fail;
    double carrier_lock_th = configuration->property(role + ".carrier_lock_th", 0.85);
    if (FLAGS_carrier_lock_th != 0.85) carrier_lock_th = FLAGS_carrier_lock_th;
    trk_param_fpga.carrier_lock_th = carrier_lock_th;
    d_data_codes = nullptr;

    // FPGA configuration parameters
    std::string default_device_name = "/dev/uio";
    std::string device_name = configuration->property(role + ".devicename", default_device_name);
    trk_param_fpga.device_name = device_name;
    uint32_t device_base = configuration->property(role + ".device_base", 27);
    trk_param_fpga.device_base = device_base;
    trk_param_fpga.multicorr_type = 1;  // 0 -> 3 correlators, 1 -> up to 5+1 correlators

    //################# PRE-COMPUTE ALL THE CODES #################
    uint32_t code_samples_per_chip = 1;
    auto code_length_chips = static_cast<uint32_t>(GALILEO_E5A_CODE_LENGTH_CHIPS);

    auto *aux_code = static_cast<gr_complex *>(volk_gnsssdr_malloc(sizeof(gr_complex) * code_length_chips * code_samples_per_chip, volk_gnsssdr_get_alignment()));

    d_ca_codes = static_cast<int32_t *>(volk_gnsssdr_malloc(static_cast<int32_t>(code_length_chips) * code_samples_per_chip * GALILEO_E5A_NUMBER_OF_CODES * sizeof(int32_t), volk_gnsssdr_get_alignment()));

    if (trk_param_fpga.track_pilot)
        {
            d_data_codes = static_cast<int32_t *>(volk_gnsssdr_malloc((static_cast<uint32_t>(code_length_chips)) * code_samples_per_chip * GALILEO_E5A_NUMBER_OF_CODES * sizeof(int32_t), volk_gnsssdr_get_alignment()));
        }

    for (uint32_t PRN = 1; PRN <= GALILEO_E5A_NUMBER_OF_CODES; PRN++)
        {
            galileo_e5_a_code_gen_complex_primary(aux_code, PRN, const_cast<char *>(sig_));
            if (trk_param_fpga.track_pilot)
                {
                    for (uint32_t s = 0; s < code_length_chips; s++)
                        {
                            d_ca_codes[static_cast<int32_t>(code_length_chips) * (PRN - 1) + s] = static_cast<int32_t>(aux_code[s].imag());
                            d_data_codes[static_cast<int32_t>(code_length_chips) * (PRN - 1) + s] = static_cast<int32_t>(aux_code[s].real());
                        }
                }
            else
                {
                    for (uint32_t s = 0; s < code_length_chips; s++)
                        {
                            d_ca_codes[static_cast<int32_t>(code_length_chips) * (PRN - 1) + s] = static_cast<int32_t>(aux_code[s].real());
                        }
                }
        }

    volk_gnsssdr_free(aux_code);
    trk_param_fpga.ca_codes = d_ca_codes;
    trk_param_fpga.data_codes = d_data_codes;
    trk_param_fpga.code_length_chips = code_length_chips;
    trk_param_fpga.code_samples_per_chip = code_samples_per_chip;  // 2 sample per chip
    //################# MAKE TRACKING GNURadio object ###################
    tracking_fpga_sc = dll_pll_veml_make_tracking_fpga(trk_param_fpga);
    channel_ = 0;

    DLOG(INFO) << "tracking(" << tracking_fpga_sc->unique_id() << ")";
}


GalileoE5aDllPllTrackingFpga::~GalileoE5aDllPllTrackingFpga()
{
    delete[] d_ca_codes;
    if (d_track_pilot)
        {
            delete[] d_data_codes;
        }
}


void GalileoE5aDllPllTrackingFpga::start_tracking()
{
    tracking_fpga_sc->start_tracking();
}


void GalileoE5aDllPllTrackingFpga::stop_tracking()
{
}


/*
 * Set tracking channel unique ID
 */
void GalileoE5aDllPllTrackingFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    tracking_fpga_sc->set_channel(channel);
}


void GalileoE5aDllPllTrackingFpga::set_gnss_synchro(Gnss_Synchro *p_gnss_synchro)
{
    tracking_fpga_sc->set_gnss_synchro(p_gnss_synchro);
}


void GalileoE5aDllPllTrackingFpga::connect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to connect, now the tracking uses gr_sync_decimator
}


void GalileoE5aDllPllTrackingFpga::disconnect(gr::top_block_sptr top_block)
{
    if (top_block)
        { /* top_block is not null */
        };
    //nothing to disconnect, now the tracking uses gr_sync_decimator
}


gr::basic_block_sptr GalileoE5aDllPllTrackingFpga::get_left_block()
{
    return tracking_fpga_sc;
}


gr::basic_block_sptr GalileoE5aDllPllTrackingFpga::get_right_block()
{
    return tracking_fpga_sc;
}
