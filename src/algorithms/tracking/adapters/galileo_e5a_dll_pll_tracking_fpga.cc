/*!
 * \file galileo_e5a_dll_pll_tracking.cc
 * \brief Adapts a code DLL + carrier PLL
 *  tracking block to a TrackingInterface for Galileo E5a signals
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E5a data and pilot Signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          </ul>
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

#include "galileo_e5a_dll_pll_tracking_fpga.h"
#include "configuration_interface.h"
#include "Galileo_E5a.h"
#include "galileo_e5_signal_processing.h"
#include "gnss_sdr_flags.h"
#include "display.h"
#include <glog/logging.h>

using google::LogMessage;

GalileoE5aDllPllTrackingFpga::GalileoE5aDllPllTrackingFpga(
    ConfigurationInterface* configuration, std::string role,
    unsigned int in_streams, unsigned int out_streams) : role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    dllpllconf_fpga_t trk_param_fpga;
    DLOG(INFO) << "role " << role;
    //################# CONFIGURATION PARAMETERS ########################
    std::string default_item_type = "gr_complex";
    std::string item_type = configuration->property(role + ".item_type", default_item_type);
    int fs_in_deprecated = configuration->property("GNSS-SDR.internal_fs_hz", 12000000);
    int fs_in = configuration->property("GNSS-SDR.internal_fs_sps", fs_in_deprecated);
    trk_param_fpga.fs_in = fs_in;
    bool dump = configuration->property(role + ".dump", false);
    trk_param_fpga.dump = dump;
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
    std::string default_dump_filename = "./track_ch";
    std::string dump_filename = configuration->property(role + ".dump_filename", default_dump_filename);
    trk_param_fpga.dump_filename = dump_filename;
    int vector_length = std::round(fs_in / (Galileo_E5a_CODE_CHIP_RATE_HZ / Galileo_E5a_CODE_LENGTH_CHIPS));
    trk_param_fpga.vector_length = vector_length;
    int extend_correlation_symbols = configuration->property(role + ".extend_correlation_symbols", 1);
    float early_late_space_narrow_chips = configuration->property(role + ".early_late_space_narrow_chips", 0.15);
    trk_param_fpga.early_late_space_narrow_chips = early_late_space_narrow_chips;
    bool track_pilot = configuration->property(role + ".track_pilot", false);
    d_track_pilot = track_pilot;
    if (extend_correlation_symbols < 1)
        {
            extend_correlation_symbols = 1;
            std::cout << TEXT_RED << "WARNING: Galileo E5a. extend_correlation_symbols must be bigger than 0. Coherent integration has been set to 1 symbol (1 ms)" << TEXT_RESET << std::endl;
        }
    else if (!track_pilot and extend_correlation_symbols > Galileo_E5a_I_SECONDARY_CODE_LENGTH)
        {
            extend_correlation_symbols = Galileo_E5a_I_SECONDARY_CODE_LENGTH;
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
    trk_param_fpga.multicorr_type = 1; // 0 -> 3 correlators, 1 -> 5 correlators

    //################# PRE-COMPUTE ALL THE CODES #################
    unsigned int code_samples_per_chip = 1;
    unsigned int code_length_chips = static_cast<unsigned int>(Galileo_E5a_CODE_LENGTH_CHIPS);

    gr_complex *aux_code = static_cast<gr_complex *>(volk_gnsssdr_malloc(sizeof(gr_complex) * code_length_chips * code_samples_per_chip, volk_gnsssdr_get_alignment()));

    float *tracking_code;
    float *data_code;

    if (trk_param_fpga.track_pilot)
        {
            data_code = static_cast<float *>(volk_gnsssdr_malloc(code_samples_per_chip * code_length_chips * sizeof(float), volk_gnsssdr_get_alignment()));
        }
    tracking_code = static_cast<float *>(volk_gnsssdr_malloc(code_samples_per_chip * code_length_chips * sizeof(float), volk_gnsssdr_get_alignment()));

    d_ca_codes = static_cast<int*>(volk_gnsssdr_malloc(static_cast<int>(code_length_chips)* code_samples_per_chip * Galileo_E5a_NUMBER_OF_CODES * sizeof(int), volk_gnsssdr_get_alignment()));

    if (trk_param_fpga.track_pilot)
        {
            d_data_codes = static_cast<int *>(volk_gnsssdr_malloc((static_cast<unsigned int>(code_length_chips)) * code_samples_per_chip * Galileo_E5a_NUMBER_OF_CODES * sizeof(int), volk_gnsssdr_get_alignment()));
        }


    for (unsigned int PRN = 1; PRN <= Galileo_E5a_NUMBER_OF_CODES; PRN++)
        {



        //galileo_e5_a_code_gen_complex_primary(aux_code, PRN, const_cast<char *>(trk_param_fpga.signal.c_str()));
        galileo_e5_a_code_gen_complex_primary(aux_code, PRN, const_cast<char *>(sig_));
        if (trk_param_fpga.track_pilot)
            {
                //d_secondary_code_string = const_cast<std::string *>(&Galileo_E5a_Q_SECONDARY_CODE[PRN - 1]);
                for (unsigned int i = 0; i < code_length_chips; i++)
                    {
                        tracking_code[i] = aux_code[i].imag();
                        data_code[i] = aux_code[i].real();
                    }
                for (unsigned int s = 0; s < code_length_chips; s++)
                    {
                        d_ca_codes[static_cast<int>(code_length_chips) * (PRN - 1) + s] = static_cast<int>(tracking_code[s]);
                        d_data_codes[static_cast<int>(code_length_chips) * (PRN - 1) + s] = static_cast<int>(data_code[s]);
                        //printf("%f %d | ", data_codes_f[s], d_data_codes[static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS)* 2 * (PRN - 1) + s]);

                    }

            }
        else
            {
                for (unsigned int i = 0; i < code_length_chips; i++)
                    {
                        tracking_code[i] = aux_code[i].real();
                    }

                for (unsigned int s = 0; s < code_length_chips; s++)
                    {
                        d_ca_codes[static_cast<int>(code_length_chips)* (PRN - 1) + s] = static_cast<int>(tracking_code[s]);
                        //printf("%f %d | ", ca_codes_f[s], d_ca_codes[static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS)* 2 * (PRN - 1) + s]);
                    }

            }


        }

    volk_gnsssdr_free(aux_code);
    volk_gnsssdr_free(tracking_code);
    if (trk_param_fpga.track_pilot)
        {
            volk_gnsssdr_free(data_code);
        }
    trk_param_fpga.ca_codes = d_ca_codes;
    trk_param_fpga.data_codes = d_data_codes;
    trk_param_fpga.code_length_chips = code_length_chips;
    trk_param_fpga.code_samples_per_chip = code_samples_per_chip; // 2 sample per chip
    //################# MAKE TRACKING GNURadio object ###################
//    if (item_type.compare("gr_complex") == 0)
//        {
//            item_size_ = sizeof(gr_complex);
//            tracking_ = dll_pll_veml_make_tracking(trk_param_fpga);
//        }
//    else
//        {
//            item_size_ = sizeof(gr_complex);
//            LOG(WARNING) << item_type << " unknown tracking item type.";
//        }
    channel_ = 0;
    //DLOG(INFO) << "tracking(" << tracking_->unique_id() << ")";
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
    //tracking_->start_tracking();
    tracking_fpga_sc->start_tracking();
}


/*
 * Set tracking channel unique ID
 */
void GalileoE5aDllPllTrackingFpga::set_channel(unsigned int channel)
{
    channel_ = channel;
    //tracking_->set_channel(channel);
    tracking_fpga_sc->set_channel(channel);
}


void GalileoE5aDllPllTrackingFpga::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    //tracking_->set_gnss_synchro(p_gnss_synchro);
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
    //return tracking_;
    return tracking_fpga_sc;
}


gr::basic_block_sptr GalileoE5aDllPllTrackingFpga::get_right_block()
{
    //return tracking_;
    return tracking_fpga_sc;
}
