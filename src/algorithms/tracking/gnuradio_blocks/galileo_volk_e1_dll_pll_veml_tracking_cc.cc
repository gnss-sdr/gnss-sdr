/*!
 * \file galileo_volk_e1_dll_pll_veml_tracking_cc.cc
 * \brief Implementation of a code DLL + carrier PLL VEML (Very Early
 *  Minus Late) tracking block for Galileo E1 signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * [1] K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
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

#include "galileo_volk_e1_dll_pll_veml_tracking_cc.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include "gnss_synchro.h"
#include "galileo_e1_signal_processing.h"
#include "tracking_discriminators.h"
#include "lock_detectors.h"
#include "Galileo_E1.h"
#include "control_message_factory.h"
#include "volk_gnsssdr/volk_gnsssdr.h"




/*!
 * \todo Include in definition header file
 */
#define CN0_ESTIMATION_SAMPLES 20
#define MINIMUM_VALID_CN0 25
#define MAXIMUM_LOCK_FAIL_COUNTER 50
#define CARRIER_LOCK_THRESHOLD 0.85


using google::LogMessage;

galileo_volk_e1_dll_pll_veml_tracking_cc_sptr
galileo_volk_e1_dll_pll_veml_make_tracking_cc(
                                         long if_freq,
                                         long fs_in,
                                         unsigned int vector_length,
                                         boost::shared_ptr<gr::msg_queue> queue,
                                         bool dump,
                                         std::string dump_filename,
                                         float pll_bw_hz,
                                         float dll_bw_hz,
                                         float early_late_space_chips,
                                         float very_early_late_space_chips)
{
    return galileo_volk_e1_dll_pll_veml_tracking_cc_sptr(new galileo_volk_e1_dll_pll_veml_tracking_cc(if_freq,
                                                                                            fs_in, vector_length, queue, dump, dump_filename, pll_bw_hz, dll_bw_hz, early_late_space_chips, very_early_late_space_chips));
}


void galileo_volk_e1_dll_pll_veml_tracking_cc::forecast (int noutput_items,
                                                    gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = static_cast<int>(d_vector_length) * 2; //set the required available samples in each call
}


galileo_volk_e1_dll_pll_veml_tracking_cc::galileo_volk_e1_dll_pll_veml_tracking_cc(
                                                                         long if_freq,
                                                                         long fs_in,
                                                                         unsigned int vector_length,
                                                                         boost::shared_ptr<gr::msg_queue> queue,
                                                                         bool dump,
                                                                         std::string dump_filename,
                                                                         float pll_bw_hz,
                                                                         float dll_bw_hz,
                                                                         float early_late_space_chips,
                                                                         float very_early_late_space_chips):
gr::block("galileo_volk_e1_dll_pll_veml_tracking_cc", gr::io_signature::make(1, 1, sizeof(gr_complex)),
          gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    this->set_relative_rate(1.0/vector_length);
    // initialize internal vars
    d_queue = queue;
    d_dump = dump;
    d_if_freq = if_freq;
    d_fs_in = fs_in;
    d_vector_length = vector_length;
    d_dump_filename = dump_filename;
    d_code_loop_filter = Tracking_2nd_DLL_filter(Galileo_E1_CODE_PERIOD);
    d_carrier_loop_filter = Tracking_2nd_PLL_filter(Galileo_E1_CODE_PERIOD);
    
    // Initialize tracking  ==========================================
    
    // Set bandwidth of code and carrier loop filters
    d_code_loop_filter.set_DLL_BW(dll_bw_hz);
    d_carrier_loop_filter.set_PLL_BW(pll_bw_hz);
    
    // Correlator spacing
    d_early_late_spc_chips = early_late_space_chips; // Define early-late offset (in chips)
    d_very_early_late_spc_chips = very_early_late_space_chips; // Define very-early-late offset (in chips)
    
    // Initialization of local code replica
    // Get space for a vector with the sinboc(1,1) replica sampled 2x/chip
    d_ca_code = static_cast<gr_complex*>(volk_malloc((2 * Galileo_E1_B_CODE_LENGTH_CHIPS + 4) * sizeof(gr_complex), volk_get_alignment()));
    
    d_very_early_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_early_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_prompt_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_late_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_very_late_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_carr_sign = static_cast<gr_complex*>(volk_malloc(2*d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    
    d_very_early_code16=static_cast<lv_16sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_16sc_t), volk_get_alignment()));
    d_early_code16=static_cast<lv_16sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_16sc_t), volk_get_alignment()));
    d_prompt_code16=static_cast<lv_16sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_16sc_t), volk_get_alignment()));
    d_late_code16=static_cast<lv_16sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_16sc_t), volk_get_alignment()));
    d_very_late_code16=static_cast<lv_16sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_16sc_t), volk_get_alignment()));
    d_carr_sign16=static_cast<lv_16sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_16sc_t), volk_get_alignment()));
    in16=static_cast<lv_16sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_16sc_t), volk_get_alignment()));
    
    d_very_early_code8=static_cast<lv_8sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_8sc_t), volk_get_alignment()));
    d_early_code8=static_cast<lv_8sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_8sc_t), volk_get_alignment()));
    d_prompt_code8=static_cast<lv_8sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_8sc_t), volk_get_alignment()));
    d_late_code8=static_cast<lv_8sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_8sc_t), volk_get_alignment()));
    d_very_late_code8=static_cast<lv_8sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_8sc_t), volk_get_alignment()));
    d_carr_sign8=static_cast<lv_8sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_8sc_t), volk_get_alignment()));
    in8=static_cast<lv_8sc_t*>(volk_malloc(2 * d_vector_length * sizeof(lv_8sc_t), volk_get_alignment()));
    
    // correlator outputs (scalar)
    d_Very_Early = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Early = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Late = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Very_Late = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    
    //--- Initializations ------------------------------
    // Initial code frequency basis of NCO
    d_code_freq_chips = static_cast<double>(Galileo_E1_CODE_CHIP_RATE_HZ);
    // Residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    // Residual carrier phase
    d_rem_carr_phase_rad = 0.0;
    
    // sample synchronization
    d_sample_counter = 0;
    //d_sample_counter_seconds = 0;
    d_acq_sample_stamp = 0;
    
    d_enable_tracking = false;
    d_pull_in = false;
    d_last_seg = 0;
    
    d_current_prn_length_samples = static_cast<int>(d_vector_length);
    
    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[CN0_ESTIMATION_SAMPLES];
    d_carrier_lock_test = 1;
    d_CN0_SNV_dB_Hz = 0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_threshold = CARRIER_LOCK_THRESHOLD;
    
    systemName["E"] = std::string("Galileo");
    *d_Very_Early = gr_complex(0,0);
    *d_Early = gr_complex(0,0);
    *d_Prompt = gr_complex(0,0);
    *d_Late = gr_complex(0,0);
    *d_Very_Late = gr_complex(0,0);

    d_channel_internal_queue = 0;
    d_acquisition_gnss_synchro = 0;
    d_channel = 0;
    d_acq_code_phase_samples = 0.0;
    d_acq_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_acc_code_phase_secs = 0.0;
}

void galileo_volk_e1_dll_pll_veml_tracking_cc::start_tracking()
{
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp =  d_acquisition_gnss_synchro->Acq_samplestamp_samples;
    
    // DLL/PLL filter initialization
    d_carrier_loop_filter.initialize(); // initialize the carrier filter
    d_code_loop_filter.initialize();    // initialize the code filter
    
    // generate local reference ALWAYS starting at chip 2 (2 samples per chip)
    galileo_e1_code_gen_complex_sampled(&d_ca_code[2],
                                        d_acquisition_gnss_synchro->Signal,
                                        false,
                                        d_acquisition_gnss_synchro->PRN,
                                        2 * Galileo_E1_CODE_CHIP_RATE_HZ,
                                        0);
    // Fill head and tail
    d_ca_code[0] = d_ca_code[static_cast<int>(2 * Galileo_E1_B_CODE_LENGTH_CHIPS)];
    d_ca_code[1] = d_ca_code[static_cast<int>(2 * Galileo_E1_B_CODE_LENGTH_CHIPS + 1)];
    d_ca_code[static_cast<int>(2 * Galileo_E1_B_CODE_LENGTH_CHIPS + 2)] = d_ca_code[2];
    d_ca_code[static_cast<int>(2 * Galileo_E1_B_CODE_LENGTH_CHIPS + 3)] = d_ca_code[3];
    
    d_carrier_lock_fail_counter = 0;
    d_rem_code_phase_samples = 0.0;
    d_rem_carr_phase_rad = 0;
    d_acc_carrier_phase_rad = 0;
    
    d_acc_code_phase_secs = 0;
    d_carrier_doppler_hz = d_acq_carrier_doppler_hz;
    d_current_prn_length_samples = d_vector_length;
    
    std::string sys_ = &d_acquisition_gnss_synchro->System;
    sys = sys_.substr(0, 1);
    
    // DEBUG OUTPUT
    std::cout << "Tracking start on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
    LOG(INFO) << "Starting tracking of satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;
    
    // enable tracking
    d_pull_in = true;
    d_enable_tracking = true;
    
    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz
    << " PULL-IN Code Phase [samples]=" << d_acq_code_phase_samples;
}


void galileo_volk_e1_dll_pll_veml_tracking_cc::update_local_code()
{
    double tcode_half_chips;
    float rem_code_phase_half_chips;
    int code_length_half_chips = static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS) * 2;
    double code_phase_step_chips;
    double code_phase_step_half_chips;
    int early_late_spc_samples;
    int very_early_late_spc_samples;
    int epl_loop_length_samples;
    
    // unified loop for VE, E, P, L, VL code vectors
    code_phase_step_chips = (static_cast<double>(d_code_freq_chips)) / (static_cast<double>(d_fs_in));
    code_phase_step_half_chips = (2.0 * static_cast<double>(d_code_freq_chips)) / (static_cast<double>(d_fs_in));
    
    rem_code_phase_half_chips = d_rem_code_phase_samples * (2*d_code_freq_chips / d_fs_in);
    tcode_half_chips = - static_cast<double>(rem_code_phase_half_chips);
    
    early_late_spc_samples = round(d_early_late_spc_chips / code_phase_step_chips);
    very_early_late_spc_samples = round(d_very_early_late_spc_chips / code_phase_step_chips);
    
    epl_loop_length_samples = d_current_prn_length_samples + very_early_late_spc_samples * 2;
    
    //HERE YOU CAN CHOOSE THE DESIRED VOLK IMPLEMENTATION
    //volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc_manual(d_very_early_code, (float) d_very_early_late_spc_chips, (float) code_length_half_chips, (float) code_phase_step_half_chips, (float) tcode_half_chips, d_ca_code, epl_loop_length_samples, "generic");
    volk_gnsssdr_32fc_s32f_x4_update_local_code_32fc(d_very_early_code, (float) d_very_early_late_spc_chips, (float) code_length_half_chips, (float) code_phase_step_half_chips, (float) tcode_half_chips, d_ca_code, epl_loop_length_samples);

    memcpy(d_early_code, &d_very_early_code[very_early_late_spc_samples - early_late_spc_samples], d_current_prn_length_samples * sizeof(gr_complex));
    memcpy(d_prompt_code, &d_very_early_code[very_early_late_spc_samples], d_current_prn_length_samples * sizeof(gr_complex));
    memcpy(d_late_code, &d_very_early_code[very_early_late_spc_samples + early_late_spc_samples], d_current_prn_length_samples * sizeof(gr_complex));
    memcpy(d_very_late_code, &d_very_early_code[2 * very_early_late_spc_samples], d_current_prn_length_samples * sizeof(gr_complex));
}

void galileo_volk_e1_dll_pll_veml_tracking_cc::update_local_carrier()
{
    float phase_rad, phase_step_rad;
    // Compute the carrier phase step for the K-1 carrier doppler estimation
    phase_step_rad = static_cast<float>(GPS_TWO_PI) * d_carrier_doppler_hz / static_cast<float>(d_fs_in);
    // Initialize the carrier phase with the remanent carrier phase of the K-2 loop
    phase_rad = d_rem_carr_phase_rad;
    
    //HERE YOU CAN CHOOSE THE DESIRED VOLK IMPLEMENTATION
    //volk_gnsssdr_s32f_x2_update_local_carrier_32fc_manual(d_carr_sign, phase_rad, phase_step_rad, d_current_prn_length_samples, "generic");
    
    //volk_gnsssdr_s32f_x2_update_local_carrier_32fc_manual(d_carr_sign, phase_rad, phase_step_rad, d_current_prn_length_samples, "u_sse2");
    
    volk_gnsssdr_s32f_x2_update_local_carrier_32fc(d_carr_sign, phase_rad, phase_step_rad, d_current_prn_length_samples);
}

galileo_volk_e1_dll_pll_veml_tracking_cc::~galileo_volk_e1_dll_pll_veml_tracking_cc()
{
    d_dump_file.close();
    
    volk_free(d_very_early_code);
    volk_free(d_early_code);
    volk_free(d_prompt_code);
    volk_free(d_late_code);
    volk_free(d_very_late_code);
    volk_free(d_carr_sign);
    volk_free(d_Very_Early);
    volk_free(d_Early);
    volk_free(d_Prompt);
    volk_free(d_Late);
    volk_free(d_Very_Late);
    volk_free(d_ca_code);
    
    volk_free(d_very_early_code16);
    volk_free(d_early_code16);
    volk_free(d_prompt_code16);
    volk_free(d_late_code16);
    volk_free(d_very_late_code16);
    volk_free(d_carr_sign16);
    volk_free(in16);
    
    volk_free(d_very_early_code8);
    volk_free(d_early_code8);
    volk_free(d_prompt_code8);
    volk_free(d_late_code8);
    volk_free(d_very_late_code8);
    volk_free(d_carr_sign8);
    volk_free(in8);
    
    delete[] d_Prompt_buffer;
}



int galileo_volk_e1_dll_pll_veml_tracking_cc::general_work (int noutput_items,gr_vector_int &ninput_items,
                                                       gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    float carr_error_hz;
    float carr_error_filt_hz;
    float code_error_chips;
    float code_error_filt_chips;
    
    if (d_enable_tracking == true)
    {
        if (d_pull_in == true)
        {
            /*
             * Signal alignment (skip samples until the incoming signal is aligned with local replica)
             */
            int samples_offset;
            float acq_trk_shif_correction_samples;
            int acq_to_trk_delay_samples;
            acq_to_trk_delay_samples = d_sample_counter - d_acq_sample_stamp;
            acq_trk_shif_correction_samples = d_current_prn_length_samples - fmod(static_cast<float>(acq_to_trk_delay_samples), static_cast<float>(d_current_prn_length_samples));
            samples_offset = round(d_acq_code_phase_samples + acq_trk_shif_correction_samples);
            d_sample_counter = d_sample_counter + samples_offset; //count for the processed samples
            d_pull_in = false;
            consume_each(samples_offset); //shift input to perform alignment with local replica
            return 1;
        }
        
        // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
        Gnss_Synchro current_synchro_data = Gnss_Synchro();
        // Fill the acquisition data
        current_synchro_data = *d_acquisition_gnss_synchro;
        
        // Block input data and block output stream pointers
        const gr_complex* in = (gr_complex*) input_items[0];
        Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0];
        
        // Generate local code and carrier replicas (using \hat{f}_d(k-1))
        update_local_code();
        update_local_carrier();
        
        // perform carrier wipe-off and compute Very Early, Early, Prompt, Late and Very Late correlation
        
        //HERE YOU CAN CHOOSE THE DESIRED VOLK IMPLEMENTATION
        
        //Float implementation:
        
        //volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5(d_Very_Early, d_Early, d_Prompt, d_Late, d_Very_Late, in, d_carr_sign, d_very_early_code, d_early_code, d_prompt_code, d_late_code, d_very_late_code, d_current_prn_length_samples, "generic");
        
        //volk_gnsssdr_32fc_x7_cw_vepl_corr_32fc_x5(d_Very_Early, d_Early, d_Prompt, d_Late, d_Very_Late, in, d_carr_sign, d_very_early_code, d_early_code, d_prompt_code, d_late_code, d_very_late_code, d_current_prn_length_samples, "u_avx");
        
        //Integer 16 bits implementation
        /*volk_gnsssdr_32fc_convert_16ic(d_very_early_code16, d_very_early_code, d_current_prn_length_samples);
         volk_gnsssdr_32fc_convert_16ic(d_early_code16, d_early_code, d_current_prn_length_samples);
         volk_gnsssdr_32fc_convert_16ic(d_prompt_code16, d_prompt_code, d_current_prn_length_samples);
         volk_gnsssdr_32fc_convert_16ic(d_late_code16, d_late_code, d_current_prn_length_samples);
         volk_gnsssdr_32fc_convert_16ic(d_very_late_code16, d_very_late_code, d_current_prn_length_samples);
         volk_gnsssdr_32fc_convert_16ic(in16, in, d_current_prn_length_samples);
         volk_gnsssdr_32fc_convert_16ic(d_carr_sign16, d_carr_sign, d_current_prn_length_samples);
         
         volk_gnsssdr_16ic_x7_cw_vepl_corr_32fc_x5(d_Very_Early, d_Early, d_Prompt, d_Late, d_Very_Late, in16, d_carr_sign16, d_very_early_code16, d_early_code16, d_prompt_code16, d_late_code16, d_very_late_code16, d_current_prn_length_samples);*/
        
        //Integer 8 bits implementation
        volk_gnsssdr_32fc_convert_8ic(d_very_early_code8, d_very_early_code, d_current_prn_length_samples);
        volk_gnsssdr_32fc_convert_8ic(d_early_code8, d_early_code, d_current_prn_length_samples);
        volk_gnsssdr_32fc_convert_8ic(d_prompt_code8, d_prompt_code, d_current_prn_length_samples);
        volk_gnsssdr_32fc_convert_8ic(d_late_code8, d_late_code, d_current_prn_length_samples);
        volk_gnsssdr_32fc_convert_8ic(d_very_late_code8, d_very_late_code, d_current_prn_length_samples);
        volk_gnsssdr_32fc_convert_8ic(d_carr_sign8, d_carr_sign, d_current_prn_length_samples);
        volk_gnsssdr_32fc_s32f_convert_8ic(in8, in, 4, d_current_prn_length_samples);
        
        volk_gnsssdr_8ic_x7_cw_vepl_corr_safe_32fc_x5(d_Very_Early, d_Early, d_Prompt, d_Late, d_Very_Late, in8, d_carr_sign8, d_very_early_code8, d_early_code8, d_prompt_code8, d_late_code8, d_very_late_code8, d_current_prn_length_samples);

        // ################## PLL ##########################################################
        // PLL discriminator
        carr_error_hz = pll_cloop_two_quadrant_atan(*d_Prompt) / static_cast<float>(GPS_TWO_PI);
        // Carrier discriminator filter
        carr_error_filt_hz = d_carrier_loop_filter.get_carrier_nco(carr_error_hz);
        // New carrier Doppler frequency estimation
        d_carrier_doppler_hz = d_acq_carrier_doppler_hz + carr_error_filt_hz;
        // New code Doppler frequency estimation
        d_code_freq_chips = Galileo_E1_CODE_CHIP_RATE_HZ + ((d_carrier_doppler_hz * Galileo_E1_CODE_CHIP_RATE_HZ) / Galileo_E1_FREQ_HZ);
        //carrier phase accumulator for (K) Doppler estimation
        d_acc_carrier_phase_rad = d_acc_carrier_phase_rad + GPS_TWO_PI * d_carrier_doppler_hz * Galileo_E1_CODE_PERIOD;
        //remnant carrier phase to prevent overflow in the code NCO
        d_rem_carr_phase_rad = d_rem_carr_phase_rad + GPS_TWO_PI * d_carrier_doppler_hz * Galileo_E1_CODE_PERIOD;
        d_rem_carr_phase_rad = fmod(d_rem_carr_phase_rad, GPS_TWO_PI);
        
        // ################## DLL ##########################################################
        // DLL discriminator
        code_error_chips = dll_nc_vemlp_normalized(*d_Very_Early, *d_Early, *d_Late, *d_Very_Late); //[chips/Ti]
        // Code discriminator filter
        code_error_filt_chips = d_code_loop_filter.get_code_nco(code_error_chips); //[chips/second]
        //Code phase accumulator
        float code_error_filt_secs;
        code_error_filt_secs = (Galileo_E1_CODE_PERIOD * code_error_filt_chips) / Galileo_E1_CODE_CHIP_RATE_HZ; //[seconds]
        //code_error_filt_secs=T_prn_seconds*code_error_filt_chips*T_chip_seconds*static_cast<float>(d_fs_in); //[seconds]
        d_acc_code_phase_secs = d_acc_code_phase_secs  + code_error_filt_secs;
        
        // ################## CARRIER AND CODE NCO BUFFER ALIGNEMENT #######################
        // keep alignment parameters for the next input buffer
        double T_chip_seconds;
        double T_prn_seconds;
        double T_prn_samples;
        double K_blk_samples;
        // Compute the next buffer lenght based in the new period of the PRN sequence and the code phase error estimation
        T_chip_seconds = 1 / static_cast<double>(d_code_freq_chips);
        T_prn_seconds = T_chip_seconds * Galileo_E1_B_CODE_LENGTH_CHIPS;
        T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
        K_blk_samples = T_prn_samples + d_rem_code_phase_samples + code_error_filt_secs * static_cast<double>(d_fs_in);
        d_current_prn_length_samples = round(K_blk_samples); //round to a discrete samples
        //d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample
        
        // ####### CN0 ESTIMATION AND LOCK DETECTORS ######
        if (d_cn0_estimation_counter < CN0_ESTIMATION_SAMPLES)
        {
            // fill buffer with prompt correlator output values
            d_Prompt_buffer[d_cn0_estimation_counter] = *d_Prompt;
            d_cn0_estimation_counter++;
        }
        else
        {
            d_cn0_estimation_counter = 0;
            
            // Code lock indicator
            d_CN0_SNV_dB_Hz = cn0_svn_estimator(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES, d_fs_in, Galileo_E1_B_CODE_LENGTH_CHIPS);
            
            // Carrier lock indicator
            d_carrier_lock_test = carrier_lock_detector(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES);
            
            // Loss of lock detection
            if (d_carrier_lock_test < d_carrier_lock_threshold or d_CN0_SNV_dB_Hz < MINIMUM_VALID_CN0)
            {
                d_carrier_lock_fail_counter++;
            }
            else
            {
                if (d_carrier_lock_fail_counter > 0) d_carrier_lock_fail_counter--;
            }
            if (d_carrier_lock_fail_counter > MAXIMUM_LOCK_FAIL_COUNTER)
            {
                std::cout << "Loss of lock in channel " << d_channel << "!" << std::endl;
                LOG(INFO) << "Loss of lock in channel " << d_channel << "!";
                std::unique_ptr<ControlMessageFactory> cmf(new ControlMessageFactory());
                if (d_queue != gr::msg_queue::sptr())
                {
                    d_queue->handle(cmf->GetQueueMessage(d_channel, 2));
                }
                d_carrier_lock_fail_counter = 0;
                d_enable_tracking = false; // TODO: check if disabling tracking is consistent with the channel state machine
            }
        }
        
        // ########### Output the tracking results to Telemetry block ##########
        
        current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt).real());
        current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt).imag());
        
        // Tracking_timestamp_secs is aligned with the NEXT PRN start sample (Hybridization problem!)
        //compute remnant code phase samples BEFORE the Tracking timestamp
        //d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample
        //current_synchro_data.Tracking_timestamp_secs = ((double)d_sample_counter +
        //        (double)d_current_prn_length_samples + (double)d_rem_code_phase_samples) / static_cast<double>(d_fs_in);
        
        // Tracking_timestamp_secs is aligned with the CURRENT PRN start sample (Hybridization OK!, but some glitches??)
        current_synchro_data.Tracking_timestamp_secs = (static_cast<double>(d_sample_counter) + static_cast<double>(d_rem_code_phase_samples)) / static_cast<double>(d_fs_in);
        //compute remnant code phase samples AFTER the Tracking timestamp
        d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample
        
        // This tracking block aligns the Tracking_timestamp_secs with the start sample of the PRN, thus, Code_phase_secs=0
        current_synchro_data.Code_phase_secs = 0;
        current_synchro_data.Carrier_phase_rads = static_cast<double>(d_acc_carrier_phase_rad);
        current_synchro_data.Carrier_Doppler_hz = static_cast<double>(d_carrier_doppler_hz);
        current_synchro_data.CN0_dB_hz = static_cast<double>(d_CN0_SNV_dB_Hz);
        current_synchro_data.Flag_valid_pseudorange = false;
        *out[0] = current_synchro_data;
        
        // ########## DEBUG OUTPUT
        /*!
         *  \todo The stop timer has to be moved to the signal source!
         */
        // stream to collect cout calls to improve thread safety
        std::stringstream tmp_str_stream;
        if (floor(d_sample_counter / d_fs_in) != d_last_seg)
        {
            d_last_seg = floor(d_sample_counter / d_fs_in);
            
            if (d_channel == 0)
            {
                // debug: Second counter in channel 0
                tmp_str_stream << "Current input signal time = " << d_last_seg << " [s]" << std::endl << std::flush;
                std::cout << tmp_str_stream.rdbuf() << std::flush;
            }
            
            tmp_str_stream << "Tracking CH " << d_channel <<  ": Satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN)
            << ", Doppler=" << d_carrier_doppler_hz << " [Hz] CN0 = " << d_CN0_SNV_dB_Hz << " [dB-Hz]" << std::endl;
            LOG(INFO) << tmp_str_stream.rdbuf() << std::flush;
            //if (d_channel == 0 || d_last_seg==5) d_carrier_lock_fail_counter=500; //DEBUG: force unlock!
        }
    }
    else
    {
        // ########## DEBUG OUTPUT (TIME ONLY for channel 0 when tracking is disabled)
        /*!
         *  \todo The stop timer has to be moved to the signal source!
         */
        // stream to collect cout calls to improve thread safety
        std::stringstream tmp_str_stream;
        if (floor(d_sample_counter / d_fs_in) != d_last_seg)
        {
            d_last_seg = floor(d_sample_counter / d_fs_in);
            
            if (d_channel == 0)
            {
                // debug: Second counter in channel 0
                tmp_str_stream << "Current input signal time = " << d_last_seg << " [s]" << std::endl << std::flush;
                std::cout << tmp_str_stream.rdbuf() << std::flush;
            }
        }
        *d_Early = gr_complex(0,0);
        *d_Prompt = gr_complex(0,0);
        *d_Late = gr_complex(0,0);
        Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0]; //block output stream pointer
        // GNSS_SYNCHRO OBJECTto interchange data between tracking->telemetry_decoder
        d_acquisition_gnss_synchro->Flag_valid_pseudorange = false;
        *out[0] = *d_acquisition_gnss_synchro;
    }
    
    if(d_dump)
    {
        // Dump results to file
        float prompt_I;
        float prompt_Q;
        float tmp_VE, tmp_E, tmp_P, tmp_L, tmp_VL;
        float tmp_float;
        double tmp_double;
        prompt_I = (*d_Prompt).real();
        prompt_Q = (*d_Prompt).imag();
        tmp_VE = std::abs<float>(*d_Very_Early);
        tmp_E = std::abs<float>(*d_Early);
        tmp_P = std::abs<float>(*d_Prompt);
        tmp_L = std::abs<float>(*d_Late);
        tmp_VL = std::abs<float>(*d_Very_Late);
        
        try
        {
            // Dump correlators output
            d_dump_file.write(reinterpret_cast<char*>(&tmp_VE), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&tmp_E), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&tmp_P), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&tmp_L), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&tmp_VL), sizeof(float));
            // PROMPT I and Q (to analyze navigation symbols)
            d_dump_file.write(reinterpret_cast<char*>(&prompt_I), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&prompt_Q), sizeof(float));
            // PRN start sample stamp
            d_dump_file.write(reinterpret_cast<char*>(&d_sample_counter), sizeof(unsigned long int));
            // accumulated carrier phase
            d_dump_file.write(reinterpret_cast<char*>(&d_acc_carrier_phase_rad), sizeof(float));
            // carrier and code frequency
            d_dump_file.write(reinterpret_cast<char*>(&d_carrier_doppler_hz), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&d_code_freq_chips), sizeof(float));
            //PLL commands
            d_dump_file.write(reinterpret_cast<char*>(&carr_error_hz), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&carr_error_filt_hz), sizeof(float));
            //DLL commands
            d_dump_file.write(reinterpret_cast<char*>(&code_error_chips), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&code_error_filt_chips), sizeof(float));
            // CN0 and carrier lock test
            d_dump_file.write(reinterpret_cast<char*>(&d_CN0_SNV_dB_Hz), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&d_carrier_lock_test), sizeof(float));
            // AUX vars (for debug purposes)
            tmp_float = d_rem_code_phase_samples;
            d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));
            tmp_double = static_cast<double>(d_sample_counter + d_current_prn_length_samples);
            d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
        }
        catch (std::ifstream::failure e)
        {
            LOG(WARNING) << "Exception writing trk dump file " << e.what() << std::endl;
        }
    }
    consume_each(d_current_prn_length_samples); // this is required for gr_block derivates
    d_sample_counter += d_current_prn_length_samples; //count for the processed samples
    //std::cout<<"Galileo tracking output at sample "<<d_sample_counter<<std::endl;
    return 1; //output tracking result ALWAYS even in the case of d_enable_tracking==false
}



void galileo_volk_e1_dll_pll_veml_tracking_cc::set_channel(unsigned int channel)
{
    d_channel = channel;
    LOG(INFO) << "Tracking Channel set to " << d_channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
    {
        if (d_dump_file.is_open() == false)
        {
            try
            {
                d_dump_filename.append(boost::lexical_cast<std::string>(d_channel));
                d_dump_filename.append(".dat");
                d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit);
                d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                LOG(INFO) << "Tracking dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
            }
            catch (std::ifstream::failure e)
            {
                LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what() << std::endl;
            }
        }
    }
}



void galileo_volk_e1_dll_pll_veml_tracking_cc::set_channel_queue(concurrent_queue<int> *channel_internal_queue)
{
    d_channel_internal_queue = channel_internal_queue;
}



void galileo_volk_e1_dll_pll_veml_tracking_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
    //  Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    //DLOG(INFO) << "Tracking code phase set to " << d_acq_code_phase_samples;
    //DLOG(INFO) << "Tracking carrier doppler set to " << d_acq_carrier_doppler_hz;
    //DLOG(INFO) << "Tracking Satellite set to " << d_satellite;
}
