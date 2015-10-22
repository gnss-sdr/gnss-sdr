/*!
 * \file galileo_e1_de_tracking_cc.cc
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

#include "galileo_e1_de_tracking_cc.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <gnuradio/io_signature.h>
#include <gnuradio/fxpt.h>  // fixed point sine and cosine
#include <glog/logging.h>
#include "gnss_synchro.h"
#include "galileo_e1_signal_processing.h"
#include "tracking_discriminators.h"
#include "lock_detectors.h"
#include "Galileo_E1.h"
#include "control_message_factory.h"
#include "fxpt64.h"



/*!
 * \todo Include in definition header file
 */
#define CN0_ESTIMATION_SAMPLES 20
#define MINIMUM_VALID_CN0 25
#define MAXIMUM_LOCK_FAIL_COUNTER 50
#define CARRIER_LOCK_THRESHOLD 0.85


using google::LogMessage;

galileo_e1_de_tracking_cc_sptr
galileo_e1_de_make_tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float sll_bw_hz,
        float early_late_code_space_chips,
        float early_late_subcarrier_space_chips,
        bool aid_subcarrier_with_carrier,
        bool aid_code_with_subcarrier)
{
    return galileo_e1_de_tracking_cc_sptr(new galileo_e1_de_tracking_cc(if_freq,
            fs_in, vector_length, queue, dump, dump_filename, pll_bw_hz, dll_bw_hz,
            sll_bw_hz, early_late_code_space_chips, early_late_subcarrier_space_chips,
            aid_subcarrier_with_carrier, aid_code_with_subcarrier));
}


void galileo_e1_de_tracking_cc::forecast (int noutput_items,
        gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = static_cast<int>(d_vector_length) * 2; //set the required available samples in each call
}


galileo_e1_de_tracking_cc::galileo_e1_de_tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float sll_bw_hz,
        float early_late_code_space_chips,
        float early_late_subcarrier_space_chips,
        bool aid_subcarrier_with_carrier,
        bool aid_code_with_subcarrier):
        gr::block("galileo_e1_de_tracking_cc", gr::io_signature::make(1, 1, sizeof(gr_complex)),
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
    d_subcarrier_loop_filter = Tracking_2nd_DLL_filter(Galileo_E1_CODE_PERIOD);
    d_carrier_loop_filter = Tracking_2nd_PLL_filter(Galileo_E1_CODE_PERIOD);
    d_aid_subcarrier_with_carrier = aid_subcarrier_with_carrier;
    d_aid_code_with_subcarrier = aid_code_with_subcarrier;

    // Initialize tracking  ==========================================

    // Set bandwidth of code and carrier loop filters
    d_code_loop_filter.set_DLL_BW(dll_bw_hz);
    d_subcarrier_loop_filter.set_DLL_BW(sll_bw_hz);
    d_carrier_loop_filter.set_PLL_BW(pll_bw_hz);

    // Correlator spacing
    d_early_late_code_spc_chips = early_late_code_space_chips; // Define early-late offset (in chips)
    d_early_late_subcarrier_spc_chips = early_late_subcarrier_space_chips; // Define very-early-late offset (in chips)

    // Initialization of local code replica
    // Get space for a vector with the code replica sampled 1x/chip
    d_e1b_code = static_cast<gr_complex*>(volk_malloc((Galileo_E1_B_CODE_LENGTH_CHIPS + 2) * sizeof(gr_complex), volk_get_alignment()));

    d_early_code= static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_prompt_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_late_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_early_subcarrier = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_prompt_subcarrier = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_late_subcarrier = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_carr_sign = static_cast<gr_complex*>(volk_malloc(2*d_vector_length * sizeof(gr_complex), volk_get_alignment()));

    // correlator outputs (scalar)
    d_Prompt_Subcarrier_Early_Code = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Subcarrier_Prompt_Code = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Subcarrier_Late_Code = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Code_Early_Subcarrier = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Code_Late_Subcarrier = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));

    //--- Initializations ------------------------------
    // Initial code frequency basis of NCO
    d_code_freq_chips = static_cast<double>(Galileo_E1_CODE_CHIP_RATE_HZ);
    d_subcarrier_freq_chips = static_cast<double>(Galileo_E1_SUB_CARRIER_A_RATE_HZ);
    // Residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    d_rem_subcarrier_phase_samples = 0.0;
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
    *d_Prompt_Subcarrier_Early_Code = gr_complex(0,0);
    *d_Prompt_Subcarrier_Prompt_Code = gr_complex(0,0);
    *d_Prompt_Subcarrier_Late_Code = gr_complex(0,0);
    *d_Prompt_Code_Early_Subcarrier = gr_complex(0,0);
    *d_Prompt_Code_Late_Subcarrier = gr_complex(0,0);

    d_channel_internal_queue = 0;
    d_acquisition_gnss_synchro = 0;
    d_channel = 0;
    d_acq_code_phase_samples = 0.0;
    d_acq_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_acc_code_phase_secs = 0.0;
}

void galileo_e1_de_tracking_cc::start_tracking()
{
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp =  d_acquisition_gnss_synchro->Acq_samplestamp_samples;

    // DLL/PLL filter initialization
    d_carrier_loop_filter.initialize(d_acq_carrier_doppler_hz); // initialize the carrier filter
    float code_doppler_chips = d_acq_carrier_doppler_hz *( Galileo_E1_CODE_CHIP_RATE_HZ) / Galileo_E1_FREQ_HZ;

    float init_freq = ( d_aid_subcarrier_with_carrier ? 0.0 : code_doppler_chips );
    d_subcarrier_loop_filter.initialize(init_freq); // initialize the carrier filter
    init_freq = ( d_aid_code_with_subcarrier ? 0.0 : code_doppler_chips );
    d_code_loop_filter.initialize(init_freq);    // initialize the code filter

    // generate local reference ALWAYS starting at chip 1 (1 samples per chip)
    galileo_e1_prn_gen_complex_sampled(&d_e1b_code[1],
                                        d_acquisition_gnss_synchro->Signal,
                                        d_acquisition_gnss_synchro->PRN,
                                        Galileo_E1_CODE_CHIP_RATE_HZ,
                                        0);
    // Fill head and tail
    d_e1b_code[0] = d_e1b_code[static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS+1)];
    d_e1b_code[static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS + 2)] = d_e1b_code[1];

    d_carrier_lock_fail_counter = 0;
    d_rem_code_phase_samples = 0.0;
    d_rem_subcarrier_phase_samples = 0.0;
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


void galileo_e1_de_tracking_cc::update_local_code()
{
    double tcode_chips;
    double tsubcarrier_phase_halfcyles;
    float rem_code_phase_chips;
    float rem_subcarrier_phase_halfcycles;
    int associated_chip_index;
    int associated_subcarrier_index;
    int code_length_chips = static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS);
    double code_phase_step_chips;
    double subcarrier_phase_step_halfcycles;
    double early_late_subcarrier_spc_halfcycles;
    double subcarrier_freq_halfcycles;

    double chips_to_halfcycles = Galileo_E1_SUB_CARRIER_A_RATE_HZ /
        Galileo_E1_CODE_CHIP_RATE_HZ * 2.0;


    subcarrier_freq_halfcycles = d_subcarrier_freq_chips * chips_to_halfcycles;

    code_phase_step_chips = (static_cast<double>(d_code_freq_chips)) / (static_cast<double>(d_fs_in));
    subcarrier_phase_step_halfcycles = subcarrier_freq_halfcycles/ (static_cast<double>(d_fs_in));

    rem_code_phase_chips = d_rem_code_phase_samples * (d_code_freq_chips / d_fs_in);
    tcode_chips = - static_cast<double>(rem_code_phase_chips) + 1.0;


    rem_subcarrier_phase_halfcycles = d_rem_subcarrier_phase_samples * subcarrier_freq_halfcycles/static_cast<double>(d_fs_in);
    tsubcarrier_phase_halfcyles = - static_cast<double>(rem_subcarrier_phase_halfcycles);

    early_late_subcarrier_spc_halfcycles = d_early_late_subcarrier_spc_chips * chips_to_halfcycles;

    int64_t early_code_phase_fxp = double_to_fxpt64( tcode_chips + d_early_late_code_spc_chips );
    int64_t prompt_code_phase_fxp = double_to_fxpt64( tcode_chips );
    int64_t late_code_phase_fxp = double_to_fxpt64( tcode_chips - d_early_late_code_spc_chips);

    int64_t early_subcarrier_phase_fxp = double_to_fxpt64(
            tsubcarrier_phase_halfcyles + early_late_subcarrier_spc_halfcycles );
    int64_t prompt_subcarrier_phase_fxp = double_to_fxpt64(
            tsubcarrier_phase_halfcyles );
    int64_t late_subcarrier_phase_fxp = double_to_fxpt64(
            tsubcarrier_phase_halfcyles - early_late_subcarrier_spc_halfcycles );

    int64_t code_phase_step_fxp = double_to_fxpt64( code_phase_step_chips );
    int64_t subcarrier_phase_step_fxp = double_to_fxpt64( subcarrier_phase_step_halfcycles );

    for (int i = 0; i < d_current_prn_length_samples; i++)
    {
        d_early_code[i] = d_e1b_code[ (early_code_phase_fxp >> 32 )];
        d_prompt_code[i] = d_e1b_code[ (prompt_code_phase_fxp >> 32 )];
        d_late_code[i] = d_e1b_code[ (late_code_phase_fxp >> 32 )];

        d_early_subcarrier[i] = (1.0 - 2.0*( (early_subcarrier_phase_fxp>>32)&0x01 ) );
        d_prompt_subcarrier[i] = (1.0 - 2.0*( (prompt_subcarrier_phase_fxp>>32)&0x01 ) );
        d_late_subcarrier[i] = (1.0 - 2.0*( (late_subcarrier_phase_fxp>>32)&0x01 ) );

        early_code_phase_fxp += code_phase_step_fxp;
        prompt_code_phase_fxp += code_phase_step_fxp;
        late_code_phase_fxp += code_phase_step_fxp;

        early_subcarrier_phase_fxp += subcarrier_phase_step_fxp;
        prompt_subcarrier_phase_fxp += subcarrier_phase_step_fxp;
        late_subcarrier_phase_fxp += subcarrier_phase_step_fxp;

    }
}


void galileo_e1_de_tracking_cc::update_local_carrier()
{
    float sin_f, cos_f;
    float phase_step_rad = static_cast<float>(2 * GALILEO_PI) * d_carrier_doppler_hz / static_cast<float>(d_fs_in);
    int phase_step_rad_i = gr::fxpt::float_to_fixed(phase_step_rad);
    int phase_rad_i = gr::fxpt::float_to_fixed(d_rem_carr_phase_rad);

    for(int i = 0; i < d_current_prn_length_samples; i++)
        {
            gr::fxpt::sincos(phase_rad_i, &sin_f, &cos_f);
            d_carr_sign[i] = std::complex<float>(cos_f, -sin_f);
            phase_rad_i += phase_step_rad_i;
        }
}

galileo_e1_de_tracking_cc::~galileo_e1_de_tracking_cc()
{
    d_dump_file.close();

    volk_free(d_early_code);
    volk_free(d_prompt_code);
    volk_free(d_late_code);
    volk_free(d_early_subcarrier);
    volk_free(d_prompt_subcarrier);
    volk_free(d_late_subcarrier);
    volk_free(d_carr_sign);
    volk_free(d_Prompt_Subcarrier_Early_Code);
    volk_free(d_Prompt_Subcarrier_Prompt_Code);
    volk_free(d_Prompt_Subcarrier_Late_Code);
    volk_free(d_Prompt_Code_Early_Subcarrier);
    volk_free(d_Prompt_Code_Late_Subcarrier);
    volk_free(d_e1b_code);

    delete[] d_Prompt_buffer;
}



int galileo_e1_de_tracking_cc::general_work (int noutput_items,gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    float carr_error_hz;
    float carr_error_filt_hz;
    float code_error_chips;
    float subcarrier_error_chips;
    float code_error_filt_chips;
    float subcarrier_error_filt_chips;

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
            Gnss_Synchro current_synchro_data;
            // Fill the acquisition data
            current_synchro_data = *d_acquisition_gnss_synchro;

            // Block input data and block output stream pointers
            const gr_complex* in = (gr_complex*) input_items[0];
            Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0];

            // Generate local code and carrier replicas (using \hat{f}_d(k-1))
            update_local_code();
            //update_local_carrier();

            gr_complex phase_as_complex( std::cos( d_rem_carr_phase_rad ),
                        -std::sin( d_rem_carr_phase_rad ) );

            double carrier_doppler_inc_rad = 2.0*M_PI*(d_if_freq + d_carrier_doppler_hz )/d_fs_in;

            gr_complex phase_inc_as_complex( std::cos( carrier_doppler_inc_rad ),
                    -std::sin( carrier_doppler_inc_rad ) );


            // perform carrier wipe-off and compute Very Early, Early, Prompt, Late and Very Late correlation
            d_correlator.Carrier_rotate_and_DE_volk(d_current_prn_length_samples,
                    in,
                    &phase_as_complex,
                    phase_inc_as_complex,
                    d_early_code,
                    d_prompt_code,
                    d_late_code,
                    d_early_subcarrier,
                    d_prompt_subcarrier,
                    d_late_subcarrier,
                    d_Prompt_Subcarrier_Early_Code,
                    d_Prompt_Subcarrier_Prompt_Code,
                    d_Prompt_Subcarrier_Late_Code,
                    d_Prompt_Code_Early_Subcarrier,
                    d_Prompt_Code_Late_Subcarrier );

            // ################## PLL ##########################################################
            // PLL discriminator
            carr_error_hz = pll_cloop_two_quadrant_atan(*d_Prompt_Subcarrier_Prompt_Code) / static_cast<float>(GPS_TWO_PI);
            // Carrier discriminator filter
            carr_error_filt_hz = d_carrier_loop_filter.get_carrier_nco(carr_error_hz);
            // New carrier Doppler frequency estimation
            d_carrier_doppler_hz = carr_error_filt_hz;

            float code_doppler_chips = ((d_carrier_doppler_hz * Galileo_E1_CODE_CHIP_RATE_HZ) / Galileo_E1_FREQ_HZ);

            // New subcarrier Doppler frequency estimation: carrier
            // aiding of the subcarrier:
            if( d_aid_subcarrier_with_carrier )
            {
                d_subcarrier_freq_chips = Galileo_E1_CODE_CHIP_RATE_HZ + code_doppler_chips;
            }
            else
            {
                d_subcarrier_freq_chips = Galileo_E1_CODE_CHIP_RATE_HZ;
            }
            //carrier phase accumulator for (K) Doppler estimation
            d_acc_carrier_phase_rad = d_acc_carrier_phase_rad + GPS_TWO_PI * d_carrier_doppler_hz * Galileo_E1_CODE_PERIOD;
            //remnant carrier phase to prevent overflow in the code NCO
            d_rem_carr_phase_rad = d_rem_carr_phase_rad + GPS_TWO_PI * d_carrier_doppler_hz * Galileo_E1_CODE_PERIOD;
            d_rem_carr_phase_rad = fmod(d_rem_carr_phase_rad, GPS_TWO_PI);

            // ################## SLL ##########################################################
            // SLL discriminator
            subcarrier_error_chips = dll_nc_e_minus_l_normalized(
                    *d_Prompt_Code_Early_Subcarrier,
                    *d_Prompt_Code_Late_Subcarrier); //[chips/Ti]
            // normalise the SLL discriminator by the slope of the
            // BOC(1,1) at the origin:
            float corr_slope = 3.0;
            subcarrier_error_chips *= ( 1 - corr_slope*d_early_late_subcarrier_spc_chips) / corr_slope;
            // Subcarrier discriminator filter
            subcarrier_error_filt_chips = d_subcarrier_loop_filter.get_code_nco(subcarrier_error_chips); //[chips/second]
            d_subcarrier_freq_chips += subcarrier_error_filt_chips;
            // Aiding the code tracking with the subcarrier:
            if( d_aid_code_with_subcarrier )
            {
                d_code_freq_chips = d_subcarrier_freq_chips;
            }
            else
            {
                d_code_freq_chips = Galileo_E1_CODE_CHIP_RATE_HZ;
            }

            //Subcarrier phase accumulator
            float subcarrier_error_filt_secs;
            subcarrier_error_filt_secs = (Galileo_E1_CODE_PERIOD * subcarrier_error_filt_chips) / Galileo_E1_CODE_CHIP_RATE_HZ; //[seconds]
            //code_error_filt_secs=T_prn_seconds*code_error_filt_chips*T_chip_seconds*static_cast<float>(d_fs_in); //[seconds]

            // ################## DLL ##########################################################
            // DLL discriminator
            code_error_chips = dll_nc_e_minus_l_normalized(
                    *d_Prompt_Subcarrier_Early_Code,
                    *d_Prompt_Subcarrier_Late_Code); //[chips/Ti]
            //Normalise the code phase error:
            corr_slope = 1.0;
            code_error_chips *= ( 1 - corr_slope*d_early_late_code_spc_chips) / corr_slope;
            // Code discriminator filter
            code_error_filt_chips = d_code_loop_filter.get_code_nco(code_error_chips); //[chips/second]
            //Code phase accumulator
            d_code_freq_chips += code_error_filt_chips;
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
            double T_sc_seconds;
            double T_sc_prn_seconds;
            double T_sc_prn_samples;
            double K_sc_samples;
            // Compute the next buffer lenght based in the new period of the PRN sequence and the code phase error estimation
            T_chip_seconds = 1 / static_cast<double>(d_code_freq_chips);
            T_prn_seconds = T_chip_seconds * Galileo_E1_B_CODE_LENGTH_CHIPS;
            T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
            K_blk_samples = T_prn_samples + d_rem_code_phase_samples; // + code_error_filt_secs * static_cast<double>(d_fs_in);

            T_sc_seconds = 1 / static_cast<double>(d_subcarrier_freq_chips);
            // THere is one subcarrier period per code chip:
            T_sc_prn_seconds = T_sc_seconds * Galileo_E1_B_CODE_LENGTH_CHIPS;
            T_sc_prn_samples = T_sc_prn_seconds * static_cast<double>(d_fs_in);
            K_sc_samples = T_sc_prn_samples + d_rem_subcarrier_phase_samples; 

            d_current_prn_length_samples = round(K_blk_samples); //round to a discrete samples
            //d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample

            // ####### CN0 ESTIMATION AND LOCK DETECTORS ######
            if (d_cn0_estimation_counter < CN0_ESTIMATION_SAMPLES)
                {
                    // fill buffer with prompt correlator output values
                    d_Prompt_buffer[d_cn0_estimation_counter] = *d_Prompt_Subcarrier_Prompt_Code;
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

            current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt_Subcarrier_Prompt_Code).real());
            current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt_Subcarrier_Prompt_Code).imag());

            // Tracking_timestamp_secs is aligned with the NEXT PRN start sample (Hybridization problem!)
            //compute remnant code phase samples BEFORE the Tracking timestamp
            //d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample
            //current_synchro_data.Tracking_timestamp_secs = ((double)d_sample_counter +
            //        (double)d_current_prn_length_samples + (double)d_rem_code_phase_samples) / static_cast<double>(d_fs_in);

            // Tracking_timestamp_secs is aligned with the CURRENT PRN start sample (Hybridization OK!, but some glitches??)
            current_synchro_data.Tracking_timestamp_secs = (static_cast<double>(d_sample_counter) + static_cast<double>(d_rem_code_phase_samples)) / static_cast<double>(d_fs_in);
            //compute remnant code phase samples AFTER the Tracking timestamp
            d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample

            d_rem_subcarrier_phase_samples = K_sc_samples - d_current_prn_length_samples; //rounding error < 1 sample
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
    	*d_Prompt_Subcarrier_Early_Code = gr_complex(0,0);
    	*d_Prompt_Subcarrier_Prompt_Code = gr_complex(0,0);
    	*d_Prompt_Subcarrier_Late_Code = gr_complex(0,0);
    	*d_Prompt_Code_Early_Subcarrier = gr_complex(0,0);
    	*d_Prompt_Code_Late_Subcarrier = gr_complex(0,0);
    	Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0]; //block output stream pointer
    	// GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
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
            prompt_I = (*d_Prompt_Subcarrier_Prompt_Code).real();
            prompt_Q = (*d_Prompt_Subcarrier_Prompt_Code).imag();
            tmp_VE = std::abs<float>(*d_Prompt_Code_Early_Subcarrier);
            tmp_E = std::abs<float>(*d_Prompt_Subcarrier_Early_Code);
            tmp_P = std::abs<float>(*d_Prompt_Subcarrier_Prompt_Code);
            tmp_L = std::abs<float>(*d_Prompt_Subcarrier_Late_Code);
            tmp_VL = std::abs<float>(*d_Prompt_Code_Late_Subcarrier);

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
                    // SLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&subcarrier_error_chips), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&subcarrier_error_filt_chips), sizeof(float));
                    // CN0 and carrier lock test
                    d_dump_file.write(reinterpret_cast<char*>(&d_CN0_SNV_dB_Hz), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&d_carrier_lock_test), sizeof(float));
                    // AUX vars (for debug purposes)
                    tmp_float = d_rem_code_phase_samples/static_cast< float >( d_fs_in )*Galileo_E1_CODE_CHIP_RATE_HZ;
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));
                    tmp_double = static_cast<double>(d_sample_counter + d_current_prn_length_samples);
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                    tmp_float = d_rem_subcarrier_phase_samples/static_cast<float>( d_fs_in ) * Galileo_E1_CODE_CHIP_RATE_HZ;
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));
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



void galileo_e1_de_tracking_cc::set_channel(unsigned int channel)
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



void galileo_e1_de_tracking_cc::set_channel_queue(concurrent_queue<int> *channel_internal_queue)
{
    d_channel_internal_queue = channel_internal_queue;
}



void galileo_e1_de_tracking_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
    //  Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    //DLOG(INFO) << "Tracking code phase set to " << d_acq_code_phase_samples;
    //DLOG(INFO) << "Tracking carrier doppler set to " << d_acq_carrier_doppler_hz;
    //DLOG(INFO) << "Tracking Satellite set to " << d_satellite;
}

