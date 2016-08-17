/*!
 * \file galileo_e1_prs_de_tracking_cc.cc
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

#include "galileo_e1_prs_de_tracking_cc.h"
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
#define MINIMUM_LOCK_SUCCESS_COUNTER 10
#define CARRIER_LOCK_THRESHOLD 0.85


using google::LogMessage;

galileo_e1_prs_de_tracking_cc_sptr
galileo_e1_prs_de_make_tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump,
        std::string dump_filename,
        int   pll_loop_order,
        float pll_initial_bw_hz,
        float pll_final_bw_hz,
        int   dll_loop_order,
        float dll_initial_bw_hz,
        float dll_final_bw_hz,
        int   sll_loop_order,
        float sll_initial_bw_hz,
        float sll_final_bw_hz,
        float initial_early_late_code_space_chips,
        float final_early_late_code_space_chips,
        float early_late_subcarrier_space_cycles,
        bool aid_subcarrier_with_carrier,
        bool aid_code_with_subcarrier,
        bool prs_sinusoidal_subcarrier,
        LongCodeInterface_sptr prs_code_gen)
{
    return galileo_e1_prs_de_tracking_cc_sptr(new galileo_e1_prs_de_tracking_cc(if_freq,
            fs_in, vector_length, queue, dump, dump_filename,
            pll_loop_order, pll_initial_bw_hz, pll_final_bw_hz,
            dll_loop_order, dll_initial_bw_hz, dll_final_bw_hz,
            sll_loop_order, sll_initial_bw_hz, sll_final_bw_hz,
            initial_early_late_code_space_chips, final_early_late_code_space_chips,
            early_late_subcarrier_space_cycles,
            aid_subcarrier_with_carrier, aid_code_with_subcarrier,
            prs_sinusoidal_subcarrier, prs_code_gen));
}


void galileo_e1_prs_de_tracking_cc::forecast (int noutput_items,
        gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = static_cast<int>(d_vector_length) * 2; //set the required available samples in each call
}


galileo_e1_prs_de_tracking_cc::galileo_e1_prs_de_tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump,
        std::string dump_filename,
        int   pll_loop_order,
        float pll_initial_bw_hz,
        float pll_final_bw_hz,
        int   dll_loop_order,
        float dll_initial_bw_hz,
        float dll_final_bw_hz,
        int   sll_loop_order,
        float sll_initial_bw_hz,
        float sll_final_bw_hz,
        float initial_early_late_code_space_chips,
        float final_early_late_code_space_chips,
        float early_late_subcarrier_space_cycles,
        bool aid_subcarrier_with_carrier,
        bool aid_code_with_subcarrier,
        bool prs_sinusoidal_subcarrier,
        LongCodeInterface_sptr prs_code_gen):
        gr::block("galileo_e1_prs_de_tracking_cc", gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // Create the gnss_message input port
    message_port_register_in( GNSS_MESSAGE_PORT_ID );
    set_msg_handler( GNSS_MESSAGE_PORT_ID,
            boost::bind( &galileo_e1_prs_de_tracking_cc::handle_gnss_message, this, _1 ) );

    d_prs_code_gen = prs_code_gen;

    d_carrier_locked = false;
    d_code_locked = false;
    d_code_locked_prs = false;

    this->set_relative_rate(1.0/vector_length);
    // initialize internal vars
    d_queue = queue;
    d_dump = dump;
    d_if_freq = if_freq;
    d_fs_in = fs_in;
    d_vector_length = vector_length;
    d_dump_filename = dump_filename;

    d_pll_loop_order = pll_loop_order;
    d_initial_pll_bw_hz = pll_initial_bw_hz;
    d_final_pll_bw_hz = pll_final_bw_hz;

    d_dll_loop_order = dll_loop_order;
    d_initial_dll_bw_hz = dll_initial_bw_hz;
    d_final_dll_bw_hz = dll_final_bw_hz;

    d_sll_loop_order = sll_loop_order;
    d_initial_sll_bw_hz = sll_initial_bw_hz;
    d_final_sll_bw_hz = sll_final_bw_hz;

    d_code_loop_filter = Tracking_loop_filter(Galileo_E1_CODE_PERIOD, dll_initial_bw_hz, dll_loop_order, false);
    d_subcarrier_loop_filter = Tracking_loop_filter(Galileo_E1_CODE_PERIOD, sll_initial_bw_hz, sll_loop_order, false);
    d_carrier_loop_filter = Tracking_loop_filter(Galileo_E1_CODE_PERIOD, pll_initial_bw_hz, pll_loop_order, false);
    d_aid_subcarrier_with_carrier = aid_subcarrier_with_carrier;
    d_aid_code_with_subcarrier = aid_code_with_subcarrier;

    d_code_loop_filter_prs = Tracking_loop_filter(Galileo_E1_CODE_PERIOD, dll_initial_bw_hz, dll_loop_order, false);
    d_subcarrier_loop_filter_prs = Tracking_loop_filter(Galileo_E1_CODE_PERIOD, sll_initial_bw_hz, sll_loop_order, false);
    d_carrier_loop_filter_prs = Tracking_loop_filter(Galileo_E1_CODE_PERIOD, pll_initial_bw_hz, pll_loop_order, false);
    // Initialize tracking  ==========================================


    // Correlator spacing
    d_initial_early_late_code_space_chips = initial_early_late_code_space_chips;
    d_final_early_late_code_space_chips = final_early_late_code_space_chips;
    d_early_late_code_spc_chips = d_initial_early_late_code_space_chips; // Define early-late offset (in chips)
    d_early_late_subcarrier_spc_cycles = early_late_subcarrier_space_cycles; // Define very-early-late offset (in chips)

    // Initialization of local code replica
    // Get space for a vector with the code replica sampled 1x/chip
    d_e1b_code = static_cast<gr_complex*>(volk_malloc((Galileo_E1_B_CODE_LENGTH_CHIPS + 2) * sizeof(gr_complex), volk_get_alignment()));

    // for the prs:
    // Store about 400 ms worth of chips:
    d_size_prs_code = static_cast< unsigned int>( std::ceil( 0.4 * Galileo_E1_A_CODE_CHIP_RATE_HZ ) );
    d_prs_code = static_cast< gr_complex*>(volk_malloc( d_size_prs_code*sizeof( gr_complex ), volk_get_alignment() ) );
    d_start_index_prs_code = 0;

    d_early_code= static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_prompt_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_late_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_early_subcarrier = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_prompt_subcarrier = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_late_subcarrier = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_carr_sign = static_cast<gr_complex*>(volk_malloc(2*d_vector_length * sizeof(gr_complex), volk_get_alignment()));

    d_early_code_prs= static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_prompt_code_prs = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_late_code_prs = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_early_subcarrier_prs = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_prompt_subcarrier_prs = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_late_subcarrier_prs = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_carr_sign = static_cast<gr_complex*>(volk_malloc(2*d_vector_length * sizeof(gr_complex), volk_get_alignment()));

    // Re-use the early subcarrier for the DPE
    d_prompt_quadrature_subcarrier_prs = d_early_subcarrier_prs;

    // correlator outputs (scalar)
    d_Prompt_Subcarrier_Early_Code = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Subcarrier_Prompt_Code = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Subcarrier_Late_Code = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Code_Early_Subcarrier = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Code_Late_Subcarrier = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));

    // correlator outputs (scalar)
    d_Prompt_Subcarrier_Early_Code_prs = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Subcarrier_Prompt_Code_prs = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Subcarrier_Late_Code_prs = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Code_Early_Subcarrier_prs = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_Code_Late_Subcarrier_prs= static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));

    // Re-use the prompt code early subcarrier for the DPE
    d_Prompt_Code_Quadrature_Subcarrier_prs = d_Prompt_Code_Early_Subcarrier_prs;

    //--- Initializations ------------------------------
    // Initial code frequency basis of NCO
    d_code_freq_chips = static_cast<double>(Galileo_E1_CODE_CHIP_RATE_HZ);
    d_subcarrier_freq_cycles = static_cast<double>(Galileo_E1_CODE_CHIP_RATE_HZ);

    d_code_freq_chips_prs = static_cast<double>(Galileo_E1_A_CODE_CHIP_RATE_HZ);
    d_subcarrier_freq_cycles_prs = static_cast<double>(Galileo_E1_A_SUB_CARRIER_RATE_HZ);
    // Residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    d_rem_subcarrier_phase_samples = 0.0;
    d_code_phase_chips = 0.0;
    d_subcarrier_phase_halfcycles = 0.0;

    d_rem_code_phase_samples_prs = 0.0;
    d_rem_subcarrier_phase_samples_prs = 0.0;
    d_integer_code_phase_chips_prs = 0;
    d_fractional_code_phase_chips_prs = 0.0;
    d_subcarrier_phase_halfcycles_prs = 0.0;

    // Residual carrier phase
    d_rem_carr_phase_rad = 0.0;
    d_rem_carr_phase_rad_prs = 0.0;

    // sample synchronization
    d_sample_counter = 0;
    //d_sample_counter_seconds = 0;
    d_acq_sample_stamp = 0;

    d_enable_tracking = false;
    d_pull_in = false;
    d_last_seg = 0;
    d_prs_tracking_enabled = false;
    d_prs_code_initialized = false;

    d_current_prn_length_samples = static_cast<int>(d_vector_length);

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[CN0_ESTIMATION_SAMPLES];
    d_carrier_lock_test = 1;
    d_CN0_SNV_dB_Hz = 0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_success_counter = 0;
    d_carrier_lock_threshold = CARRIER_LOCK_THRESHOLD;

    systemName["E"] = std::string("Galileo");
    *d_Prompt_Subcarrier_Early_Code = gr_complex(0,0);
    *d_Prompt_Subcarrier_Prompt_Code = gr_complex(0,0);
    *d_Prompt_Subcarrier_Late_Code = gr_complex(0,0);
    *d_Prompt_Code_Early_Subcarrier = gr_complex(0,0);
    *d_Prompt_Code_Late_Subcarrier = gr_complex(0,0);

    *d_Prompt_Subcarrier_Early_Code_prs = gr_complex(0,0);
    *d_Prompt_Subcarrier_Prompt_Code_prs = gr_complex(0,0);
    *d_Prompt_Subcarrier_Late_Code_prs = gr_complex(0,0);
    *d_Prompt_Code_Early_Subcarrier_prs = gr_complex(0,0);
    *d_Prompt_Code_Late_Subcarrier_prs = gr_complex(0,0);

    d_channel_internal_queue = 0;
    d_acquisition_gnss_synchro = 0;
    d_channel = 0;
    d_acq_code_phase_samples = 0.0;
    d_acq_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz_prs = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_acc_code_phase_secs = 0.0;

    d_tow_received = false;
    d_rx_time_set = false;
    d_preamble_start_detected = false;

    d_prs_sinusoidal_subcarrier = prs_sinusoidal_subcarrier;

    d_mean_code_error = 0.0;
    d_mean_code_error_prs = 0.0;
}

void galileo_e1_prs_de_tracking_cc::start_tracking()
{
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp =  d_acquisition_gnss_synchro->Acq_samplestamp_samples;

    // DLL/PLL filter initialization
    d_code_loop_filter.set_noise_bandwidth( d_initial_dll_bw_hz );
    d_subcarrier_loop_filter.set_noise_bandwidth( d_initial_sll_bw_hz );
    d_carrier_loop_filter.set_noise_bandwidth( d_initial_pll_bw_hz );

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
    d_carrier_lock_success_counter = 0;
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
    d_code_locked = false;
    d_carrier_locked = false;
    d_cn0_estimation_counter = 0;

    d_code_locked = false;
    d_mean_code_error = 0.0;

    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz
              << " PULL-IN Code Phase [samples]=" << d_acq_code_phase_samples;
}


void galileo_e1_prs_de_tracking_cc::update_local_code()
{
    double tcode_chips;
    double tsubcarrier_phase_halfcyles;
    float rem_code_phase_chips;
    int associated_chip_index;
    int associated_subcarrier_index;
    int code_length_chips = static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS);
    double code_phase_step_chips;
    double subcarrier_phase_step_halfcycles;
    double early_late_subcarrier_spc_halfcycles;
    double subcarrier_freq_halfcycles;

    double chips_to_halfcycles = Galileo_E1_SUB_CARRIER_A_RATE_HZ /
        Galileo_E1_CODE_CHIP_RATE_HZ * 2.0;


    subcarrier_freq_halfcycles = d_subcarrier_freq_cycles * chips_to_halfcycles;

    code_phase_step_chips = (static_cast<double>(d_code_freq_chips)) / (static_cast<double>(d_fs_in));
    subcarrier_phase_step_halfcycles = subcarrier_freq_halfcycles/ (static_cast<double>(d_fs_in));

    rem_code_phase_chips = d_rem_code_phase_samples * (d_code_freq_chips / d_fs_in);
    tcode_chips = - static_cast<double>(rem_code_phase_chips) + 1.0;


    tsubcarrier_phase_halfcyles = static_cast<double>(d_subcarrier_phase_halfcycles);

    early_late_subcarrier_spc_halfcycles = d_early_late_subcarrier_spc_cycles * chips_to_halfcycles;

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


void galileo_e1_prs_de_tracking_cc::update_local_code_prs()
{
    double tcode_chips;
    double tsubcarrier_phase_halfcyles;
    float rem_code_phase_chips;
    int associated_chip_index;
    int associated_subcarrier_index;
    int code_length_chips = static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS);
    double code_phase_step_chips;
    double subcarrier_phase_step_halfcycles;
    double early_late_subcarrier_spc_halfcycles;
    double subcarrier_freq_halfcycles;


    double chips_to_halfcycles = Galileo_E1_A_SUB_CARRIER_RATE_HZ /
        Galileo_E1_A_CODE_CHIP_RATE_HZ * 2.0;


    subcarrier_freq_halfcycles = d_subcarrier_freq_cycles_prs * 2.0;

    code_phase_step_chips = (static_cast<double>(d_code_freq_chips_prs)) / (static_cast<double>(d_fs_in));
    subcarrier_phase_step_halfcycles = subcarrier_freq_halfcycles/ (static_cast<double>(d_fs_in));

    // Update the local PRS code if necessary:
    uint64_t last_code_phase_store = d_start_index_prs_code + d_size_prs_code;

    uint64_t last_code_phase_required = d_integer_code_phase_chips_prs + static_cast< uint64_t >(
            d_fractional_code_phase_chips_prs + code_phase_step_chips*d_current_prn_length_samples );

    if( !d_prs_code_initialized || ( last_code_phase_required > last_code_phase_store - 2 ) )
    {
        // Make sure we go back at least 2 chips:
        d_start_index_prs_code = ( d_integer_code_phase_chips_prs < 2 ?
                d_integer_code_phase_chips_prs + d_prs_code_gen->get_code_length() - 2 :
                d_integer_code_phase_chips_prs - 2 );

        d_prs_code_gen->get_chips(d_start_index_prs_code, d_size_prs_code, d_prs_code_shorts );

        for( unsigned int ii = 0; ii < d_size_prs_code; ++ii )
        {
            d_prs_code[ii] = gr_complex( 1.0-2.0*static_cast<float>( d_prs_code_shorts[ii] ), 0.0f );
        }

        d_prs_code_initialized = true;

    }

    tcode_chips = d_fractional_code_phase_chips_prs +
        static_cast< double >( d_integer_code_phase_chips_prs - d_start_index_prs_code );
    //tcode_chips = d_code_phase_chips_prs - static_cast< double >( d_start_index_prs_code );

    // Add 1/4 of a cyle here to account for cosine phasing:
    tsubcarrier_phase_halfcyles = static_cast<double>(d_subcarrier_phase_halfcycles_prs) + 0.5;

    if( d_prs_sinusoidal_subcarrier )
    {
        early_late_subcarrier_spc_halfcycles = 0.5; // DPE requires quarter cycle spacing
    }
    else
    {
        early_late_subcarrier_spc_halfcycles = d_early_late_subcarrier_spc_cycles * 2.0;
    }

    int64_t early_code_phase_fxp = double_to_fxpt64( tcode_chips + d_early_late_code_spc_chips );
    int64_t prompt_code_phase_fxp = double_to_fxpt64( tcode_chips );
    int64_t late_code_phase_fxp = double_to_fxpt64( tcode_chips - d_early_late_code_spc_chips);


    int64_t code_phase_step_fxp = double_to_fxpt64( code_phase_step_chips );

    for (int i = 0; i < d_current_prn_length_samples; i++)
    {
        d_early_code_prs[i] = d_prs_code[ (early_code_phase_fxp >> 32 )];
        d_prompt_code_prs[i] = d_prs_code[ (prompt_code_phase_fxp >> 32 )];
        d_late_code_prs[i] = d_prs_code[ (late_code_phase_fxp >> 32 )];


        early_code_phase_fxp += code_phase_step_fxp;
        prompt_code_phase_fxp += code_phase_step_fxp;
        late_code_phase_fxp += code_phase_step_fxp;


    }

    if( d_prs_sinusoidal_subcarrier )
    {
        float phase_step_rad = subcarrier_phase_step_halfcycles * M_PI;
        float phase_rad = tsubcarrier_phase_halfcyles * M_PI;
        int phase_step_rad_i = gr::fxpt::float_to_fixed(phase_step_rad);
        int phase_rad_i = gr::fxpt::float_to_fixed(phase_rad);

        float sin_f, cos_f;
        for(int i = 0; i < d_current_prn_length_samples; i++)
        {
            gr::fxpt::sincos(phase_rad_i, &sin_f, &cos_f);
            d_prompt_subcarrier_prs[i] = std::complex<float>(cos_f, 0.0);
            d_early_subcarrier_prs[i] = std::complex<float>(sin_f, 0.0);
            phase_rad_i += phase_step_rad_i;
        }
    }
    else
    {
        int64_t early_subcarrier_phase_fxp = double_to_fxpt64(
                tsubcarrier_phase_halfcyles + early_late_subcarrier_spc_halfcycles );
        int64_t prompt_subcarrier_phase_fxp = double_to_fxpt64(
                tsubcarrier_phase_halfcyles );
        int64_t late_subcarrier_phase_fxp = double_to_fxpt64(
                tsubcarrier_phase_halfcyles - early_late_subcarrier_spc_halfcycles );

        int64_t subcarrier_phase_step_fxp = double_to_fxpt64( subcarrier_phase_step_halfcycles );
        for (int i = 0; i < d_current_prn_length_samples; i++)
        {
            d_early_subcarrier_prs[i] = (1.0 - 2.0*( (early_subcarrier_phase_fxp>>32)&0x01 ) );
            d_prompt_subcarrier_prs[i] = (1.0 - 2.0*( (prompt_subcarrier_phase_fxp>>32)&0x01 ) );
            d_late_subcarrier_prs[i] = (1.0 - 2.0*( (late_subcarrier_phase_fxp>>32)&0x01 ) );

            early_subcarrier_phase_fxp += subcarrier_phase_step_fxp;
            prompt_subcarrier_phase_fxp += subcarrier_phase_step_fxp;
            late_subcarrier_phase_fxp += subcarrier_phase_step_fxp;
        }
    }
    // Check for propagation:
    //double code_phase_at_end = d_code_phase_chips_prs + d_current_prn_length_samples*code_phase_step_chips;

    //double fxp_code_phase_at_end = static_cast< double >( prompt_code_phase_fxp ) *
        //std::pow( 2, -32 );

    //std::cerr << "PRN: " << d_acquisition_gnss_synchro->PRN
              //<< "Timestamp : " << 
}
void galileo_e1_prs_de_tracking_cc::update_local_carrier()
{
    float sin_f, cos_f;
    float phase_step_rad = static_cast<float>(2 * GALILEO_PI) * ( d_if_freq + d_carrier_doppler_hz ) / static_cast<float>(d_fs_in);
    int phase_step_rad_i = gr::fxpt::float_to_fixed(phase_step_rad);
    int phase_rad_i = gr::fxpt::float_to_fixed(d_rem_carr_phase_rad);

    for(int i = 0; i < d_current_prn_length_samples; i++)
        {
            gr::fxpt::sincos(phase_rad_i, &sin_f, &cos_f);
            d_carr_sign[i] = std::complex<float>(cos_f, -sin_f);
            phase_rad_i += phase_step_rad_i;
        }
}

galileo_e1_prs_de_tracking_cc::~galileo_e1_prs_de_tracking_cc()
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

    volk_free(d_early_code_prs);
    volk_free(d_prompt_code_prs);
    volk_free(d_late_code_prs);
    volk_free(d_early_subcarrier_prs);
    volk_free(d_prompt_subcarrier_prs);
    volk_free(d_late_subcarrier_prs);
    volk_free(d_Prompt_Subcarrier_Early_Code_prs);
    volk_free(d_Prompt_Subcarrier_Prompt_Code_prs);
    volk_free(d_Prompt_Subcarrier_Late_Code_prs);
    volk_free(d_Prompt_Code_Early_Subcarrier_prs);
    volk_free(d_Prompt_Code_Late_Subcarrier_prs);
    volk_free(d_prs_code);
    delete[] d_Prompt_buffer;
}



int galileo_e1_prs_de_tracking_cc::general_work (int noutput_items,gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    float carr_error_hz = 0.0;
    float carr_error_filt_hz = 0.0;
    float code_error_chips = 0.0;
    float subcarrier_error_cycles = 0.0;
    float code_error_filt_chips = 0.0;
    float subcarrier_error_filt_cycles = 0.0;
    double integer_subcarrier_periods = 0.0;
    float carr_error_hz_prs = 0.0;
    float carr_error_filt_hz_prs = 0.0;
    float code_error_chips_prs = 0.0;
    float subcarrier_error_cycles_prs = 0.0;
    float code_error_filt_chips_prs = 0.0;
    float subcarrier_error_filt_cycles_prs = 0.0;

    // Block input data and block output stream pointers
    const gr_complex* in = (gr_complex*) input_items[0];
    Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0];

    // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
    Gnss_Synchro current_synchro_data;

    int next_prn_length_samples = d_current_prn_length_samples;
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
                    // Now update the code and carrier phase estimates:
                    d_code_phase_chips = 0.0;
                    d_rem_code_phase_samples = 0.0;
                    d_subcarrier_phase_halfcycles = 0.0;

                    //std::cout<<" samples_offset="<<samples_offset<<"\r\n";
                    // Fill the acquisition data
                    current_synchro_data = *d_acquisition_gnss_synchro;
                    *out[0] = current_synchro_data;
                    consume_each(samples_offset); //shift input to perform alignment with local replica
                    return 1;
                }

            // Fill the acquisition data
            current_synchro_data = *d_acquisition_gnss_synchro;

            // Set the timestamp to the *start* of the epoch
            current_synchro_data.Tracking_timestamp_secs = (static_cast<double>(d_sample_counter) + static_cast<double>(d_rem_code_phase_samples)) / static_cast<double>(d_fs_in);

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

            // Now update the code and carrier phase estimates:
            double chips_to_halfcycles = Galileo_E1_SUB_CARRIER_A_RATE_HZ /
                Galileo_E1_CODE_CHIP_RATE_HZ * 2.0;
            double T = static_cast<double>( d_current_prn_length_samples ) / static_cast<double>( d_fs_in );
            d_code_phase_chips += T*d_code_freq_chips;
            d_code_phase_chips = std::fmod( d_code_phase_chips, Galileo_E1_B_CODE_LENGTH_CHIPS );
            d_carrier_phase_rad += T*2.0*M_PI*d_carrier_doppler_hz;
            //int64_t subcarrier_inc_halfcycles_fxp = double_to_fxpt64(
                    //d_subcarrier_freq_cycles*chips_to_halfcycles/d_fs_in, 48 );

            //int64_t subcarrier_delta_halfcycles_fxp = subcarrier_inc_halfcycles_fxp * d_current_prn_length_samples;
            //// take out the integer part:
            //subcarrier_delta_halfcycles_fxp &=0xFFFFFFFFFFFF; // 48 ones in LSB

            //d_subcarrier_phase_halfcycles += static_cast< double >( subcarrier_delta_halfcycles_fxp )*std::pow(2.0,-48.0);
            d_subcarrier_phase_halfcycles += T*d_subcarrier_freq_cycles*chips_to_halfcycles;
            integer_subcarrier_periods = std::floor( d_subcarrier_phase_halfcycles );
            d_subcarrier_phase_halfcycles = std::fmod( d_subcarrier_phase_halfcycles, 2.0 );
            double rem_subcarrier_phase_cycles = 1.0 - d_subcarrier_phase_halfcycles/2.0;

            if( rem_subcarrier_phase_cycles > 0.5 ){
                rem_subcarrier_phase_cycles -= 1.0;
            }

            d_rem_subcarrier_phase_samples = rem_subcarrier_phase_cycles*
                static_cast<double>(d_fs_in)/Galileo_E1_SUB_CARRIER_A_RATE_HZ;

            double rem_code_phase_chips = Galileo_E1_B_CODE_LENGTH_CHIPS - d_code_phase_chips;
            if( rem_code_phase_chips > Galileo_E1_B_CODE_LENGTH_CHIPS / 2.0 )
            {
                rem_code_phase_chips = ( rem_code_phase_chips - Galileo_E1_B_CODE_LENGTH_CHIPS );
            }

            d_rem_code_phase_samples = rem_code_phase_chips * d_fs_in/Galileo_E1_CODE_CHIP_RATE_HZ;

            //remnant carrier phase to prevent overflow in the code NCO
            d_rem_carr_phase_rad = d_rem_carr_phase_rad + GPS_TWO_PI * d_carrier_doppler_hz * T;
            d_rem_carr_phase_rad = fmod(d_rem_carr_phase_rad, GPS_TWO_PI);

            double chips_to_halfcycles_prs = Galileo_E1_A_SUB_CARRIER_RATE_HZ /
                Galileo_E1_A_CODE_CHIP_RATE_HZ * 2.0;
            // PRS tracking
            if( d_prs_tracking_enabled ){
                // Generate local code and carrier replicas (using \hat{f}_d(k-1))
                update_local_code_prs();
                //update_local_carrier();

                phase_as_complex = gr_complex( std::cos( d_rem_carr_phase_rad_prs ),
                        -std::sin( d_rem_carr_phase_rad_prs ) );

                carrier_doppler_inc_rad = 2.0*M_PI*(d_if_freq + d_carrier_doppler_hz_prs )/d_fs_in;

                phase_inc_as_complex = gr_complex( std::cos( carrier_doppler_inc_rad ),
                        -std::sin( carrier_doppler_inc_rad ) );


                // Two options: Double estimator or Double phase estimator
                // In the double estimator the subcarrier is assumed to be a square wave
                // In the double phase estiamtor the subcarrier is approximated by its fundamental
                // Fourier component
                //
                if( d_prs_sinusoidal_subcarrier ){

                    d_correlator.Carrier_rotate_and_DPE_volk(d_current_prn_length_samples,
                            in,
                            &phase_as_complex,
                            phase_inc_as_complex,
                            d_early_code_prs,
                            d_prompt_code_prs,
                            d_late_code_prs,
                            d_prompt_quadrature_subcarrier_prs,
                            d_prompt_subcarrier_prs,
                            d_Prompt_Subcarrier_Early_Code_prs,
                            d_Prompt_Subcarrier_Prompt_Code_prs,
                            d_Prompt_Subcarrier_Late_Code_prs,
                            d_Prompt_Code_Quadrature_Subcarrier_prs );
                }
                else
                {

                    // perform carrier wipe-off and compute Very Early, Early, Prompt, Late and Very Late correlation
                    d_correlator.Carrier_rotate_and_DE_volk(d_current_prn_length_samples,
                            in,
                            &phase_as_complex,
                            phase_inc_as_complex,
                            d_early_code_prs,
                            d_prompt_code_prs,
                            d_late_code_prs,
                            d_early_subcarrier_prs,
                            d_prompt_subcarrier_prs,
                            d_late_subcarrier_prs,
                            d_Prompt_Subcarrier_Early_Code_prs,
                            d_Prompt_Subcarrier_Prompt_Code_prs,
                            d_Prompt_Subcarrier_Late_Code_prs,
                            d_Prompt_Code_Early_Subcarrier_prs,
                            d_Prompt_Code_Late_Subcarrier_prs );
                }

                // Now update the code and carrier phase estimates:
                double delta_code_phase_prs = T*d_code_freq_chips_prs;

                d_fractional_code_phase_chips_prs += delta_code_phase_prs;

                d_integer_code_phase_chips_prs += static_cast< int64_t >(
                        std::floor( d_fractional_code_phase_chips_prs ) );

                d_fractional_code_phase_chips_prs = std::fmod(
                        d_fractional_code_phase_chips_prs, 1.0 );

                d_carrier_phase_rad_prs += T*2.0*M_PI*d_carrier_doppler_hz_prs;

                d_subcarrier_phase_halfcycles_prs += T*d_subcarrier_freq_cycles_prs*2.0;
                d_subcarrier_phase_halfcycles_prs = std::fmod( d_subcarrier_phase_halfcycles_prs, 2.0 );
                double rem_subcarrier_phase_cycles_prs = 1.0 - d_subcarrier_phase_halfcycles_prs/2.0;

                if( rem_subcarrier_phase_cycles_prs > 0.5 ){
                    rem_subcarrier_phase_cycles_prs -= 1.0;
                }

                d_rem_subcarrier_phase_samples_prs = rem_subcarrier_phase_cycles_prs*
                    static_cast<double>(d_fs_in)/Galileo_E1_A_SUB_CARRIER_RATE_HZ;

                int64_t e1b_code_length_prs_chips = static_cast< int64_t>(
                        Galileo_E1_B_CODE_LENGTH_CHIPS /
                        Galileo_E1_CODE_CHIP_RATE_HZ * Galileo_E1_A_CODE_CHIP_RATE_HZ
                        );


                int64_t chips_into_e1b_code_period =
                    d_integer_code_phase_chips_prs % e1b_code_length_prs_chips;

                double rem_code_phase_chips_prs = static_cast< double >(
                        e1b_code_length_prs_chips - chips_into_e1b_code_period
                        ) - d_fractional_code_phase_chips_prs;

                if( rem_code_phase_chips_prs > static_cast< double >(
                            e1b_code_length_prs_chips / 2 ) )
                {
                    rem_code_phase_chips_prs = ( rem_code_phase_chips_prs -
                            static_cast<double>(e1b_code_length_prs_chips) );
                }

                d_rem_code_phase_samples_prs = rem_code_phase_chips_prs * d_fs_in/Galileo_E1_A_CODE_CHIP_RATE_HZ;

                d_rem_carr_phase_rad_prs = d_rem_carr_phase_rad_prs + GPS_TWO_PI * d_carrier_doppler_hz_prs * T;
                d_rem_carr_phase_rad_prs = fmod(d_rem_carr_phase_rad_prs, GPS_TWO_PI);

            }

            // check for samples consistency (this should be done before in the receiver / here only if the source is a file)
            if (std::isnan((*d_Prompt_Subcarrier_Prompt_Code).real()) == true or
                    std::isnan((*d_Prompt_Subcarrier_Prompt_Code).imag()) == true ) // or std::isinf(in[i].real())==true or std::isinf(in[i].imag())==true)
                {
                    const int samples_available = ninput_items[0];
                    d_sample_counter = d_sample_counter + samples_available;
                    LOG(WARNING) << "Detected NaN samples at sample number " << d_sample_counter;
                    consume_each(samples_available);

                    // make an output to not stop the rest of the processing blocks
                    current_synchro_data.Prompt_I = 0.0;
                    current_synchro_data.Prompt_Q = 0.0;
                    current_synchro_data.Tracking_timestamp_secs = static_cast<double>(d_sample_counter) / static_cast<double>(d_fs_in);
                    current_synchro_data.Carrier_phase_rads = 0.0;
                    current_synchro_data.Code_phase_secs = 0.0;
                    current_synchro_data.CN0_dB_hz = 0.0;
                    current_synchro_data.Flag_valid_tracking = false;
                    current_synchro_data.Flag_valid_pseudorange = false;

                    *out[0] = current_synchro_data;
                    return 1;
                }

            // consume the input samples:
            d_sample_counter += d_current_prn_length_samples;

            // ################## PLL ##########################################################
            // PLL discriminator
            carr_error_hz = pll_cloop_two_quadrant_atan(*d_Prompt_Subcarrier_Prompt_Code) / static_cast<float>(GPS_TWO_PI);
            // Carrier discriminator filter
            carr_error_filt_hz = d_carrier_loop_filter.apply(carr_error_hz);
            // New carrier Doppler frequency estimation
            d_carrier_doppler_hz = carr_error_filt_hz;

            float code_doppler_chips = ((d_carrier_doppler_hz * Galileo_E1_CODE_CHIP_RATE_HZ) / Galileo_E1_FREQ_HZ);

            // New subcarrier Doppler frequency estimation: carrier
            // aiding of the subcarrier:
            if( d_aid_subcarrier_with_carrier )
            {
                d_subcarrier_freq_cycles = Galileo_E1_CODE_CHIP_RATE_HZ + code_doppler_chips;
            }
            else
            {
                d_subcarrier_freq_cycles = Galileo_E1_CODE_CHIP_RATE_HZ;
            }
            //carrier phase accumulator for (K) Doppler estimation
            d_acc_carrier_phase_rad = d_acc_carrier_phase_rad + GPS_TWO_PI * d_carrier_doppler_hz * Galileo_E1_CODE_PERIOD;

            // ################## SLL ##########################################################
            // SLL discriminator
            subcarrier_error_cycles = dll_nc_e_minus_l_normalized(
                    *d_Prompt_Code_Early_Subcarrier,
                    *d_Prompt_Code_Late_Subcarrier); //[chips/Ti]
            // normalise the SLL discriminator by the slope of the
            // BOC(1,1) at the origin:
            float corr_slope = 4.0;
            subcarrier_error_cycles *= 2.0*( 1 - corr_slope*d_early_late_subcarrier_spc_cycles) / corr_slope;
            // Subcarrier discriminator filter
            subcarrier_error_filt_cycles = d_subcarrier_loop_filter.apply(subcarrier_error_cycles); //[chips/second]
            d_subcarrier_freq_cycles += subcarrier_error_filt_cycles;
            // Aiding the code tracking with the subcarrier:
            if( d_aid_code_with_subcarrier )
            {
                d_code_freq_chips = d_subcarrier_freq_cycles;
            }
            else if( d_aid_subcarrier_with_carrier )
            {
                d_code_freq_chips = Galileo_E1_CODE_CHIP_RATE_HZ + code_doppler_chips;
            }
            else
            {
                d_code_freq_chips = Galileo_E1_CODE_CHIP_RATE_HZ;
            }

            //Subcarrier phase accumulator
            float subcarrier_error_filt_secs;
            subcarrier_error_filt_secs = (Galileo_E1_CODE_PERIOD * subcarrier_error_filt_cycles) / Galileo_E1_CODE_CHIP_RATE_HZ; //[seconds]
            //code_error_filt_secs=T_prn_seconds*code_error_filt_chips*T_chip_seconds*static_cast<float>(d_fs_in); //[seconds]

            // ################## DLL ##########################################################
            // DLL discriminator
            code_error_chips = dll_nc_e_minus_l_normalized(
                    *d_Prompt_Subcarrier_Early_Code,
                    *d_Prompt_Subcarrier_Late_Code); //[chips/Ti]
            //Normalise the code phase error:
            corr_slope = 1.0;
            code_error_chips *= 2.0*( 1 - corr_slope*d_early_late_code_spc_chips) / corr_slope;
            // Code discriminator filter
            code_error_filt_chips = d_code_loop_filter.apply(code_error_chips); //[chips/second]
            //Code phase accumulator
            d_code_freq_chips += code_error_filt_chips;
            float code_error_filt_secs;
            code_error_filt_secs = (Galileo_E1_CODE_PERIOD * code_error_filt_chips) / Galileo_E1_CODE_CHIP_RATE_HZ; //[seconds]
            //code_error_filt_secs=T_prn_seconds*code_error_filt_chips*T_chip_seconds*static_cast<float>(d_fs_in); //[seconds]
            d_acc_code_phase_secs = d_acc_code_phase_secs  + code_error_filt_secs;

            // ################## PRS ##########################################################
            if( d_prs_tracking_enabled )
            {
                // ################## PLL ##########################################################
                // PLL discriminator
                carr_error_hz_prs = pll_cloop_two_quadrant_atan(*d_Prompt_Subcarrier_Prompt_Code_prs) / static_cast<float>(GPS_TWO_PI);

                // Carrier discriminator filter
                carr_error_filt_hz_prs = d_carrier_loop_filter_prs.apply(carr_error_hz_prs);

                // New carrier Doppler frequency estimation
                d_carrier_doppler_hz_prs = carr_error_filt_hz_prs;

                float subcarrier_doppler_cycles_prs = ((d_carrier_doppler_hz_prs *
                            Galileo_E1_A_SUB_CARRIER_RATE_HZ) / Galileo_E1_FREQ_HZ);

                // New subcarrier Doppler frequency estimation: carrier
                // aiding of the subcarrier:
                if( d_aid_subcarrier_with_carrier )
                {
                    d_subcarrier_freq_cycles_prs = Galileo_E1_A_SUB_CARRIER_RATE_HZ + subcarrier_doppler_cycles_prs;
                }
                else
                {
                    d_subcarrier_freq_cycles_prs = Galileo_E1_A_SUB_CARRIER_RATE_HZ;
                }

                // ################## SLL ##########################################################
                // SLL discriminator
                if( d_prs_sinusoidal_subcarrier )
                {
                    gr_complex Z = (
                            *d_Prompt_Code_Quadrature_Subcarrier_prs /
                            *d_Prompt_Subcarrier_Prompt_Code_prs );

                    subcarrier_error_cycles_prs = std::atan( Z.real() ) / static_cast<float>(GPS_TWO_PI);
                }
                else
                {
                    subcarrier_error_cycles_prs = dll_nc_e_minus_l_normalized(
                            *d_Prompt_Code_Early_Subcarrier_prs,
                            *d_Prompt_Code_Late_Subcarrier_prs); //[chips/Ti]

                    // normalise the SLL discriminator by the slope of the
                    // BOC(1,1) at the origin:
                    corr_slope = 4.0;
                    subcarrier_error_cycles_prs *= 2.0*( 1 - corr_slope*d_early_late_subcarrier_spc_cycles) / corr_slope;
                }

                // Subcarrier discriminator filter
                subcarrier_error_filt_cycles_prs = d_subcarrier_loop_filter_prs.apply(subcarrier_error_cycles_prs); //[chips/second]

                // For now use the E1b values
                d_subcarrier_freq_cycles_prs += subcarrier_error_filt_cycles_prs;

                // ################## DLL ##########################################################
                // DLL discriminator
                code_error_chips_prs = dll_nc_e_minus_l_normalized(
                        *d_Prompt_Subcarrier_Early_Code_prs,
                        *d_Prompt_Subcarrier_Late_Code_prs); //[chips/Ti]
                //Normalise the code phase error:
                corr_slope = 1.0;
                code_error_chips_prs *= 2.0*( 1 - corr_slope*d_early_late_code_spc_chips) / corr_slope;

                // Code discriminator filter
                code_error_filt_chips_prs = d_code_loop_filter_prs.apply(code_error_chips_prs); //[chips/second]

                // Aiding the code tracking with the subcarrier:
                if( d_aid_code_with_subcarrier )
                {
                    d_code_freq_chips_prs = d_subcarrier_freq_cycles_prs *
                        Galileo_E1_A_CODE_CHIP_RATE_HZ/Galileo_E1_A_SUB_CARRIER_RATE_HZ;
                }
                else if( d_aid_subcarrier_with_carrier )
                {
                    double code_doppler_chips_prs = ((d_carrier_doppler_hz_prs *
                                Galileo_E1_A_CODE_CHIP_RATE_HZ) / Galileo_E1_FREQ_HZ);

                    d_code_freq_chips_prs = Galileo_E1_A_CODE_CHIP_RATE_HZ + code_doppler_chips_prs;
                }
                else
                {
                    d_code_freq_chips_prs = Galileo_E1_A_CODE_CHIP_RATE_HZ;
                }
                // For now use the E1 values:
                d_code_freq_chips_prs += code_error_filt_chips_prs;

            }

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
            if( d_prs_tracking_enabled )
            {
                T_chip_seconds = 1 / static_cast<double>(d_code_freq_chips_prs);
                T_prn_seconds = T_chip_seconds * Galileo_E1_B_CODE_LENGTH_CHIPS *
                    Galileo_E1_A_CODE_CHIP_RATE_HZ/Galileo_E1_CODE_CHIP_RATE_HZ;
                T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
                K_blk_samples = T_prn_samples + d_rem_code_phase_samples_prs; // + code_error_filt_secs * static_cast<double>(d_fs_in);
            }
            else
            {
                T_chip_seconds = 1 / static_cast<double>(d_code_freq_chips);
                T_prn_seconds = T_chip_seconds * Galileo_E1_B_CODE_LENGTH_CHIPS;
                T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
                K_blk_samples = T_prn_samples + d_rem_code_phase_samples; // + code_error_filt_secs * static_cast<double>(d_fs_in);
            }


            T_sc_seconds = 1 / static_cast<double>(d_subcarrier_freq_cycles);
            // THere is one subcarrier period per code chip:
            T_sc_prn_seconds = T_sc_seconds * Galileo_E1_B_CODE_LENGTH_CHIPS;
            T_sc_prn_samples = T_sc_prn_seconds * static_cast<double>(d_fs_in);
            K_sc_samples = T_sc_prn_samples + d_rem_subcarrier_phase_samples; 

            next_prn_length_samples = round(K_blk_samples); //round to a discrete samples
            //d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample

            // ####### CN0 ESTIMATION AND LOCK DETECTORS ######
            if (d_cn0_estimation_counter < CN0_ESTIMATION_SAMPLES)
                {
                    // fill buffer with prompt correlator output values
                    d_Prompt_buffer[d_cn0_estimation_counter] = ( d_prs_tracking_enabled ?
                            *d_Prompt_Subcarrier_Prompt_Code_prs :
                            *d_Prompt_Subcarrier_Prompt_Code );
                    d_cn0_estimation_counter++;

                    d_mean_code_error += std::fabs( code_error_chips );

                    if( d_prs_tracking_enabled )
                    {
                        d_mean_code_error_prs += std::fabs( code_error_chips_prs );
                    }
                }
            else
                {
                    d_cn0_estimation_counter = 0;

                    d_mean_code_error /= static_cast<double>( CN0_ESTIMATION_SAMPLES );
                    d_mean_code_error_prs /= static_cast<double>( CN0_ESTIMATION_SAMPLES );

                    // Code lock indicator
                    d_CN0_SNV_dB_Hz = cn0_svn_estimator(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES, d_fs_in, d_current_prn_length_samples);

                    // Carrier lock indicator
                    d_carrier_lock_test = carrier_lock_detector(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES);

                    // Loss of lock detection
                    if (d_carrier_lock_test < d_carrier_lock_threshold or d_CN0_SNV_dB_Hz < MINIMUM_VALID_CN0)
                        {
                            d_carrier_lock_fail_counter++;
                            d_carrier_lock_success_counter = 0;
                        }
                    else
                        {
                            d_carrier_lock_success_counter++;
                            if (d_carrier_lock_fail_counter > 0) d_carrier_lock_fail_counter--;
                        }
                    if( not d_carrier_locked )
                    {
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
                                d_prs_tracking_enabled = false;
                                d_tow_received = false;
                            }

                        if( d_carrier_lock_success_counter > MINIMUM_LOCK_SUCCESS_COUNTER )
                        {
                            LOG(INFO) << "Phase lock achieved in channel " << d_channel;
                            d_carrier_locked = true;
                            d_code_loop_filter.set_noise_bandwidth( d_final_dll_bw_hz );
                            d_subcarrier_loop_filter.set_noise_bandwidth( d_final_sll_bw_hz );
                            d_carrier_loop_filter.set_noise_bandwidth( d_final_pll_bw_hz );
                            d_early_late_code_spc_chips = d_final_early_late_code_space_chips;

                            d_code_loop_filter.initialize( code_error_filt_chips );
                            d_subcarrier_loop_filter.initialize( subcarrier_error_filt_cycles );
                            d_carrier_loop_filter.initialize( carr_error_filt_hz );

                            d_carrier_lock_fail_counter = 0;

                        }
                    }
                    else // not d_carrier_locked
                    {
                        if (d_carrier_lock_fail_counter > MAXIMUM_LOCK_FAIL_COUNTER)
                            {
                                LOG(INFO) << "Loss of carrier lock in channel "
                                          << d_channel << "! Reverting to initial tracking state";
                                d_carrier_lock_fail_counter = 0;
                                d_carrier_locked = false;
                                d_code_loop_filter.set_noise_bandwidth( d_initial_dll_bw_hz );
                                d_subcarrier_loop_filter.set_noise_bandwidth( d_initial_sll_bw_hz );
                                d_carrier_loop_filter.set_noise_bandwidth( d_initial_pll_bw_hz );
                                d_early_late_code_spc_chips = d_initial_early_late_code_space_chips;
                                d_code_loop_filter.initialize( code_error_filt_chips );
                                d_carrier_loop_filter.initialize( carr_error_filt_hz );
                                d_subcarrier_loop_filter.initialize( subcarrier_error_filt_cycles );
                            }
                        else
                        {
                            if( d_code_locked )
                            {
                                if( d_mean_code_error > 0.1 )
                                {
                                    d_code_locked = false;

                                    d_code_loop_filter.set_noise_bandwidth(
                                            d_initial_dll_bw_hz );

                                    std::stringstream ss("");

                                    ss << "Loss of code lock in channel "
                                        << d_channel << "!"
                                        << "[PRN: " << d_acquisition_gnss_synchro->PRN
                                        << ". @ " << static_cast< double >( d_sample_counter )/
                                        static_cast<double>( d_fs_in )
                                        << "]";

                                    LOG(INFO) << ss.str();

                                    std::cout << ss.str() << std::endl;;
                                }
                            }
                            else // if d_code_locked
                            {
                                if( d_mean_code_error < 0.05 )
                                {
                                    d_code_locked = true;
                                    d_code_loop_filter.set_noise_bandwidth(
                                            d_final_dll_bw_hz );

                                    std::stringstream ss("");

                                    ss << "Code lock achieved in channel "
                                        << d_channel << "!"
                                        << "[PRN: " << d_acquisition_gnss_synchro->PRN
                                        << ". @ " << static_cast< double >( d_sample_counter )/
                                        static_cast<double>( d_fs_in )
                                        << "]";

                                    LOG(INFO) << ss.str();

                                    std::cout << ss.str() << std::endl;;
                                    // Try to enable prs tracking:
                                    start_tracking_prs();
                                }
                            }

                            if( d_prs_tracking_enabled )
                            {

                                if( d_code_locked_prs )
                                {
                                    if( d_mean_code_error_prs*chips_to_halfcycles_prs > 1.0 )
                                    {
                                        d_code_locked_prs = false;

                                        d_code_loop_filter_prs.set_noise_bandwidth(
                                                d_initial_dll_bw_hz );

                                        std::stringstream ss("");

                                        ss << "PRS Loss of code lock in channel "
                                            << d_channel << "!"
                                            << "[PRN: " << d_acquisition_gnss_synchro->PRN
                                            << ". @ " << static_cast< double >( d_sample_counter )/
                                            static_cast<double>( d_fs_in )
                                            << "]";

                                        LOG(INFO) << ss.str();

                                        std::cout << ss.str() << std::endl;;
                                    }
                                }
                                else // if d_code_locked
                                {
                                    if( d_mean_code_error_prs*chips_to_halfcycles_prs < 0.2 )
                                    {
                                        d_code_locked_prs = true;

                                        d_code_loop_filter_prs.set_noise_bandwidth(
                                                d_final_dll_bw_hz );

                                        std::stringstream ss("");

                                        ss << "PRS Code lock achieved in channel "
                                            << d_channel << "!"
                                            << "[PRN: " << d_acquisition_gnss_synchro->PRN
                                            << ". @ " << static_cast< double >( d_sample_counter )/
                                            static_cast<double>( d_fs_in )
                                            << "]";

                                        LOG(INFO) << ss.str();

                                        std::cout << ss.str() << std::endl;;
                                    }
                                }

                            }

                        } // Carrier lock fail counter > threshold

                    } // else if not carrier locked

                    d_mean_code_error = 0.0;
                    d_mean_code_error_prs = 0.0;
                } // cno estimation

            // ########### Output the tracking results to Telemetry block ##########

            current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt_Subcarrier_Prompt_Code).real());
            current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt_Subcarrier_Prompt_Code).imag());

            // Tracking_timestamp_secs is aligned with the NEXT PRN start sample (Hybridization problem!)
            //compute remnant code phase samples BEFORE the Tracking timestamp
            //d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample
            //current_synchro_data.Tracking_timestamp_secs = ((double)d_sample_counter +
            //        (double)d_current_prn_length_samples + (double)d_rem_code_phase_samples) / static_cast<double>(d_fs_in);

            // Tracking_timestamp_secs is aligned with the CURRENT PRN start sample (Hybridization OK!, but some glitches??)
            //current_synchro_data.Tracking_timestamp_secs = (static_cast<double>(d_sample_counter) + static_cast<double>(d_rem_code_phase_samples)) / static_cast<double>(d_fs_in);
            //compute remnant code phase samples AFTER the Tracking timestamp
            //d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample

            //d_rem_subcarrier_phase_samples = K_sc_samples - d_current_prn_length_samples; //rounding error < 1 sample
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
        d_sample_counter += d_current_prn_length_samples;
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
                    tmp_float = static_cast<float>(d_acc_carrier_phase_rad);
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));
                    // carrier and code frequency
                    d_dump_file.write(reinterpret_cast<char*>(&d_carrier_doppler_hz), sizeof(float));
                    tmp_float = static_cast<float>( d_code_freq_chips );
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));
                    //PLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&carr_error_hz), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&carr_error_filt_hz), sizeof(float));
                    //DLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&code_error_chips), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&code_error_filt_chips), sizeof(float));
                    // SLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&subcarrier_error_cycles), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&subcarrier_error_filt_cycles), sizeof(float));
                    // CN0 and carrier lock test
                    d_dump_file.write(reinterpret_cast<char*>(&d_CN0_SNV_dB_Hz), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&d_carrier_lock_test), sizeof(float));
                    // AUX vars (for debug purposes)
                    //tmp_float = d_rem_code_phase_samples/static_cast< float >( d_fs_in )*Galileo_E1_CODE_CHIP_RATE_HZ;
                    tmp_float = d_code_phase_chips;
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));
                    //tmp_double = static_cast<double>(d_sample_counter + d_current_prn_length_samples);
                    tmp_double = integer_subcarrier_periods;
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                    //tmp_float = d_rem_subcarrier_phase_samples/static_cast<float>( d_fs_in ) * Galileo_E1_CODE_CHIP_RATE_HZ;
                    tmp_float = d_subcarrier_phase_halfcycles/2.0;
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));


                    // ****************************************************************************
                    // PRS Variables:
                    prompt_I = (*d_Prompt_Subcarrier_Prompt_Code_prs).real();
                    prompt_Q = (*d_Prompt_Subcarrier_Prompt_Code_prs).imag();
                    if( d_prs_sinusoidal_subcarrier )
                    {
                        tmp_VE = d_Prompt_Code_Quadrature_Subcarrier_prs->real();
                        tmp_VL = d_Prompt_Code_Quadrature_Subcarrier_prs->imag();
                    }
                    else
                    {
                        tmp_VE = std::abs<float>(*d_Prompt_Code_Early_Subcarrier_prs);
                        tmp_VL = std::abs<float>(*d_Prompt_Code_Late_Subcarrier_prs);
                    }
                    tmp_E = std::abs<float>(*d_Prompt_Subcarrier_Early_Code_prs);
                    tmp_P = std::abs<float>(*d_Prompt_Subcarrier_Prompt_Code_prs);
                    tmp_L = std::abs<float>(*d_Prompt_Subcarrier_Late_Code_prs);
                    // Dump correlators output
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_VE), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_E), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_P), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_L), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_VL), sizeof(float));
                    // PROMPT I and Q (to analyze navigation symbols)
                    d_dump_file.write(reinterpret_cast<char*>(&prompt_I), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&prompt_Q), sizeof(float));
                    // carrier and code frequency
                    d_dump_file.write(reinterpret_cast<char*>(&d_carrier_doppler_hz_prs), sizeof(float));
                    tmp_float = static_cast<float>(d_code_freq_chips_prs);
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));
                    tmp_float = static_cast<float>(d_subcarrier_freq_cycles_prs);
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));
                    //PLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&carr_error_hz_prs), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&carr_error_filt_hz_prs), sizeof(float));
                    //DLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&code_error_chips_prs), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&code_error_filt_chips_prs), sizeof(float));
                    // SLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&subcarrier_error_cycles_prs), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&subcarrier_error_filt_cycles_prs), sizeof(float));

                    tmp_double = static_cast< double >( d_integer_code_phase_chips_prs ) +
                        d_fractional_code_phase_chips_prs;
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                    tmp_float = d_subcarrier_phase_halfcycles_prs/2.0;
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));
            }
            catch (std::ifstream::failure e)
            {
                    LOG(WARNING) << "Exception writing trk dump file " << e.what() << std::endl;
            }
        }
    consume_each(d_current_prn_length_samples); // this is required for gr_block derivates
    //d_sample_counter += d_current_prn_length_samples; //count for the processed samples
    d_current_prn_length_samples = next_prn_length_samples;
    //std::cout<<"Galileo tracking output at sample "<<d_sample_counter<<std::endl;
    return 1; //output tracking result ALWAYS even in the case of d_enable_tracking==false
}



void galileo_e1_prs_de_tracking_cc::set_channel(unsigned int channel)
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



void galileo_e1_prs_de_tracking_cc::set_channel_queue(concurrent_queue<int> *channel_internal_queue)
{
    d_channel_internal_queue = channel_internal_queue;
}



void galileo_e1_prs_de_tracking_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
    //  Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    //DLOG(INFO) << "Tracking code phase set to " << d_acq_code_phase_samples;
    //DLOG(INFO) << "Tracking carrier doppler set to " << d_acq_carrier_doppler_hz;
    //DLOG(INFO) << "Tracking Satellite set to " << d_satellite;
}

void galileo_e1_prs_de_tracking_cc::start_tracking_prs()
{

    if( d_prs_tracking_enabled ){
        return;
    }
    // Initialise the code/phase and subcarrier estimates:
    // Two possibilities:
    // 1) We have decoded TOW from the nav message
    // 2) We have rx time and a preamble in which case we can estimate TOW:
    double timestamp_last_tow = d_timestamp_last_tow;
    double last_tow = d_last_tow;

    if( d_tow_received )
    {
        timestamp_last_tow = d_timestamp_last_tow;
        last_tow = d_last_tow;

    }
    else if( d_rx_time_set && d_preamble_start_detected )
    {
        double t_preamble = std::floor( ( d_tow_rx_time +
                    ( d_preamble_timestamp - d_timestamp_rx_time ) ) + 0.5 );

        last_tow = t_preamble;
        timestamp_last_tow = d_preamble_timestamp;
    }
    else
    {
        DLOG(INFO) << "Attempted to initialise PRS tracking with insufficient timing information. Bailing.";
        return;
    }

    double time_since_tow = static_cast< double >( d_sample_counter ) /
        static_cast<double>( d_fs_in ) - timestamp_last_tow;

    double code_periods_since_tow = std::floor( time_since_tow / Galileo_E1_CODE_PERIOD + 0.5);

    double last_tow_round = std::ceil( last_tow/Galileo_E1_CODE_PERIOD ) * Galileo_E1_CODE_PERIOD;

    double curr_tow = std::floor( 
            (last_tow + time_since_tow)/Galileo_E1_CODE_PERIOD
            + 0.5 ) * Galileo_E1_CODE_PERIOD
        + d_rem_code_phase_samples/static_cast<double>(d_fs_in);

    // Handle week rollover:
    if( curr_tow > 604800.0 ){
        curr_tow -= 604800.0;
    }

    double code_phase_chips_prs = std::fmod( curr_tow * Galileo_E1_A_CODE_CHIP_RATE_HZ,
            d_prs_code_gen->get_code_length() );

    d_integer_code_phase_chips_prs = static_cast< int64_t >( std::floor( code_phase_chips_prs ) );

    d_fractional_code_phase_chips_prs = std::fmod( code_phase_chips_prs, 1.0 );

    d_subcarrier_phase_halfcycles_prs = d_fractional_code_phase_chips_prs *
        Galileo_E1_A_SUB_CARRIER_RATE_HZ / Galileo_E1_A_CODE_CHIP_RATE_HZ * 2.0;

    d_subcarrier_phase_halfcycles_prs = std::fmod( d_subcarrier_phase_halfcycles_prs, 2.0 );

    d_rem_carr_phase_rad_prs = d_rem_carr_phase_rad - M_PI/2.0;

    d_code_freq_chips_prs = d_code_freq_chips * Galileo_E1_A_CODE_CHIP_RATE_HZ /
        Galileo_E1_CODE_CHIP_RATE_HZ;

    d_subcarrier_freq_cycles_prs = d_subcarrier_freq_cycles * Galileo_E1_A_SUB_CARRIER_RATE_HZ /
        Galileo_E1_SUB_CARRIER_A_RATE_HZ;

    d_carrier_doppler_hz_prs = d_carrier_doppler_hz;


    // Initialise the filters:
    // DLL/PLL filter Initialization
    d_code_loop_filter_prs.set_noise_bandwidth( d_initial_dll_bw_hz );
    d_subcarrier_loop_filter_prs.set_noise_bandwidth( d_final_sll_bw_hz );
    d_carrier_loop_filter_prs.set_noise_bandwidth( d_final_pll_bw_hz );

    //d_early_late_code_spc_chips = d_initial_early_late_code_space_chips;

    d_carrier_loop_filter_prs.initialize(d_carrier_doppler_hz_prs); // initialize the carrier filter

    d_subcarrier_loop_filter_prs.initialize(
            d_aid_subcarrier_with_carrier ? 0.0 : d_subcarrier_freq_cycles_prs); // initialize the subcarrier filter
    d_code_loop_filter_prs.initialize(
            ( d_aid_subcarrier_with_carrier || d_aid_code_with_subcarrier ) ?
            0.0 : d_code_freq_chips_prs);    // initialize the code filter


    std::string sys_ = &d_acquisition_gnss_synchro->System;
    sys = sys_.substr(0, 1);

    // DEBUG OUTPUT
    std::cout << "PRS tracking start on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
    LOG(INFO) << "Starting tracking of PRS for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;

    DLOG(INFO) << "Starting params: current TOW estimate: " << std::fixed << std::setprecision(12) <<curr_tow
               << " Code phase " << d_integer_code_phase_chips_prs
               << std::setw(6) << std::fixed << d_fractional_code_phase_chips_prs << " chips."
               << " Subcarrier Phase " << ( d_subcarrier_phase_halfcycles_prs*2.0 ) << " cycles"
               << " Last TOW: " << std::fixed << std::setprecision(12) << last_tow
               << " @ " << timestamp_last_tow
               << " Correction: " << (curr_tow - last_tow)
               << " Current Time: " << d_sample_counter/static_cast<double>(d_fs_in);
    // enable tracking
    d_prs_tracking_enabled = true;
    d_prs_code_gen->set_prn( d_acquisition_gnss_synchro->PRN );
    d_code_locked_prs = false;
    d_cn0_estimation_counter = 0;
    d_mean_code_error_prs = 0.0;

    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz_prs
              << " PULL-IN Code Phase [chips]=" << d_integer_code_phase_chips_prs
              << std::setw(6) << std::fixed << d_fractional_code_phase_chips_prs;
}

void galileo_e1_prs_de_tracking_cc::handle_gnss_message( pmt::pmt_t msg )
{
    std::string telem_msg = gnss_message::get_message( msg );

    std::stringstream log_str("");

    log_str << "Received message " << telem_msg
               << " with timestamp: " << gnss_message::get_timestamp( msg );

    pmt::pmt_t not_found;

    if( gnss_message::get_message( msg ) == "TOW_ACQUIRED" ){
        d_tow_received = true;
        d_last_tow = pmt::to_double( pmt::dict_ref( msg, pmt::mp( "TOW" ), not_found ) ) ;
        log_str << ". TOW: " << std::fixed << std::setprecision(12) << d_last_tow;
        d_timestamp_last_tow = gnss_message::get_timestamp( msg );

        if( d_code_locked and !d_prs_tracking_enabled )
        {
            log_str << ". Enabling PRS tracking.";
            start_tracking_prs();
        }
    }

    if( gnss_message::get_message( msg ) == "RECEIVER_TIME_SET" )
    {
        d_rx_time_set = true;
        d_tow_rx_time = pmt::to_double( pmt::dict_ref( msg, pmt::mp( "TOW" ), not_found ) );
        d_timestamp_rx_time = gnss_message::get_timestamp( msg );

        log_str << ". TOW: " << d_tow_rx_time;
    }

    if( gnss_message::get_message( msg ) == "PREAMBLE_START_DETECTED" )
    {
        d_preamble_start_detected = true;
        d_preamble_timestamp = gnss_message::get_timestamp( msg );
    }

    if( !d_prs_tracking_enabled && ( d_preamble_start_detected && d_rx_time_set ) )
    {
        log_str << ". Enabling PRS tracking with 1 s ambiguity resolution";
        start_tracking_prs();
    }

    LOG(INFO) << log_str.str();


}
