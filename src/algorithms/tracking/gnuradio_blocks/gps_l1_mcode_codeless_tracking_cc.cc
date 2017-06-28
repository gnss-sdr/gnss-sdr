/*!
 * \file gps_l1_mcode_codeless_tracking_cc.cc
 * \brief Implementation of a code DLL + carrier PLL VEML (Very Early
 *  Minus Late) tracking block for GPS L1 signals plus codeless tracking of
 *  the M-Code signal
 * \author Cillian O'Driscoll, 2017. cillian.odriscoll(at)gmail.com
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * [1] K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * Codeless approach described in:
 * [2] D. Borio, M. Rao and C. O'Driscoll,
 * Codeless Processing of BOC Modulated Signals, IET Radar, Sonar and Navigation,
 * vol 7, no. 2, pp 143-152, 2013.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#include "gps_l1_mcode_codeless_tracking_cc.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <gnuradio/io_signature.h>
#include <gnuradio/fxpt.h>  // fixed point sine and cosine
#include <glog/logging.h>
#include "gnss_synchro.h"
#include "gps_sdr_signal_processing.h"
#include "tracking_discriminators.h"
#include "lock_detectors.h"
#include "GPS_L1_CA.h"
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

gps_l1_mcode_codeless_tracking_cc_sptr
gps_l1_mcode_codeless_make_tracking_cc(
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
        float initial_early_late_code_space_cycles,
        float final_early_late_code_space_cycles,
        float initial_very_early_late_code_space_chips,
        float final_very_early_late_code_space_chips,
        bool aid_code_with_carrier,
        bool use_bump_jumping,
        unsigned int bump_jumping_threshold,
        float initial_divergence_bw_hz,
        float final_divergence_bw_hz,
        int mcode_accumulation_length,
        bool close_mcode_loops,
        float pll_bw_hz_mcode,
        float dll_bw_hz_mcode
        )
{
    return gps_l1_mcode_codeless_tracking_cc_sptr(new gps_l1_mcode_codeless_tracking_cc(if_freq,
            fs_in, vector_length, queue, dump, dump_filename,
            pll_loop_order, pll_initial_bw_hz, pll_final_bw_hz,
            dll_loop_order, dll_initial_bw_hz, dll_final_bw_hz,
            initial_early_late_code_space_cycles,
            final_early_late_code_space_cycles,
            initial_very_early_late_code_space_chips,
            final_very_early_late_code_space_chips,
            aid_code_with_carrier,
            use_bump_jumping, bump_jumping_threshold,
            initial_divergence_bw_hz, final_divergence_bw_hz,
            mcode_accumulation_length,
            close_mcode_loops,
            pll_bw_hz_mcode,
            dll_bw_hz_mcode
            ));
}


void gps_l1_mcode_codeless_tracking_cc::forecast (int noutput_items,
        gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = static_cast<int>(d_vector_length) * 2; //set the required available samples in each call
}


gps_l1_mcode_codeless_tracking_cc::gps_l1_mcode_codeless_tracking_cc(
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
        float initial_early_late_code_space_chips,
        float final_early_late_code_space_chips,
        float initial_very_early_late_code_space_chips,
        float final_very_early_late_code_space_chips,
        bool aid_code_with_carrier,
        bool use_bump_jumping,
        unsigned int bump_jumping_threshold,
        float initial_divergence_bw_hz,
        float final_divergence_bw_hz, int mcode_accumulation_length,
        bool close_mcode_loops,
        float pll_bw_hz_mcode,
        float dll_bw_hz_mcode):
        gr::block("gps_l1_mcode_codeless_tracking_cc", gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    DLOG(INFO) << "Initialising mcode Codeless Tracking: " << std::endl
               << "\t pll_loop_order: " << pll_loop_order << std::endl
               << "\t pll_initial_bw_hz: " << pll_initial_bw_hz << std::endl
               << "\t pll_final_bw_hz:   " << pll_final_bw_hz << std::endl
               << "\t dll_loop_order:   " << dll_loop_order << std::endl
               << "\t dll_initial_bw_hz:   " << dll_initial_bw_hz << std::endl 
               << "\t dll_final_bw_hz:   " << dll_final_bw_hz << std::endl
               << "\t initial_early_late_code_space_chips:   " << initial_early_late_code_space_chips << std::endl
               << "\t final_early_late_code_space_chips:   " << final_early_late_code_space_chips << std::endl
               << "\t initial_very_early_late_code_space_chips:   " << initial_very_early_late_code_space_chips << std::endl
               << "\t final_very_early_late_code_space_chips:   " << final_very_early_late_code_space_chips << std::endl
               << "\t initial_divergence_bw_hz:   " << initial_divergence_bw_hz << std::endl
               << "\t final_divergence_bw_hz:   " << final_divergence_bw_hz << std::endl
               << "\t aid_code_with_carrier:   " << aid_code_with_carrier << std::endl;

    // Create the gnss_message input port
    message_port_register_in( GNSS_MESSAGE_PORT_ID );
    set_msg_handler( GNSS_MESSAGE_PORT_ID,
            boost::bind( &gps_l1_mcode_codeless_tracking_cc::handle_gnss_message, this, _1 ) );

    d_carrier_locked = false;
    d_frequency_locked = false;
    d_code_locked = false;
    d_code_locked_mcode = false;

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

    d_dll_loop_order_mcode = 1;
    d_dll_bw_hz_mcode = dll_bw_hz_mcode;

    d_pll_loop_order_mcode = 1;
    d_pll_bw_hz_mcode = pll_bw_hz_mcode;


    d_code_loop_filter = Tracking_loop_filter(GPS_L1_CA_CODE_PERIOD, dll_initial_bw_hz, dll_loop_order, false);
    d_carrier_loop_filter = Tracking_loop_filter(GPS_L1_CA_CODE_PERIOD, pll_initial_bw_hz, pll_loop_order, false);
    d_frequency_loop_filter = Tracking_loop_filter(GPS_L1_CA_CODE_PERIOD, pll_initial_bw_hz, pll_loop_order-1, true);
    d_aid_code_with_carrier = aid_code_with_carrier;

    // Should we close the mcode loops?
    d_close_mcode_loops = close_mcode_loops;

    d_code_loop_filter_mcode = Tracking_loop_filter(GPS_L1_CA_CODE_PERIOD * mcode_accumulation_length,
            d_dll_bw_hz_mcode, d_dll_loop_order_mcode, false);
    d_carrier_loop_filter_mcode = Tracking_loop_filter(GPS_L1_CA_CODE_PERIOD * mcode_accumulation_length,
            d_pll_bw_hz_mcode, d_pll_loop_order_mcode, false);
    // Initialize tracking  ==========================================


    // Correlator spacing
    d_initial_early_late_code_space_chips = initial_early_late_code_space_chips;
    d_final_early_late_code_space_chips = final_early_late_code_space_chips;
    d_early_late_code_spc_chips = d_initial_early_late_code_space_chips; // Define early-late offset (in chips)

    d_early_late_code_spc_cycles_mcode = 0.125;

    d_initial_very_early_late_code_space_chips = initial_very_early_late_code_space_chips;
    d_final_very_early_late_code_space_chips = final_very_early_late_code_space_chips;
    if( !use_bump_jumping )
    {
        d_very_early_late_code_spc_chips_mcode = d_initial_very_early_late_code_space_chips; // Define early-late offset (in chips)
    }
    else
    {
        d_very_early_late_code_spc_chips_mcode = GPS_M_CODE_CHIP_RATE_HZ /
            (2.0*GPS_M_CODE_SUB_CARRIER_RATE_HZ ); // 0.5 subcarrier cycles
    }

    // Initialization of local code replica
    // Get space for a vector with the code replica sampled 1x/chip
    d_ca_code = static_cast<gr_complex*>(volk_malloc((GPS_L1_CA_CODE_LENGTH_CHIPS + 2) * sizeof(gr_complex), volk_get_alignment()));

    d_early_code= static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_prompt_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_late_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));

    d_early_subcarrier_mcode= static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_prompt_subcarrier_mcode = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_late_subcarrier_mcode = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_very_early_subcarrier_mcode = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_very_late_subcarrier_mcode = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));

    d_early_code_phases_mcode= static_cast<int*>(volk_malloc(2 * d_vector_length * sizeof(int), volk_get_alignment()));
    d_prompt_code_phases_mcode = static_cast<int*>(volk_malloc(2 * d_vector_length * sizeof(int), volk_get_alignment()));
    d_late_code_phases_mcode = static_cast<int*>(volk_malloc(2 * d_vector_length * sizeof(int), volk_get_alignment()));
    d_very_early_code_phases_mcode = static_cast<int*>(volk_malloc(2 * d_vector_length * sizeof(int), volk_get_alignment()));
    d_very_late_code_phases_mcode = static_cast<int*>(volk_malloc(2 * d_vector_length * sizeof(int), volk_get_alignment()));

    // correlator outputs (scalar)
    d_Early = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Late = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));

    // correlator outputs (scalar)
    d_Very_Early_mcode = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Early_mcode = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Prompt_mcode = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Late_mcode = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));
    d_Very_Late_mcode = static_cast<gr_complex*>(volk_malloc(sizeof(gr_complex), volk_get_alignment()));


    // Set up the mcode code phase store. This should consist of a set of indices:
    // 0, 1, 2, 3, ... N-1
    // where N is the maxumum possible number of chips in the integration interval, accounting for the
    // early late code spacing.
    int num_mcode_chips_per_ca_code = static_cast< int >( GPS_L1_CA_CODE_PERIOD * GPS_M_CODE_CHIP_RATE_HZ );
    d_mcode_code_phase_store.resize( num_mcode_chips_per_ca_code + 2 ); // to account for E-L spacing
    // Handy C++11 utility to fill in an ascending array
    std::iota( d_mcode_code_phase_store.begin(), d_mcode_code_phase_store.end(), 0 );


    //--- Initializations ------------------------------
    // Initial code frequency basis of NCO
    d_code_freq_chips = static_cast<double>(GPS_L1_CA_CODE_RATE_HZ);

    d_code_freq_chips_mcode = static_cast<double>(GPS_M_CODE_CHIP_RATE_HZ);

    d_chips_to_cycles_mcode = GPS_M_CODE_SUB_CARRIER_RATE_HZ / GPS_M_CODE_CHIP_RATE_HZ;

    d_subcarrier_freq_cycles_mcode = d_code_freq_chips_mcode * d_chips_to_cycles_mcode;

    // Residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    d_code_phase_chips = 0.0;

    d_rem_code_phase_samples_mcode = 0.0;
    d_integer_code_phase_chips_mcode = 0;
    d_fractional_code_phase_chips_mcode = 0.0;

    d_integer_subcarrier_phase_cycles_mcode = 0;
    d_fractional_subcarrier_phase_cycles_mcode = 0.0;
    // Residual carrier phase
    d_rem_carr_phase_rad = 0.0;
    d_rem_carr_phase_rad_mcode = 0.0;

    // sample synchronization
    d_sample_counter = 0;
    //d_sample_counter_seconds = 0;
    d_acq_sample_stamp = 0;

    d_enable_tracking = false;
    d_pull_in = false;
    d_last_seg = 0;
    d_mcode_tracking_enabled = false;

    d_current_prn_length_samples = static_cast<int>(d_vector_length);

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[CN0_ESTIMATION_SAMPLES];
    d_carrier_lock_test = 1;
    d_CN0_SNV_dB_Hz = 0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_success_counter = 0;
    d_carrier_lock_threshold = CARRIER_LOCK_THRESHOLD;

    d_fll_epochs = 0;
    d_max_fll_epochs = 250;

    systemName["G"] = std::string("GPS");
    *d_Early = gr_complex(0,0);
    *d_Prompt = gr_complex(0,0);
    *d_Late = gr_complex(0,0);

    *d_Very_Early_mcode = gr_complex(0,0);
    *d_Early_mcode = gr_complex(0,0);
    *d_Prompt_mcode = gr_complex(0,0);
    *d_Late_mcode = gr_complex(0,0);
    *d_Very_Late_mcode = gr_complex(0,0);

    d_channel_internal_queue = 0;
    d_acquisition_gnss_synchro = 0;
    d_channel = 0;
    d_acq_code_phase_samples = 0.0;
    d_acq_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz_mcode = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_acc_code_phase_secs = 0.0;

    d_tow_received = false;
    d_rx_time_set = false;
    d_preamble_start_detected = false;

    // Bump jumping:
    d_use_bj = use_bump_jumping;
    d_bj_ve_counter_mcode = 0;
    d_bj_vl_counter_mcode = 0;

    d_bj_threshold = bump_jumping_threshold;

    // Subcarrier aiding:
    d_use_sa = !use_bump_jumping;
    d_initial_divergence_loop_filter_bandwidth = initial_divergence_bw_hz;
    d_final_divergence_loop_filter_bandwidth = final_divergence_bw_hz;

    d_divergence_loop_filter_mcode = Tracking_loop_filter( GPS_L1_CA_CODE_PERIOD * d_mcode_accumulation_length,
        d_initial_divergence_loop_filter_bandwidth, 1, false );

    d_subcarrier_locked_mcode = false;

    d_mean_subcarrier_error_mcode = 0.0;

    d_code_locked = false;
    d_code_locked_mcode = false;

    d_mean_code_error = 0.0;
    d_mean_code_error_mcode = 0.0;

    d_carr_error_hz_mcode = 0;
    d_carr_error_filt_hz_mcode = 0;
    d_subcarrier_error_cycles_mcode = 0;
    d_subcarrier_error_filt_cycles_mcode = 0;
    d_code_error_chips_veml_mcode = 0;
    d_code_error_filt_chips_veml_mcode = 0;


    // How many CA code periods do we accumulate over for the mcode:
    d_mcode_accumulation_length = mcode_accumulation_length;
    d_mcode_accumulation_index = 0;

    // Now setup the code/subcarrier resamplers:
    // For now use the Fxpt64 implementation, but might look into LUT approach
    // for efficiency later.
    d_ca_code_resampler = boost::shared_ptr< CodeResamplerInterface< gr_complex > >(
            new CodeResamplerFxpt64<gr_complex> );

    d_mcode_code_phase_resampler = boost::shared_ptr< CodeResamplerInterface< int > >(
            new CodeResamplerFxpt64<int> );
    d_mcode_subcarrier_resampler = boost::shared_ptr< SubcarrierResamplerInterface< gr_complex > >(
            new SubcarrierResamplerFxpt64< gr_complex > );

}

void gps_l1_mcode_codeless_tracking_cc::start_tracking()
{
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp =  d_acquisition_gnss_synchro->Acq_samplestamp_samples;

    // DLL/PLL filter initialization
    d_code_loop_filter.set_noise_bandwidth( d_initial_dll_bw_hz );
    d_carrier_loop_filter.set_noise_bandwidth( d_initial_pll_bw_hz );
    d_frequency_loop_filter.set_noise_bandwidth( d_initial_pll_bw_hz );

    d_frequency_loop_filter.initialize(d_acq_carrier_doppler_hz); // initialize the carrier filter
    d_carrier_loop_filter.initialize( 0 );
    float code_doppler_chips = d_acq_carrier_doppler_hz *( GPS_L1_CA_CODE_RATE_HZ) / GPS_L1_FREQ_HZ;

    if( d_aid_code_with_carrier ){
        d_code_loop_filter.initialize(0.0);    // initialize the code filter
    }
    else{
        d_code_loop_filter.initialize(code_doppler_chips);    // initialize the code filter
    }

    // generate local reference ALWAYS starting at chip 1 (1 samples per chip)
    gps_l1_ca_code_gen_complex(&d_ca_code[1],
            d_acquisition_gnss_synchro->PRN,
            0);
    d_ca_code[0] = d_ca_code[static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS)];
    d_ca_code[static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS) + 1] = d_ca_code[1];

    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_success_counter = 0;
    d_rem_code_phase_samples = 0.0;
    d_rem_carr_phase_rad = 0;
    d_acc_carrier_phase_rad = 0;

    d_fll_epochs = 0;

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
    d_frequency_locked = false;
    d_cn0_estimation_counter = 0;

    d_code_locked = false;
    d_mean_code_error = 0.0;

    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz
              << " PULL-IN Code Phase [samples]=" << d_acq_code_phase_samples;
}


void gps_l1_mcode_codeless_tracking_cc::update_local_code()
{

    std::vector< double > init_code_phase( 3 );

    init_code_phase[0] = d_code_phase_chips+
        d_early_late_code_spc_chips;
    init_code_phase[1] = d_code_phase_chips;
    init_code_phase[2] = d_code_phase_chips-
        d_early_late_code_spc_chips;

    // Store the output in a vector:
    std::vector< gr_complex * > resampled_codes( 3 );
    resampled_codes[0] = d_early_code;
    resampled_codes[1] = d_prompt_code;
    resampled_codes[2] = d_late_code;

    double code_phase_step = d_code_freq_chips / static_cast< double >( d_fs_in );

    unsigned int code_len = 1023;

    // Now resample the code:
    d_ca_code_resampler->resample_code( d_ca_code, code_len,
            init_code_phase, code_phase_step,
            d_current_prn_length_samples,
            resampled_codes );

}


void gps_l1_mcode_codeless_tracking_cc::update_local_code_mcode()
{
    // Here we use the mcode code phase and subcarrier resamplers to update the local replicas:

    // 1) Resample the subcarrier:
    std::vector< double > init_subcarrier_phase_cycles( 5 );
    // very early:
    init_subcarrier_phase_cycles[0] = d_fractional_subcarrier_phase_cycles_mcode
        + d_very_early_late_code_spc_chips_mcode * d_chips_to_cycles_mcode;
    // early:
    init_subcarrier_phase_cycles[1] = d_fractional_subcarrier_phase_cycles_mcode
        + d_early_late_code_spc_cycles_mcode;
    // prompt:
    init_subcarrier_phase_cycles[2] = d_fractional_subcarrier_phase_cycles_mcode;
    // late:
    init_subcarrier_phase_cycles[3] = d_fractional_subcarrier_phase_cycles_mcode
        - d_early_late_code_spc_cycles_mcode;
    // very late:
    init_subcarrier_phase_cycles[4] = d_fractional_subcarrier_phase_cycles_mcode
        - d_very_early_late_code_spc_chips_mcode * d_chips_to_cycles_mcode;

    double subcarrier_phase_step = d_subcarrier_freq_cycles_mcode
        / static_cast< double >( d_fs_in );

    std::vector< gr_complex * > resampled_subcarriers( 5 );
    resampled_subcarriers[0] = d_very_early_subcarrier_mcode;
    resampled_subcarriers[1] = d_early_subcarrier_mcode;
    resampled_subcarriers[2] = d_prompt_subcarrier_mcode;
    resampled_subcarriers[3] = d_late_subcarrier_mcode;
    resampled_subcarriers[4] = d_very_late_subcarrier_mcode;

    d_mcode_subcarrier_resampler->resample_subcarrier( init_subcarrier_phase_cycles,
            subcarrier_phase_step,
            d_current_prn_length_samples,
            resampled_subcarriers,
            false // <-- Cosine phasing
            );

    // 2) Resample the code phases:
    std::vector< double > init_code_phase( 5 );
    // Add 1.0 to each code phase to ensure we have no negative values
    init_code_phase[0] = d_fractional_code_phase_chips_mcode +
        d_very_early_late_code_spc_chips_mcode + 1.0;
    init_code_phase[1] = d_fractional_code_phase_chips_mcode +
        d_early_late_code_spc_cycles_mcode/d_chips_to_cycles_mcode + 1.0;
    init_code_phase[2] = d_fractional_code_phase_chips_mcode + 1.0;
    init_code_phase[3] = d_fractional_code_phase_chips_mcode -
        d_early_late_code_spc_cycles_mcode/d_chips_to_cycles_mcode + 1.0;
    init_code_phase[4] = d_fractional_code_phase_chips_mcode -
        d_very_early_late_code_spc_chips_mcode + 1.0;

    // Store the output in a vector:
    std::vector< int * > resampled_codes( 5 );
    resampled_codes[0] = d_very_early_code_phases_mcode;
    resampled_codes[1] = d_early_code_phases_mcode;
    resampled_codes[2] = d_prompt_code_phases_mcode;
    resampled_codes[3] = d_late_code_phases_mcode;
    resampled_codes[4] = d_very_late_code_phases_mcode;

    double code_phase_step = d_code_freq_chips_mcode / static_cast< double >( d_fs_in );

    d_mcode_code_phase_resampler->resample_code( &d_mcode_code_phase_store[0],
            d_mcode_code_phase_store.size(),
            init_code_phase, code_phase_step,
            d_current_prn_length_samples,
            resampled_codes );

}

gps_l1_mcode_codeless_tracking_cc::~gps_l1_mcode_codeless_tracking_cc()
{
    d_dump_file.close();

    volk_free(d_early_code);
    volk_free(d_prompt_code);
    volk_free(d_late_code);

    volk_free(d_Early);
    volk_free(d_Prompt);
    volk_free(d_Late);
    volk_free(d_ca_code);

    volk_free(d_very_early_code_phases_mcode);
    volk_free(d_early_code_phases_mcode);
    volk_free(d_prompt_code_phases_mcode);
    volk_free(d_late_code_phases_mcode);
    volk_free(d_very_late_code_phases_mcode);

    volk_free(d_very_early_subcarrier_mcode);
    volk_free(d_early_subcarrier_mcode);
    volk_free(d_prompt_subcarrier_mcode);
    volk_free(d_late_subcarrier_mcode);
    volk_free(d_very_late_subcarrier_mcode);

    volk_free(d_Very_Early_mcode);
    volk_free(d_Early_mcode);
    volk_free(d_Prompt_mcode);
    volk_free(d_Late_mcode);
    volk_free(d_Very_Late_mcode);
    delete[] d_Prompt_buffer;
}



int gps_l1_mcode_codeless_tracking_cc::general_work (int noutput_items,gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    double carr_error_hz = 0.0;
    double carr_error_filt_hz = 0.0;
    double code_error_chips = 0.0;
    double code_error_filt_chips = 0.0;

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
                    d_fractional_subcarrier_phase_cycles_mcode = 0;

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


            // perform carrier wipe-off and compute Early, Prompt, and Late correlation
            d_correlator.Carrier_rotate_and_EPL_volk(d_current_prn_length_samples,
                    in,
                    &phase_as_complex,
                    phase_inc_as_complex,
                    d_early_code,
                    d_prompt_code,
                    d_late_code,
                    d_Early,
                    d_Prompt,
                    d_Late );


            // Now update the code and carrier phase estimates:
            double T = static_cast<double>( d_current_prn_length_samples ) / static_cast<double>( d_fs_in );

            d_code_phase_chips += T*d_code_freq_chips;
            d_code_phase_chips = std::fmod( d_code_phase_chips, GPS_L1_CA_CODE_LENGTH_CHIPS );

            d_carrier_phase_rad += T*2.0*M_PI*d_carrier_doppler_hz;

            double rem_code_phase_chips = GPS_L1_CA_CODE_LENGTH_CHIPS - d_code_phase_chips;
            if( rem_code_phase_chips > GPS_L1_CA_CODE_LENGTH_CHIPS / 2.0 )
            {
                rem_code_phase_chips = ( rem_code_phase_chips - GPS_L1_CA_CODE_LENGTH_CHIPS );
            }

            d_rem_code_phase_samples = rem_code_phase_chips * d_fs_in/GPS_L1_CA_CODE_RATE_HZ;

            //remnant carrier phase to prevent overflow in the code NCO
            d_rem_carr_phase_rad = d_rem_carr_phase_rad + 2.0 * M_PI * (d_if_freq + d_carrier_doppler_hz )* T;
            d_rem_carr_phase_rad = fmod(d_rem_carr_phase_rad, 2.0 * M_PI);

            //carrier phase accumulator for (K) Doppler estimation
            d_acc_carrier_phase_rad = d_acc_carrier_phase_rad + 2.0 * M_PI * d_carrier_doppler_hz * T;

            if( d_rem_carr_phase_rad >= M_PI )
            {
                d_rem_carr_phase_rad -= 2.0 * M_PI;
            }

            if( d_rem_carr_phase_rad < -M_PI )
            {
                d_rem_carr_phase_rad += 2.0 * M_PI;
            }

            // Compare carrier phase with the rotated phase from VOLK:
            //double rotator_carr_phase_rad = std::atan2( -phase_as_complex.imag(), phase_as_complex.real() );
            //LOG(INFO) << "Propagated Phase: " << d_rem_carr_phase_rad
                      //<< " . Rotated phase : " << rotator_carr_phase_rad
                      //<< " . Diff: " << d_rem_carr_phase_rad - rotator_carr_phase_rad;

            // mcode tracking
            if( d_mcode_tracking_enabled ){
                // Generate local code and carrier replicas (using \hat{f}_d(k-1))
                update_local_code_mcode();

                phase_as_complex = gr_complex( std::cos( d_rem_carr_phase_rad_mcode ),
                        -std::sin( d_rem_carr_phase_rad_mcode ) );

                carrier_doppler_inc_rad = 2.0*M_PI*(d_if_freq + d_carrier_doppler_hz_mcode )/d_fs_in;

                phase_inc_as_complex = gr_complex( std::cos( carrier_doppler_inc_rad ),
                        -std::sin( carrier_doppler_inc_rad ) );


                // perform carrier wipe-off and compute Very Early, Early, Prompt, Late and Very Late correlation
                d_correlator.Carrier_rotate_and_VEPL_codeless(d_current_prn_length_samples,
                        in,
                        &phase_as_complex,
                        phase_inc_as_complex,
                        d_very_early_code_phases_mcode,
                        d_early_code_phases_mcode,
                        d_prompt_code_phases_mcode,
                        d_late_code_phases_mcode,
                        d_very_late_code_phases_mcode,
                        d_very_early_subcarrier_mcode,
                        d_early_subcarrier_mcode,
                        d_prompt_subcarrier_mcode,
                        d_late_subcarrier_mcode,
                        d_very_late_subcarrier_mcode,
                        d_Very_Early_mcode,
                        d_Early_mcode,
                        d_Prompt_mcode,
                        d_Late_mcode,
                        d_Very_Late_mcode,
                        d_mcode_code_phase_store.size() );

                // Accumulate:
                if( d_mcode_accumulation_index == 0 )
                {

                    d_VE_acumm_mcode = 0.0;
                    d_E_acumm_mcode  = 0.0;
                    d_P_acumm_mcode  = 0.0;
                    d_L_acumm_mcode  = 0.0;
                    d_VL_acumm_mcode = 0.0;
                }

                d_VE_acumm_mcode += *d_Very_Early_mcode;
                d_E_acumm_mcode += *d_Early_mcode;
                d_P_acumm_mcode += *d_Prompt_mcode;
                d_L_acumm_mcode += *d_Late_mcode;
                d_VL_acumm_mcode += *d_Very_Late_mcode;

                // Increment the accumulation count:
                d_mcode_accumulation_index++;

                // Now update the code and carrier phase estimates:
                double delta_code_phase_mcode = T*d_code_freq_chips_mcode;

                d_fractional_code_phase_chips_mcode += delta_code_phase_mcode;

                d_integer_code_phase_chips_mcode += static_cast< int64_t >(
                        std::floor( d_fractional_code_phase_chips_mcode ) );


                d_fractional_code_phase_chips_mcode = std::fmod(
                        d_fractional_code_phase_chips_mcode, 1.0 );
                //DLOG(INFO) << "Propagated mcode code phase: " << std::fixed << std::setprecision( 12 ) << T*d_code_freq_chips_mcode;

                if( d_use_sa )
                {
                    d_fractional_subcarrier_phase_cycles_mcode += T*d_subcarrier_freq_cycles_mcode;

                    d_integer_subcarrier_phase_cycles_mcode += static_cast< int64_t >(
                            std::floor( d_fractional_subcarrier_phase_cycles_mcode ) );
                }
                else
                {
                    d_fractional_subcarrier_phase_cycles_mcode = std::fmod( d_fractional_code_phase_chips_mcode,
                            1.0) *d_chips_to_cycles_mcode;
                }

                d_fractional_subcarrier_phase_cycles_mcode = std::fmod( d_fractional_subcarrier_phase_cycles_mcode, 1.0 );

                d_carrier_phase_rad_mcode += T*2.0*M_PI*d_carrier_doppler_hz_mcode;

                int64_t ca_code_length_mcode_chips = static_cast< int64_t>(
                        GPS_L1_CA_CODE_LENGTH_CHIPS /
                        GPS_L1_CA_CODE_RATE_HZ * GPS_M_CODE_CHIP_RATE_HZ
                        );


                int64_t chips_into_ca_code_period =
                    d_integer_code_phase_chips_mcode % ca_code_length_mcode_chips;

                double rem_code_phase_chips_mcode = static_cast< double >(
                        ca_code_length_mcode_chips - chips_into_ca_code_period
                        ) - d_fractional_code_phase_chips_mcode;

                if( rem_code_phase_chips_mcode > static_cast< double >(
                            ca_code_length_mcode_chips / 2 ) )
                {
                    rem_code_phase_chips_mcode = ( rem_code_phase_chips_mcode -
                            static_cast<double>(ca_code_length_mcode_chips) );
                }

                d_rem_code_phase_samples_mcode = rem_code_phase_chips_mcode * d_fs_in/GPS_M_CODE_CHIP_RATE_HZ;

                d_rem_carr_phase_rad_mcode = d_rem_carr_phase_rad_mcode + 2.0*M_PI *( d_if_freq + d_carrier_doppler_hz_mcode ) * T;
                d_rem_carr_phase_rad_mcode = fmod(d_rem_carr_phase_rad_mcode, 2.0*M_PI);

            }

            // check for samples consistency (this should be done before in the receiver / here only if the source is a file)
            if (std::isnan((*d_Prompt).real()) == true or
                    std::isnan((*d_Prompt).imag()) == true ) // or std::isinf(in[i].real())==true or std::isinf(in[i].imag())==true)
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

            // ################## FLL/PLL ##########################################################
            if( !d_frequency_locked ){
                d_fll_epochs++;
                // Only do the FLL every second correlator dump
                if( d_fll_epochs % 2 ==  0){
                    // FLL discriminator
                    int buf_ind = d_cn0_estimation_counter - 1;
                    if( buf_ind < 0 ){
                        buf_ind += CN0_ESTIMATION_SAMPLES;
                    }
                    // FLL discriminator
                    double T = static_cast< double >( d_current_prn_length_samples )/d_fs_in;
                    carr_error_hz = fll_two_quadrant_atan(d_Prompt_buffer[buf_ind], *d_Prompt, 0, T) / static_cast<float>(GPS_TWO_PI);
                    // Carrier discriminator filter
                    carr_error_filt_hz = d_frequency_loop_filter.apply(carr_error_hz);
                    // New carrier Doppler frequency estimation
                    d_carrier_doppler_hz = carr_error_filt_hz;
                }
            }
            else {
                // PLL discriminator
                carr_error_hz = pll_cloop_two_quadrant_atan(*d_Prompt) / static_cast<float>(GPS_TWO_PI);
                // Carrier discriminator filter
                carr_error_filt_hz = d_carrier_loop_filter.apply(carr_error_hz);
                // New carrier Doppler frequency estimation
                d_carrier_doppler_hz = carr_error_filt_hz;
            }

            float code_doppler_chips = ((d_carrier_doppler_hz * GPS_L1_CA_CODE_RATE_HZ) / GPS_L1_FREQ_HZ);


            // ################## DLL ##########################################################
            // DLL discriminator
            // ################## VE - VL Processing ############################################

            code_error_chips = dll_nc_e_minus_l_normalized(
                    *d_Early, *d_Late );

            double corr_slope = 1.0;
            code_error_chips *= 2.0*( 1 - corr_slope*d_early_late_code_spc_chips) / corr_slope;

            code_error_filt_chips = d_code_loop_filter.apply( code_error_chips );

            d_code_freq_chips = GPS_L1_CA_CODE_RATE_HZ + code_error_filt_chips;

            if( d_aid_code_with_carrier ){
                d_code_freq_chips += code_doppler_chips;
            }

            // ################## mcode ##########################################################
            if( d_mcode_tracking_enabled && d_mcode_accumulation_index == d_mcode_accumulation_length )
            {
                // ################## PLL ##########################################################
                // PLL discriminator
                d_carr_error_hz_mcode = 0.5 * pll_cloop_two_quadrant_atan(d_P_acumm_mcode) / static_cast<float>(2.0*M_PI);

                // Carrier discriminator filter
                d_carr_error_filt_hz_mcode = d_carrier_loop_filter_mcode.apply(d_carr_error_hz_mcode);


                // ################## DLL ##########################################################
                // DLL discriminator
                d_subcarrier_error_cycles_mcode = dll_nc_e_minus_l_normalized(
                         d_E_acumm_mcode,
                         d_L_acumm_mcode ); //[chips/Ti]
                //Normalise the code phase error:
                // Here we assume that the front-end filter is only passing the
                // first lobe of the mcode and we enforce a correlator spacing of
                // +/- 1/8 of a subchip.
                corr_slope = 4*M_PI;
                d_subcarrier_error_cycles_mcode *= 2.0/corr_slope;

                // Code discriminator filter
                d_subcarrier_error_filt_cycles_mcode = d_code_loop_filter_mcode.apply(d_subcarrier_error_cycles_mcode); //[chips/second]

                // ################## VE - VL Processing ############################################

                d_code_error_chips_veml_mcode = dll_nc_e_minus_l_normalized(
                        d_VE_acumm_mcode, d_VL_acumm_mcode );

                corr_slope = 2.0;
                d_code_error_chips_veml_mcode *= 2.0 / corr_slope;

                if( d_close_mcode_loops )
                {

                    if( d_use_sa && d_subcarrier_locked_mcode )
                    {
                        d_code_error_filt_chips_veml_mcode = d_divergence_loop_filter_mcode.apply(
                                d_code_error_chips_veml_mcode );
                    }

                    if( d_use_bj && d_carrier_locked ){

                        float P = std::abs<float>( *d_Prompt_mcode );
                        float VE = std::abs<float>( *d_Very_Early_mcode );
                        float VL = std::abs<float>( *d_Very_Late_mcode );

                        double jump_dir = 0.0;
                        bool do_jump = false;

                        if( VE > P && VE > VL )
                        {

                            d_bj_ve_counter_mcode++;
                            if( d_bj_vl_counter_mcode > 0 )
                            {
                                d_bj_vl_counter_mcode--;
                            }

                            if( d_bj_ve_counter_mcode >= d_bj_threshold )
                            {
                                // Time to jump!
                                jump_dir = 1.0;
                                do_jump = true;
                            }
                        }

                        if( VL > P && VL > VE )
                        {
                            d_bj_vl_counter_mcode++;
                            if( d_bj_ve_counter_mcode > 0 )
                            {
                                d_bj_ve_counter_mcode--;
                            }

                            if( d_bj_vl_counter_mcode >= d_bj_threshold )
                            {
                                jump_dir = -1.0;
                                do_jump = true;
                            }
                        }

                        if( do_jump )
                        {
                            double half_cycle_in_chips = 0.5/d_chips_to_cycles_mcode;

                            d_fractional_code_phase_chips_mcode += half_cycle_in_chips*jump_dir;

                            if( d_fractional_code_phase_chips_mcode >= 1.0 )
                            {
                                d_fractional_code_phase_chips_mcode -= 1.0;
                                d_integer_code_phase_chips_mcode += 1;
                            }

                            if( d_fractional_code_phase_chips_mcode < 0.0 )
                            {
                                d_fractional_code_phase_chips_mcode += 1.0;
                                d_integer_code_phase_chips_mcode -= 1;
                            }

                            std::stringstream ss("");

                            ss << "BJ: false peak detected on mcode! "
                                << " Jumping " << ( jump_dir < 0 ? "forward" : "backward" )
                                << " . Channel: " << d_channel
                                << " . [PRN: " << d_acquisition_gnss_synchro->PRN
                                << " @ " << static_cast< double >( d_sample_counter )/
                                static_cast< double >( d_fs_in )
                                << "]" << std::endl;

                            LOG(INFO) << ss.str();
                            std::cout << ss.str();

                            d_bj_ve_counter_mcode = 0;
                            d_bj_vl_counter_mcode = 0;
                        }


                    }
                }

                d_mcode_accumulation_index = 0;
            }

            // By default we simply update the mcode frequencies from the CA code:
            d_carrier_doppler_hz_mcode = d_carrier_doppler_hz;
            d_code_freq_chips_mcode = GPS_M_CODE_CHIP_RATE_HZ +
                    d_carrier_doppler_hz * GPS_M_CODE_CHIP_RATE_HZ/GPS_L1_FREQ_HZ;
            d_subcarrier_freq_cycles_mcode = GPS_M_CODE_SUB_CARRIER_RATE_HZ +
                   d_carrier_doppler_hz * GPS_M_CODE_SUB_CARRIER_RATE_HZ/GPS_L1_FREQ_HZ;

            if( d_close_mcode_loops )
            {
                d_carrier_doppler_hz_mcode += d_carr_error_filt_hz_mcode;
                d_subcarrier_freq_cycles_mcode += d_subcarrier_error_filt_cycles_mcode;
                d_code_freq_chips_mcode = d_subcarrier_freq_cycles_mcode /d_chips_to_cycles_mcode;
                if( d_use_sa && d_subcarrier_locked_mcode )
                {
                    d_code_freq_chips_mcode += d_code_error_filt_chips_veml_mcode;
                }
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
            T_chip_seconds = 1 / static_cast<double>(d_code_freq_chips);
            T_prn_seconds = T_chip_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
            T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
            K_blk_samples = T_prn_samples + d_rem_code_phase_samples; // + code_error_filt_secs * static_cast<double>(d_fs_in);


            next_prn_length_samples = round(K_blk_samples); //round to a discrete samples
            //d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample

            // ####### CN0 ESTIMATION AND LOCK DETECTORS ######
            // fill buffer with prompt correlator output values
            d_Prompt_buffer[d_cn0_estimation_counter] = *d_Prompt;
            d_cn0_estimation_counter++;

            d_mean_code_error += std::fabs( code_error_chips );
            if (d_cn0_estimation_counter >= CN0_ESTIMATION_SAMPLES )
                {
                    d_cn0_estimation_counter = 0;

                    d_mean_code_error /= static_cast<double>( CN0_ESTIMATION_SAMPLES );
                    // Code lock indicator
                    d_CN0_SNV_dB_Hz = cn0_svn_estimator(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES, d_fs_in, d_current_prn_length_samples);

                    // Carrier lock indicator
                    d_carrier_lock_test = carrier_lock_detector(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES);

                    if( not d_frequency_locked ){
                        if( d_fll_epochs >= 2*d_max_fll_epochs ){
                            d_frequency_locked = true;
                            d_carrier_loop_filter.initialize( d_carrier_doppler_hz );
                        }
                    }
                    else if( not d_carrier_locked ){

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
                                d_mcode_tracking_enabled = false;
                                d_tow_received = false;
                            }

                        if( d_carrier_lock_success_counter > MINIMUM_LOCK_SUCCESS_COUNTER )
                        {
                            LOG(INFO) << "Phase lock achieved in channel " << d_channel;
                            d_carrier_locked = true;
                            d_carrier_loop_filter.set_noise_bandwidth( d_final_pll_bw_hz );
                            d_carrier_loop_filter.initialize( d_carrier_doppler_hz );

                            d_carrier_lock_fail_counter = 0;

                            // Try to enable mcode tracking:
                            //start_tracking_mcode();
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
                                d_carrier_loop_filter.set_noise_bandwidth( d_initial_pll_bw_hz );
                                d_carrier_loop_filter.initialize( d_carrier_doppler_hz );
                                d_code_locked=false;
                                d_code_loop_filter.set_noise_bandwidth( d_initial_dll_bw_hz );
                                d_code_loop_filter.initialize( code_error_filt_chips );
                                d_early_late_code_spc_chips = d_initial_early_late_code_space_chips;
                                //d_very_early_late_code_spc_chips = d_initial_very_early_late_code_space_chips;
                                d_mean_code_error = 0.0;
                                d_cn0_estimation_counter = 0;
                            }
                    }

                    if( d_code_locked )
                    {
                        if( d_mean_code_error > 0.1 )
                        {
                            d_code_locked = false;
                            d_code_loop_filter.set_noise_bandwidth( d_initial_dll_bw_hz );
                            d_early_late_code_spc_chips = d_initial_early_late_code_space_chips;
                            d_code_loop_filter.initialize( code_error_filt_chips );
                            d_mean_code_error = 0.0;
                            d_cn0_estimation_counter = 0;

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
                            d_code_loop_filter.set_noise_bandwidth( d_final_dll_bw_hz );
                            d_early_late_code_spc_chips = d_final_early_late_code_space_chips;
                            d_code_loop_filter.initialize( code_error_filt_chips );

                            std::stringstream ss("");

                            ss << "Code lock achieved in channel "
                                << d_channel << "!"
                                << "[PRN: " << d_acquisition_gnss_synchro->PRN
                                << ". @ " << static_cast< double >( d_sample_counter )/
                                static_cast<double>( d_fs_in )
                                << "]";

                            LOG(INFO) << ss.str();

                            std::cout << ss.str() << std::endl;;
                            // Try to enable mcode tracking:
                            start_tracking_mcode();
                        }
                    }


                    d_mean_code_error = 0.0;
                }

            if( d_mcode_tracking_enabled )
            {

                if( d_mcode_accumulation_index == 0 )
                {
                    d_mean_subcarrier_error_mcode += std::fabs(
                            d_subcarrier_error_cycles_mcode );

                    d_mean_code_error_mcode += std::fabs( d_code_error_chips_veml_mcode );

                    d_cn0_estimation_counter_mcode++;

                    if (d_cn0_estimation_counter_mcode >= CN0_ESTIMATION_SAMPLES )
                        {
                            d_cn0_estimation_counter_mcode = 0;

                            d_mean_subcarrier_error_mcode /= static_cast<double>( CN0_ESTIMATION_SAMPLES );

                            d_mean_code_error_mcode /= static_cast<double>( CN0_ESTIMATION_SAMPLES );


                            if( d_subcarrier_locked_mcode )
                            {
                                if( d_mean_subcarrier_error_mcode > 0.4 )
                                {
                                    d_subcarrier_locked_mcode = false;

                                    if( d_use_sa )
                                    {
                                        d_divergence_loop_filter_mcode.set_noise_bandwidth(
                                                d_initial_divergence_loop_filter_bandwidth );
                                    }

                                    std::stringstream ss("");

                                    ss << "Loss of mcode subcarrier lock in channel "
                                        << d_channel << "!"
                                        << "[PRN: " << d_acquisition_gnss_synchro->PRN
                                        << ". @ " << static_cast< double >( d_sample_counter )/
                                        static_cast<double>( d_fs_in )
                                        << "]";

                                    LOG(INFO) << ss.str();

                                    std::cout << ss.str() << std::endl;
                                }
                                else
                                {
                                    if( d_code_locked_mcode )
                                    {
                                        if( d_mean_code_error_mcode*d_chips_to_cycles_mcode > 0.5 )
                                        {
                                            d_code_locked_mcode = false;

                                            if( d_use_sa )
                                            {
                                                d_divergence_loop_filter_mcode.set_noise_bandwidth(
                                                        d_initial_divergence_loop_filter_bandwidth );
                                            }

                                            std::stringstream ss("");

                                            ss << "mcode Loss of code lock in channel "
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
                                        if( d_mean_code_error_mcode*d_chips_to_cycles_mcode < 0.1 )
                                        {
                                            d_code_locked_mcode = true;

                                            if( d_use_sa )
                                            {
                                                d_very_early_late_code_spc_chips_mcode = d_final_very_early_late_code_space_chips;
                                                d_divergence_loop_filter_mcode.set_noise_bandwidth(
                                                        d_final_divergence_loop_filter_bandwidth );
                                            }

                                            std::stringstream ss("");

                                            ss << "mcode Code lock achieved in channel "
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


                            } // if d_subcarrier_locked_mcode
                            else
                            {
                                if( d_mean_subcarrier_error_mcode < 0.01 )
                                {
                                    d_subcarrier_locked_mcode = true;

                                    std::stringstream ss("");

                                    ss << "mcode Subcarrier lock achieved in channel "
                                        << d_channel << "!"
                                        << "[PRN: " << d_acquisition_gnss_synchro->PRN
                                        << ". @ " << static_cast< double >( d_sample_counter )/
                                        static_cast<double>( d_fs_in )
                                        << "]";

                                    LOG(INFO) << ss.str();

                                    std::cout << ss.str() << std::endl;

                                    d_code_locked_mcode = false;
                                    if( d_use_sa )
                                    {
                                        d_divergence_loop_filter_mcode.set_noise_bandwidth(
                                                d_initial_divergence_loop_filter_bandwidth );
                                        d_divergence_loop_filter_mcode.initialize( 0.0 );
                                    }
                                }

                            }

                            d_mean_subcarrier_error_mcode = 0.0;
                            d_mean_code_error_mcode = 0.0;
                        } 

                    }// if d_mcode_accumulation_index == 0


            } // d_mcode_tracking_enabled

            // ########### Output the tracking results to Telemetry block ##########

            current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt).real());
            current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt).imag());

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
    	*d_Early = gr_complex(0,0);
    	*d_Prompt = gr_complex(0,0);
    	*d_Late = gr_complex(0,0);

    	*d_Very_Early_mcode = gr_complex(0,0);
    	*d_Early_mcode = gr_complex(0,0);
    	*d_Prompt_mcode = gr_complex(0,0);
    	*d_Late_mcode = gr_complex(0,0);
    	*d_Very_Late_mcode = gr_complex(0,0);

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
            prompt_I = (*d_Prompt).real();
            prompt_Q = (*d_Prompt).imag();
            tmp_E = std::abs<float>(*d_Early);
            tmp_P = std::abs<float>(*d_Prompt);
            tmp_L = std::abs<float>(*d_Late);

            try
            {
                    // Dump correlators output
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_E), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_P), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_L), sizeof(float));
                    // PROMPT I and Q (to analyze navigation symbols)
                    d_dump_file.write(reinterpret_cast<char*>(&prompt_I), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&prompt_Q), sizeof(float));
                    // PRN start sample stamp
                    d_dump_file.write(reinterpret_cast<char*>(&d_sample_counter), sizeof(unsigned long int));
                    // accumulated carrier phase
                    tmp_float = static_cast<float>(d_acc_carrier_phase_rad);
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));
                    // carrier and code frequency
                    d_dump_file.write(reinterpret_cast<char*>(&d_carrier_doppler_hz), sizeof(double));
                    d_dump_file.write(reinterpret_cast<char*>(&d_code_freq_chips), sizeof(double));
                    //PLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&carr_error_hz), sizeof(double));
                    d_dump_file.write(reinterpret_cast<char*>(&carr_error_filt_hz), sizeof(double));
                    //DLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&code_error_chips), sizeof(double));
                    d_dump_file.write(reinterpret_cast<char*>(&code_error_filt_chips), sizeof(double));
                    // CN0 and carrier lock test
                    d_dump_file.write(reinterpret_cast<char*>(&d_CN0_SNV_dB_Hz), sizeof(double));
                    d_dump_file.write(reinterpret_cast<char*>(&d_carrier_lock_test), sizeof(double));
                    // AUX vars (for debug purposes)
                    tmp_float = d_code_phase_chips;
                    d_dump_file.write(reinterpret_cast<char*>(&d_code_phase_chips), sizeof(double));
                    //tmp_double = static_cast<double>(d_sample_counter + d_current_prn_length_samples);

                    // ****************************************************************************
                    // mcode Variables:
                    prompt_I = (*d_Prompt_mcode).real();
                    prompt_Q = (*d_Prompt_mcode).imag();
                    tmp_VE = std::abs<float>( d_VE_acumm_mcode);
                    tmp_E = std::abs<float>(d_E_acumm_mcode);
                    tmp_P = std::abs<float>(d_P_acumm_mcode);
                    tmp_L = std::abs<float>(d_L_acumm_mcode);
                    tmp_VL = std::abs<float>(d_VL_acumm_mcode);
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
                    d_dump_file.write(reinterpret_cast<char*>(&d_carrier_doppler_hz_mcode), sizeof(double));
                    d_dump_file.write(reinterpret_cast<char*>(&d_code_freq_chips_mcode), sizeof(double));
                    //PLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&d_carr_error_hz_mcode), sizeof(double));
                    d_dump_file.write(reinterpret_cast<char*>(&d_carr_error_filt_hz_mcode), sizeof(double));
                    //DLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&d_subcarrier_error_cycles_mcode), sizeof(double));
                    d_dump_file.write(reinterpret_cast<char*>(&d_subcarrier_error_filt_cycles_mcode), sizeof(double));
                    // SLL commands

                    tmp_double = static_cast< double >( d_integer_code_phase_chips_mcode ) +
                        d_fractional_code_phase_chips_mcode;
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                    d_dump_file.write(reinterpret_cast<char*>(&d_code_error_chips_veml_mcode), sizeof(double));
                    d_dump_file.write(reinterpret_cast<char*>(&d_code_error_filt_chips_veml_mcode), sizeof(double));
                    tmp_double = static_cast< double >( d_integer_subcarrier_phase_cycles_mcode ) +
                        d_fractional_subcarrier_phase_cycles_mcode;
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
            }
            catch (std::ifstream::failure e)
            {
                    LOG(WARNING) << "Exception writing trk dump file " << e.what() << std::endl;
            }
        }
    consume_each(d_current_prn_length_samples); // this is required for gr_block derivates
    //d_sample_counter += d_current_prn_length_samples; //count for the processed samples
    d_current_prn_length_samples = next_prn_length_samples;
    return 1; //output tracking result ALWAYS even in the case of d_enable_tracking==false
}



void gps_l1_mcode_codeless_tracking_cc::set_channel(unsigned int channel)
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



void gps_l1_mcode_codeless_tracking_cc::set_channel_queue(concurrent_queue<int> *channel_internal_queue)
{
    d_channel_internal_queue = channel_internal_queue;
}



void gps_l1_mcode_codeless_tracking_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
    //  Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    //DLOG(INFO) << "Tracking code phase set to " << d_acq_code_phase_samples;
    //DLOG(INFO) << "Tracking carrier doppler set to " << d_acq_carrier_doppler_hz;
    //DLOG(INFO) << "Tracking Satellite set to " << d_satellite;
}

void gps_l1_mcode_codeless_tracking_cc::start_tracking_mcode()
{

    double code_phase_chips_mcode = d_code_phase_chips * GPS_M_CODE_CHIP_RATE_HZ /
        GPS_L1_CA_CODE_RATE_HZ;

    d_integer_code_phase_chips_mcode = 0;

    d_fractional_code_phase_chips_mcode = std::fmod( code_phase_chips_mcode, 1.0 );

    d_fractional_subcarrier_phase_cycles_mcode = d_fractional_code_phase_chips_mcode * d_chips_to_cycles_mcode;

    d_integer_subcarrier_phase_cycles_mcode = static_cast< int64_t >(
            std::floor( d_fractional_subcarrier_phase_cycles_mcode ) );

    d_fractional_subcarrier_phase_cycles_mcode = std::fmod( d_fractional_subcarrier_phase_cycles_mcode, 1.0 );

    d_rem_carr_phase_rad_mcode = d_rem_carr_phase_rad - M_PI/2.0;

    d_code_freq_chips_mcode = d_code_freq_chips * GPS_M_CODE_CHIP_RATE_HZ /
        GPS_L1_CA_CODE_RATE_HZ;

    d_carrier_doppler_hz_mcode = d_carrier_doppler_hz;


    // Initialise the filters:
    // DLL/PLL filter Initialization
    d_code_loop_filter_mcode.set_noise_bandwidth( d_dll_bw_hz_mcode );
    d_carrier_loop_filter_mcode.set_noise_bandwidth( d_pll_bw_hz_mcode );
    d_divergence_loop_filter_mcode.set_noise_bandwidth( d_initial_divergence_loop_filter_bandwidth );
    //d_code_loop_filter_mcode.set_noise_bandwidth( d_initial_dll_bw_hz );
    //d_carrier_loop_filter_mcode.set_noise_bandwidth( d_initial_pll_bw_hz );

    //d_early_late_code_spc_chips = d_initial_early_late_code_space_chips;

    d_carrier_loop_filter_mcode.initialize(0.0); // initialize the carrier filter

    d_code_loop_filter_mcode.initialize( 0.0 );    // initialize the code filter


    std::string sys_ = &d_acquisition_gnss_synchro->System;
    sys = sys_.substr(0, 1);

    // DEBUG OUTPUT
    std::cout << "mcode codeless tracking start on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
    LOG(INFO) << "Starting codeless tracking of mcode for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;

    DLOG(INFO) << "Starting params: current code phase " << code_phase_chips_mcode << " chips.";
    // enable tracking
    d_mcode_tracking_enabled = true;
    d_code_locked_mcode = false;
    d_cn0_estimation_counter = 0;

    // Bump jumping
    d_bj_ve_counter_mcode = 0;
    d_bj_vl_counter_mcode = 0;

    // subcarrieri aiding:
    d_subcarrier_locked_mcode = false;
    d_mean_subcarrier_error_mcode = 0.0;

    d_code_locked_mcode = false;
    d_mean_code_error_mcode = 0.0;

    d_mcode_accumulation_index = 0;

    d_VE_acumm_mcode = 0;
    d_E_acumm_mcode = 0;
    d_P_acumm_mcode = 0;
    d_L_acumm_mcode = 0;
    d_VL_acumm_mcode = 0;

    d_carr_error_hz_mcode = 0;
    d_carr_error_filt_hz_mcode = 0;
    d_subcarrier_error_cycles_mcode = 0;
    d_subcarrier_error_filt_cycles_mcode = 0;
    d_code_error_chips_veml_mcode = 0;
    d_code_error_filt_chips_veml_mcode = 0;

    d_cn0_estimation_counter_mcode = 0;

    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz_mcode
              << " PULL-IN Code Phase [samples]=" << code_phase_chips_mcode;
}

void gps_l1_mcode_codeless_tracking_cc::handle_gnss_message( pmt::pmt_t msg )
{
    std::string telem_msg = gnss_message::get_message( msg );

    std::stringstream log_str("");

    log_str << "Received message " << telem_msg
               << " with timestamp: " << gnss_message::get_timestamp( msg );

    pmt::pmt_t not_found;

    if( gnss_message::get_message( msg ) == "TOW_ACQUIRED" ){
        d_tow_received = true;
        d_last_tow = pmt::to_double( pmt::dict_ref( msg, pmt::mp( "TOW" ), not_found ) ) ;
        log_str << ". TOW: " << d_last_tow;
        d_timestamp_last_tow = gnss_message::get_timestamp( msg );

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

    if( !d_mcode_tracking_enabled && ( d_preamble_start_detected && d_rx_time_set ) )
    {
        log_str << ". Enabling mcode tracking with 1 s ambiguity resolution";
    }

    LOG(INFO) << log_str.str();

}

