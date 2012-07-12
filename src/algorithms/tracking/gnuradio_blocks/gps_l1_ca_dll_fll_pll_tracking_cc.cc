/*!
 * \file gps_l1_ca_dll_fll_pll_tracking_cc.cc
 * \brief Implementation of a code DLL + carrier FLL/PLL tracking block
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * This file implements the code Delay Locked Loop (DLL) + carrier
 * Phase Locked Loop (PLL) helped with a carrier Frequency Locked Loop (FLL)
 * according to the algorithms described in:
 * E.D. Kaplan and C. Hegarty, Understanding GPS. Principles and
 * Applications, Second Edition, Artech House Publishers, 2005.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#include "gnss_synchro.h"
#include "gps_l1_ca_dll_fll_pll_tracking_cc.h"
//#include "gnss_signal_processing.h"
#include "gps_sdr_signal_processing.h"
#include "GPS_L1_CA.h"
#include "tracking_discriminators.h"
#include "CN_estimators.h"
#include "tracking_FLL_PLL_filter.h"
#include "control_message_factory.h"
#include "gnss_flowgraph.h"
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <sstream>
#include <cmath>
#include "math.h"
#include <gnuradio/gr_io_signature.h>
#include <glog/log_severity.h>
#include <glog/logging.h>



/*!
 * \todo Include in definition header file
 */
#define CN0_ESTIMATION_SAMPLES 10
#define MINIMUM_VALID_CN0 25
#define MAXIMUM_LOCK_FAIL_COUNTER 200

using google::LogMessage;

gps_l1_ca_dll_fll_pll_tracking_cc_sptr
gps_l1_ca_dll_fll_pll_make_tracking_cc(
		long if_freq,
		long fs_in,
		unsigned
        int vector_length,
        gr_msg_queue_sptr queue,
        bool dump, std::string dump_filename,
        int order,
        float fll_bw_hz,
        float pll_bw_hz,
        float dll_bw_hz,
        float early_late_space_chips)
{

    return gps_l1_ca_dll_fll_pll_tracking_cc_sptr(new Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc(if_freq,
            fs_in, vector_length, queue, dump, dump_filename, order, fll_bw_hz, pll_bw_hz,dll_bw_hz,
            early_late_space_chips));
}




void Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = d_vector_length*2; //set the required available samples in each call
}


Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc::Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        gr_msg_queue_sptr queue,
        bool dump,
        std::string dump_filename,
        int order,
        float fll_bw_hz,
        float pll_bw_hz,
        float dll_bw_hz,
        float early_late_space_chips) :
        gr_block ("Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc", gr_make_io_signature (1, 1, sizeof(gr_complex)),
                gr_make_io_signature(1, 1, sizeof(Gnss_Synchro)))
{
    // initialize internal vars
    d_queue = queue;
    d_dump = dump;

    d_acquisition_gnss_synchro=NULL;

    d_if_freq = (double)if_freq;
    d_fs_in = (double)fs_in;
    d_vector_length = vector_length;
    d_early_late_spc_chips = (double)early_late_space_chips; // Define early-late offset (in chips)
    d_dump_filename = dump_filename;

    // Initialize tracking variables ==========================================
    d_carrier_loop_filter.set_params(fll_bw_hz,pll_bw_hz,order);

    // Get space for a vector with the C/A code replica sampled 1x/chip
    d_ca_code = new gr_complex[(int)GPS_L1_CA_CODE_LENGTH_CHIPS + 2];

    /* If an array is partitioned for more than one thread to operate on,
     * having the sub-array boundaries unaligned to cache lines could lead
     * to performance degradation. Here we allocate memory
     * (gr_comlex array of size 2*d_vector_length) aligned to cache of 16 bytes
     */
    // todo: do something if posix_memalign fails
    // Get space for the resampled early / prompt / late local replicas
    if (posix_memalign((void**)&d_early_code, 16, d_vector_length * sizeof(gr_complex) * 2) == 0){};
    if (posix_memalign((void**)&d_late_code, 16, d_vector_length * sizeof(gr_complex) * 2) == 0){};
    if (posix_memalign((void**)&d_prompt_code, 16, d_vector_length * sizeof(gr_complex) * 2) == 0){};
    // space for carrier wipeoff and signal baseband vectors
    if (posix_memalign((void**)&d_carr_sign, 16, d_vector_length * sizeof(gr_complex) * 2) == 0){};
    // correlator outputs (scalar)
    if (posix_memalign((void**)&d_Early, 16, sizeof(gr_complex)) == 0){};
    if (posix_memalign((void**)&d_Prompt, 16, sizeof(gr_complex)) == 0){};
    if (posix_memalign((void**)&d_Late, 16, sizeof(gr_complex)) == 0){};

    // sample synchronization
    d_sample_counter = 0;
    d_acq_sample_stamp = 0;
    d_last_seg = 0;// this is for debug output only

    d_code_phase_samples=0;

    d_enable_tracking = false;

    d_current_prn_length_samples = (int)d_vector_length;

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[CN0_ESTIMATION_SAMPLES];
    d_carrier_lock_test = 1;
    d_CN0_SNV_dB_Hz = 0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_threshold = 20;

    systemName["G"] = std::string("GPS");
    systemName["R"] = std::string("GLONASS");
    systemName["S"] = std::string("SBAS");
    systemName["E"] = std::string("Galileo");
    systemName["C"] = std::string("Compass");
}




void Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc::start_tracking()
{
    /*
     *  correct the code phase according to the delay between acq and trk
     */

    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp =  d_acquisition_gnss_synchro->Acq_samplestamp_samples;

    unsigned long int acq_trk_diff_samples;
    double acq_trk_diff_seconds;
    acq_trk_diff_samples = d_sample_counter - d_acq_sample_stamp;//-d_vector_length;
    acq_trk_diff_seconds = (double)acq_trk_diff_samples / d_fs_in;
    //doppler effect
    // Fd=(C/(C+Vr))*F
    double radial_velocity;
    radial_velocity = (GPS_L1_FREQ_HZ + d_acq_carrier_doppler_hz) / GPS_L1_FREQ_HZ;
    // new chip and prn sequence periods based on acq Doppler
    double T_chip_mod_seconds;
    double T_prn_mod_seconds;
    double T_prn_mod_samples;
    d_code_freq_hz = radial_velocity * GPS_L1_CA_CODE_RATE_HZ;
    T_chip_mod_seconds = 1 / d_code_freq_hz;
    T_prn_mod_seconds = T_chip_mod_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
    T_prn_mod_samples = T_prn_mod_seconds * d_fs_in;
    d_current_prn_length_samples = round(T_prn_mod_samples);

    double T_prn_true_seconds = GPS_L1_CA_CODE_LENGTH_CHIPS / GPS_L1_CA_CODE_RATE_HZ;
    double T_prn_true_samples = T_prn_true_seconds * d_fs_in;
    double T_prn_diff_seconds;
    T_prn_diff_seconds = T_prn_true_seconds - T_prn_mod_seconds;
    double N_prn_diff;
    N_prn_diff = acq_trk_diff_seconds / T_prn_true_seconds;
    double corrected_acq_phase_samples, delay_correction_samples;
    corrected_acq_phase_samples = fmod((d_acq_code_phase_samples + T_prn_diff_seconds * N_prn_diff * d_fs_in), T_prn_true_samples);

    if (corrected_acq_phase_samples < 0)
        {
            corrected_acq_phase_samples = T_prn_mod_samples + corrected_acq_phase_samples;
        }
    delay_correction_samples = d_acq_code_phase_samples - corrected_acq_phase_samples;
    d_acq_code_phase_samples = corrected_acq_phase_samples;

    d_carrier_doppler_hz = d_acq_carrier_doppler_hz;
    // DLL/PLL filter initialization
    d_carrier_loop_filter.initialize(d_acq_carrier_doppler_hz);
    d_FLL_wait = 1;

    // generate local reference ALWAYS starting at chip 1 (1 sample per chip)
    gps_l1_ca_code_gen_complex(&d_ca_code[1], d_acquisition_gnss_synchro->PRN, 0);

    d_ca_code[0] = d_ca_code[(int)GPS_L1_CA_CODE_LENGTH_CHIPS];
    d_ca_code[(int)GPS_L1_CA_CODE_LENGTH_CHIPS + 1] = d_ca_code[1];

    d_carrier_lock_fail_counter = 0;
    d_Prompt_prev = 0;
    d_rem_code_phase_samples = 0;
    d_rem_carr_phase = 0;
    d_FLL_discriminator_hz = 0;
    d_rem_code_phase_samples = 0;
    d_acc_carrier_phase_rad = 0;

    std::string sys_ = &d_acquisition_gnss_synchro->System;
    sys = sys_.substr(0,1);

    // DEBUG OUTPUT
    std::cout << "Tracking start on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
    DLOG(INFO) << "Start tracking for satellite " << d_acquisition_gnss_synchro->System << " "<< d_acquisition_gnss_synchro->PRN  << " received ";

    // enable tracking Gnss_Satellite(systemName[&d_acquisition_gnss_synchro->System], d_acquisition_gnss_synchro->PRN)
    d_pull_in = true;
    d_enable_tracking = true;

    std::cout << "PULL-IN Doppler [Hz]= " << d_carrier_doppler_hz
            << " Code Phase correction [samples]=" << delay_correction_samples
            << " PULL-IN Code Phase [samples]= " << d_acq_code_phase_samples << std::endl;





}





void Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc::update_local_code()
{
	double tcode_chips;
	double rem_code_phase_chips;
	double code_phase_step_chips;
    int associated_chip_index;
    int code_length_chips = (int)GPS_L1_CA_CODE_LENGTH_CHIPS;
    code_phase_step_chips = d_code_freq_hz / d_fs_in;
    rem_code_phase_chips = d_rem_code_phase_samples * (d_code_freq_hz / d_fs_in);
    // unified loop for E, P, L code vectors
    tcode_chips = -rem_code_phase_chips;
    for (int i=0; i<d_current_prn_length_samples; i++)
        {
            associated_chip_index = 1 + round(fmod(tcode_chips - d_early_late_spc_chips, code_length_chips));
            d_early_code[i] = d_ca_code[associated_chip_index];
            associated_chip_index = 1 + round(fmod(tcode_chips, code_length_chips));
            d_prompt_code[i] = d_ca_code[associated_chip_index];
            associated_chip_index = 1 + round(fmod(tcode_chips + d_early_late_spc_chips, code_length_chips));
            d_late_code[i] = d_ca_code[associated_chip_index];
            tcode_chips = tcode_chips + code_phase_step_chips;
        }
}





void Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc::update_local_carrier()
{
	double phase, phase_step;
    phase_step = GPS_TWO_PI * d_carrier_doppler_hz / d_fs_in;
    phase = d_rem_carr_phase;
    for(int i = 0; i < d_current_prn_length_samples; i++)
        {
            d_carr_sign[i] = gr_complex(cos(phase), sin(phase));
            phase += phase_step;
        }
    d_rem_carr_phase = fmod(phase, GPS_TWO_PI);
    d_acc_carrier_phase_rad = d_acc_carrier_phase_rad + phase;
}





Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc::~Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc()
{
    d_dump_file.close();
    delete[] d_ca_code;

    free(d_prompt_code);
    free(d_late_code);
    free(d_early_code);
    free(d_carr_sign);
    free(d_Early);
    free(d_Prompt);
    free(d_Late);
    delete[] d_Prompt_buffer;
}



/* Tracking signal processing
 * Notice that this is a class derived from gr_sync_decimator, so each of the ninput_items has vector_length samples
 */

int Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{

    double code_error_chips = 0;
    double correlation_time_s = 0;
    double PLL_discriminator_hz = 0;
    double carr_nco_hz = 0;
    // get the sample in and out pointers
    const gr_complex* in = (gr_complex*) input_items[0]; //block input samples pointer
    Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0];         //block output streams pointer

    d_Prompt_prev = *d_Prompt; // for the FLL discriminator

    if (d_enable_tracking == true)
        {
			// GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
			Gnss_Synchro current_synchro_data;
			// Fill the acquisition data
			current_synchro_data=*d_acquisition_gnss_synchro;
            /*
             * Receiver signal alignment
             */
            if (d_pull_in == true)
                {
                    int samples_offset;
                    double acq_trk_shif_correction_samples;
                    int acq_to_trk_delay_samples;
                    acq_to_trk_delay_samples = d_sample_counter-d_acq_sample_stamp;
                    acq_trk_shif_correction_samples = d_current_prn_length_samples - fmod((double)acq_to_trk_delay_samples, (double)d_current_prn_length_samples);
                    samples_offset = round(d_acq_code_phase_samples + acq_trk_shif_correction_samples);
                    // /todo: Check if the sample counter sent to the next block as a time reference should be incremented AFTER sended or BEFORE
                    d_sample_counter = d_sample_counter + samples_offset; //count for the processed samples
                    d_pull_in = false;
                    consume_each(samples_offset); //shift input to perform alignment with local replica

    				// make an output to not stop the rest of the processing blocks
    	            current_synchro_data.Prompt_I=0.0;
    	            current_synchro_data.Prompt_Q=0.0;
    	            current_synchro_data.Tracking_timestamp_secs=(double)d_sample_counter/d_fs_in;
    	            current_synchro_data.Carrier_phase_rads=0.0;
    	            current_synchro_data.Code_phase_secs=0.0;
    	            current_synchro_data.CN0_dB_hz=0.0;
    	            current_synchro_data.Flag_valid_tracking=false;

    	            *out[0] =current_synchro_data;

                    return 1;
                }


            update_local_code();
            update_local_carrier();

            // perform Early, Prompt and Late correlation
            d_correlator.Carrier_wipeoff_and_EPL_volk(d_current_prn_length_samples,
                    in,
                    d_carr_sign,
                    d_early_code,
                    d_prompt_code,
                    d_late_code,
                    d_Early,
                    d_Prompt,
                    d_Late);

            // check for samples consistency (this should be done before in the receiver / here only if the source is a file)
            if (std::isnan((*d_Prompt).real()) == true or std::isnan((*d_Prompt).imag()) == true )// or std::isinf(in[i].real())==true or std::isinf(in[i].imag())==true)
			{
				const int samples_available = ninput_items[0];
				d_sample_counter = d_sample_counter + samples_available;
				LOG_AT_LEVEL(WARNING) << "Detected NaN samples at sample number " << d_sample_counter;
				consume_each(samples_available);

				// make an output to not stop the rest of the processing blocks
	            current_synchro_data.Prompt_I=0.0;
	            current_synchro_data.Prompt_Q=0.0;
	            current_synchro_data.Tracking_timestamp_secs=(double)d_sample_counter/d_fs_in;
	            current_synchro_data.Carrier_phase_rads=0.0;
	            current_synchro_data.Code_phase_secs=0.0;
	            current_synchro_data.CN0_dB_hz=0.0;
	            current_synchro_data.Flag_valid_tracking=false;

	            *out[0] =current_synchro_data;

				return 1;
			}

            /*
             * DLL, FLL, and PLL discriminators
             */
            // Compute DLL error
            code_error_chips = dll_nc_e_minus_l_normalized(*d_Early,*d_Late);

            //compute FLL error
            correlation_time_s = ((double)d_current_prn_length_samples) / d_fs_in;
            if (d_FLL_wait == 1)
                {
                    d_Prompt_prev = *d_Prompt;
                    d_FLL_wait = 0;
                }
            else
                {
                    d_FLL_discriminator_hz = fll_four_quadrant_atan(d_Prompt_prev, *d_Prompt, 0, correlation_time_s) / GPS_TWO_PI;
                    d_Prompt_prev = *d_Prompt;
                    d_FLL_wait = 1;
                }

            // Compute PLL error
            PLL_discriminator_hz = pll_cloop_two_quadrant_atan(*d_Prompt) / GPS_TWO_PI;

            /*
             * \todo Update FLL assistance algorithm!
             */
            if ((((double)d_sample_counter - (double)d_acq_sample_stamp) / d_fs_in) > 3)
                {
                    d_FLL_discriminator_hz = 0; //disconnect the FLL after the initial lock
                }
            /*
             * DLL and FLL+PLL filter and get current carrier Doppler and code frequency
             */
            carr_nco_hz = d_carrier_loop_filter.get_carrier_error(d_FLL_discriminator_hz, PLL_discriminator_hz, correlation_time_s);
            d_carrier_doppler_hz = d_if_freq + carr_nco_hz;
            d_code_freq_hz = GPS_L1_CA_CODE_RATE_HZ - (((d_carrier_doppler_hz - d_if_freq) * GPS_L1_CA_CODE_RATE_HZ) / GPS_L1_FREQ_HZ) - code_error_chips;

            /*!
             * \todo Improve the lock detection algorithm!
             */
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
                    d_CN0_SNV_dB_Hz = gps_l1_ca_CN0_SNV(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES, d_fs_in);
                    d_carrier_lock_test = carrier_lock_detector(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES);
                    // ###### TRACKING UNLOCK NOTIFICATION #####
                    if (std::abs(d_carrier_lock_test) > d_carrier_lock_threshold or d_CN0_SNV_dB_Hz < MINIMUM_VALID_CN0)
                        {
                            d_carrier_lock_fail_counter++;
                        }
                    else
                        {
                            if (d_carrier_lock_fail_counter > 0) d_carrier_lock_fail_counter--;
                        }
                    if (d_carrier_lock_fail_counter > MAXIMUM_LOCK_FAIL_COUNTER)
                        {
                            std::cout << "Channel " << d_channel << " loss of lock!" << std::endl;
                        	ControlMessageFactory* cmf = new ControlMessageFactory();
                        	if (d_queue != gr_msg_queue_sptr()) {
                        		d_queue->handle(cmf->GetQueueMessage(d_channel, 2));
                        	}
                        	delete cmf;
                            d_carrier_lock_fail_counter = 0;
                            d_enable_tracking = false; // TODO: check if disabling tracking is consistent with the channel state machine
                        }
                }

            // ########## DEBUG OUTPUT
            /*!
             *  \todo The stop timer has to be moved to the signal source!
             */
            // debug: Second counter in channel 0
            if (d_channel == 0)
                {
                    if (floor(d_sample_counter/d_fs_in) != d_last_seg)
                        {
                            d_last_seg = floor(d_sample_counter/d_fs_in);
                            std::cout << "Current input signal time = " << d_last_seg << " [s]" << std::endl;
                            std::cout << "Tracking CH " << d_channel <<  ": Satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN)  << ", CN0 = " << d_CN0_SNV_dB_Hz << " [dB-Hz]" << std::endl;
                        }
                }
            else
                {
                    if (floor(d_sample_counter/d_fs_in) != d_last_seg)
                        {
                            d_last_seg = floor(d_sample_counter/d_fs_in);
                            std::cout << "Tracking CH " << d_channel <<  ": Satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN)  << ", CN0 = " << d_CN0_SNV_dB_Hz << " [dB-Hz]" << std::endl;
                        }
                }

            //predict the next loop PRN period length prediction
            double T_chip_seconds;
            double T_prn_seconds;
            double T_prn_samples;
            double K_blk_samples;
            T_chip_seconds = 1/d_code_freq_hz;
            T_prn_seconds = T_chip_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
            T_prn_samples = T_prn_seconds * d_fs_in;
            K_blk_samples = T_prn_samples + d_rem_code_phase_samples;
            d_current_prn_length_samples = round(K_blk_samples); //round to a discrete sample
            d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error

            // ########### Output the tracking data to navigation and PVT ##########
            current_synchro_data.Prompt_I=(double)(*d_Prompt).imag();
            current_synchro_data.Prompt_Q=(double)(*d_Prompt).real();
            // Tracking_timestamp_secs is aligned with the PRN start sample
            current_synchro_data.Tracking_timestamp_secs=((double)d_sample_counter+(double)d_current_prn_length_samples+d_rem_code_phase_samples)/d_fs_in;
            // This tracking block aligns the Tracking_timestamp_secs with the start sample of the PRN, Code_phase_secs=0
            current_synchro_data.Code_phase_secs=0;
            current_synchro_data.Carrier_phase_rads=d_acc_carrier_phase_rad;
            current_synchro_data.CN0_dB_hz=d_CN0_SNV_dB_Hz;
            current_synchro_data.Flag_valid_tracking=true;
            *out[0] =current_synchro_data;
        }
    else
        {
            *d_Early  = gr_complex(0,0);
            *d_Prompt = gr_complex(0,0);
            *d_Late   = gr_complex(0,0);
            Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0]; //block output streams pointer
            *out[0]=*d_acquisition_gnss_synchro;
        }


    if(d_dump)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            float prompt_I;
            float prompt_Q;
            float tmp_E, tmp_P, tmp_L;
            float tmp_float;
            double tmp_double;
            prompt_I = (*d_Prompt).imag();
            prompt_Q = (*d_Prompt).real();
            tmp_E=std::abs<float>(*d_Early);
            tmp_P=std::abs<float>(*d_Prompt);
            tmp_L=std::abs<float>(*d_Late);
            try
            {
                    // EPR
                    d_dump_file.write((char*)&tmp_E, sizeof(float));
                    d_dump_file.write((char*)&tmp_P, sizeof(float));
                    d_dump_file.write((char*)&tmp_L, sizeof(float));
                    // PROMPT I and Q (to analyze navigation symbols)
                    d_dump_file.write((char*)&prompt_I, sizeof(float));
                    d_dump_file.write((char*)&prompt_Q, sizeof(float));
                    // PRN start sample stamp
                    //tmp_float=(float)d_sample_counter;
                    d_dump_file.write((char*)&d_sample_counter, sizeof(unsigned long int));
                    // accumulated carrier phase
                    tmp_float=(float)d_acc_carrier_phase_rad;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));

                    // carrier and code frequency
                    tmp_float=(float)d_carrier_doppler_hz;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));
                    tmp_float=(float)d_code_freq_hz;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));

                    //PLL commands
                    tmp_float=(float)PLL_discriminator_hz;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));
                    tmp_float=(float)carr_nco_hz;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));

                    //DLL commands
                    tmp_float=(float)code_error_chips;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));
                    tmp_float=(float)d_code_phase_samples;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));

                    // CN0 and carrier lock test
                    tmp_float=(float)d_CN0_SNV_dB_Hz;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));
                    tmp_float=(float)d_carrier_lock_test;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));

                    // AUX vars (for debug purposes)
                    tmp_float = (float)d_rem_code_phase_samples;
                    d_dump_file.write((char*)&tmp_float, sizeof(float));
                    tmp_double=(double)(d_sample_counter+d_current_prn_length_samples);
                    d_dump_file.write((char*)&tmp_double, sizeof(double));
            }
            catch (std::ifstream::failure e)
            {
                    std::cout << "Exception writing trk dump file "<< e.what() << std::endl;
            }
        }
    consume_each(d_current_prn_length_samples); // this is necesary in gr_block derivates
    d_sample_counter += d_current_prn_length_samples; //count for the processed samples
    return 1; //output tracking result ALWAYS even in the case of d_enable_tracking==false
}


void Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc::set_channel(unsigned int channel)
{
    d_channel = channel;
    DLOG(INFO) << "Tracking Channel set to " << d_channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                    {
                            d_dump_filename.append(boost::lexical_cast<std::string>(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            std::cout << "Tracking dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str() << std::endl;
                    }
                    catch (std::ifstream::failure e)
                    {
                            std::cout << "channel " << d_channel << " Exception opening trk dump file " << e.what() << std::endl;
                    }
                }
        }
}



void Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc::set_channel_queue(concurrent_queue<int> *channel_internal_queue)
{
    d_channel_internal_queue = channel_internal_queue;
}

void Gps_L1_Ca_Dll_Fll_Pll_Tracking_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro=p_gnss_synchro;
}
