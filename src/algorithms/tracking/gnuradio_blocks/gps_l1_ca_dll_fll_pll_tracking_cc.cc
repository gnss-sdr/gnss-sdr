/*!
 * \file gps_l1_ca_dll_fll_pll_tracking_cc.cc
 * \brief code DLL + carrier FLL/PLL tracking
 * \author Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * This file implements the code Delay Locked Loop (DLL) + carrier Phase Locked Loop (PLL) helped with a carrier Frequency Locked Loop (FLL) stage
 * according to the algorithms described in [1]
 * [1] E.D. Kaplan and C. Hegarty, Understanding GPS. Principles and
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

#include "gps_l1_ca_dll_fll_pll_tracking_cc.h"
#include "gps_sdr_signal_processing.h"
#include "GPS_L1_CA.h"
#include "tracking_discriminators.h"
#include "CN_estimators.h"
#include "tracking_FLL_PLL_filter.h"

#include "control_message_factory.h"
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
gps_l1_ca_dll_fll_pll_make_tracking_cc(unsigned int satellite, long if_freq, long fs_in, unsigned
			int vector_length, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int order, float fll_bw_hz, float pll_bw_hz, float dll_bw_hz, float early_late_space_chips) {

	return gps_l1_ca_dll_fll_pll_tracking_cc_sptr(new gps_l1_ca_dll_fll_pll_tracking_cc(satellite, if_freq,
			fs_in, vector_length, queue, dump, dump_filename, order, fll_bw_hz, pll_bw_hz,dll_bw_hz,early_late_space_chips));
}

void gps_l1_ca_dll_fll_pll_tracking_cc::forecast (int noutput_items,
    gr_vector_int &ninput_items_required){
    ninput_items_required[0] =d_vector_length*2; //set the required available samples in each call
}

gps_l1_ca_dll_fll_pll_tracking_cc::gps_l1_ca_dll_fll_pll_tracking_cc(unsigned int satellite, long if_freq, long fs_in, unsigned
		int vector_length, gr_msg_queue_sptr queue, bool dump, std::string dump_filename, int order, float fll_bw_hz, float pll_bw_hz, float dll_bw_hz, float early_late_space_chips) :
	    gr_block ("gps_l1_ca_dll_fll_pll_tracking_cc", gr_make_io_signature (1, 1, sizeof(gr_complex)),
	              gr_make_io_signature(5, 5, sizeof(double))) {
		//gr_sync_decimator ("gps_l1_ca_dll_pll_tracking_cc", gr_make_io_signature (1, 1, sizeof(gr_complex)),
		//		gr_make_io_signature(3, 3, sizeof(float)),vector_length) {
	// initialize internal vars
	d_queue = queue;
	d_dump = dump;
	d_satellite = satellite;
	d_if_freq = if_freq;
	d_fs_in = fs_in;
	d_vector_length = vector_length;
	d_early_late_spc_chips = early_late_space_chips; // Define early-late offset (in chips)
    d_dump_filename=dump_filename;

	// Initialize tracking variables ==========================================
	d_carrier_loop_filter.set_params(fll_bw_hz,pll_bw_hz,order);

    // Get space for a vector with the C/A code replica sampled 1x/chip
    d_ca_code=new gr_complex[(int)GPS_L1_CA_CODE_LENGTH_CHIPS+2];
    // Get space for the resampled early / prompt / late local replicas
    d_early_code= new gr_complex[d_vector_length*2];
    d_prompt_code=new gr_complex[d_vector_length*2];
    d_late_code=new gr_complex[d_vector_length*2];
    // space for carrier wipeoff LO vector
    d_carr_sign=new gr_complex[d_vector_length*2];

    // sample synchronization
    d_sample_counter=0;
    d_sample_counter_seconds=0;
    d_acq_sample_stamp=0;
    d_last_seg=0;// this is for debug output only

    d_enable_tracking=false;

    d_current_prn_length_samples=(int)d_vector_length;

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter=0;
    d_Prompt_buffer=new gr_complex[CN0_ESTIMATION_SAMPLES];
    d_carrier_lock_test=1;
    d_CN0_SNV_dB_Hz=0;
    d_carrier_lock_fail_counter=0;
    d_carrier_lock_threshold=5;

}

void gps_l1_ca_dll_fll_pll_tracking_cc::start_tracking(){

	/*!
	 *  correct the code phase according to the delay between acq and trk
	 */
	unsigned long int acq_trk_diff_samples;
	float acq_trk_diff_seconds;
	acq_trk_diff_samples=d_sample_counter-d_acq_sample_stamp;//-d_vector_length;
	//std::cout<<"acq_trk_diff_samples="<<acq_trk_diff_samples<<"\r\n";
	acq_trk_diff_seconds=(float)acq_trk_diff_samples/(float)d_fs_in;
	//doppler effect
	// Fd=(C/(C+Vr))*F
	float radial_velocity;
	radial_velocity=(GPS_L1_FREQ_HZ+d_acq_carrier_doppler_hz)/GPS_L1_FREQ_HZ;
	// new chip and prn sequence periods based on acq Doppler
	float T_chip_mod_seconds;
	float T_prn_mod_seconds;
	float T_prn_mod_samples;
	d_code_freq_hz=radial_velocity*GPS_L1_CA_CODE_RATE_HZ;
	T_chip_mod_seconds=1/d_code_freq_hz;
	T_prn_mod_seconds=T_chip_mod_seconds*GPS_L1_CA_CODE_LENGTH_CHIPS;
	T_prn_mod_samples=T_prn_mod_seconds*(float)d_fs_in;
    d_next_prn_length_samples=round(T_prn_mod_samples);


    float T_prn_true_seconds = GPS_L1_CA_CODE_LENGTH_CHIPS/GPS_L1_CA_CODE_RATE_HZ;
    float T_prn_true_samples = T_prn_true_seconds*(float)d_fs_in;
    float T_prn_diff_seconds;
    T_prn_diff_seconds=T_prn_true_seconds-T_prn_mod_seconds;
    float N_prn_diff;
    N_prn_diff=acq_trk_diff_seconds/T_prn_true_seconds;
    float corrected_acq_phase_samples,delay_correction_samples;
    corrected_acq_phase_samples=fmod((d_acq_code_phase_samples+T_prn_diff_seconds*N_prn_diff*(float)d_fs_in),T_prn_true_samples);

    if (corrected_acq_phase_samples<0)
    {
    	corrected_acq_phase_samples=T_prn_mod_samples+corrected_acq_phase_samples;
    }
	delay_correction_samples=d_acq_code_phase_samples-corrected_acq_phase_samples;
	d_acq_code_phase_samples=corrected_acq_phase_samples;

	d_carrier_doppler_hz=d_acq_carrier_doppler_hz;
	// DLL/PLL filter initialization
	d_carrier_loop_filter.initialize(d_acq_carrier_doppler_hz);
	d_FLL_wait=1;

	// generate local reference ALWAYS starting at chip 1 (1 sample per chip)
    code_gen_conplex(&d_ca_code[1],d_satellite,0);
    d_ca_code[0]=d_ca_code[(int)GPS_L1_CA_CODE_LENGTH_CHIPS];
    d_ca_code[(int)GPS_L1_CA_CODE_LENGTH_CHIPS+1]=d_ca_code[1];

	d_carrier_lock_fail_counter=0;
	d_Prompt_prev=0;
	d_rem_code_phase_samples=0;
	d_rem_carr_phase=0;
	d_FLL_discriminator_hz=0;
	d_rem_code_phase_samples=0;
	d_next_rem_code_phase_samples=0;
	d_acc_carrier_phase_rad=0;

	d_code_phase_samples = d_acq_code_phase_samples;

	// DEBUG OUTPUT
	std::cout<<"Tracking start on channel "<<d_channel<<" for satellite ID* "<< this->d_satellite<< std::endl;
	DLOG(INFO) << "Start tracking for satellite "<<this->d_satellite<<" received ";

	// enable tracking
	d_pull_in=true;
	d_enable_tracking=true;

	std::cout<<"PULL-IN Doppler [Hz]= "<<d_carrier_doppler_hz<<" Code Phase correction [samples]="<<delay_correction_samples<<" PULL-IN Code Phase [samples]= "<<d_acq_code_phase_samples<<"\r\n";
}

void gps_l1_ca_dll_fll_pll_tracking_cc::update_local_code()
{
	float tcode_chips;
	float rem_code_phase_chips;
	float code_phase_step_chips;
	int associated_chip_index;
	int code_length_chips=(int)GPS_L1_CA_CODE_LENGTH_CHIPS;
	code_phase_step_chips=d_code_freq_hz/((float)d_fs_in);
	rem_code_phase_chips=d_rem_code_phase_samples*(d_code_freq_hz/d_fs_in);
	// unified loop for E, P, L code vectors
	tcode_chips=-rem_code_phase_chips;
    for (int i=0;i<d_current_prn_length_samples;i++)
    {
        associated_chip_index=1+round(fmod(tcode_chips-d_early_late_spc_chips,code_length_chips));
        d_early_code[i] = d_ca_code[associated_chip_index];
        associated_chip_index = 1+round(fmod(tcode_chips, code_length_chips));
        d_prompt_code[i] = d_ca_code[associated_chip_index];
        associated_chip_index = 1+round(fmod(tcode_chips+d_early_late_spc_chips, code_length_chips));
        d_late_code[i] = d_ca_code[associated_chip_index];
        tcode_chips=tcode_chips+code_phase_step_chips;
    }
	//d_code_phase_samples=d_code_phase_samples+(float)d_fs_in*GPS_L1_CA_CODE_LENGTH_CHIPS*(1/d_code_freq_hz-1/GPS_L1_CA_CODE_RATE_HZ);
}

void gps_l1_ca_dll_fll_pll_tracking_cc::update_local_carrier()
{
    float phase, phase_step;
    phase_step = (float)TWO_PI*d_carrier_doppler_hz/(float)d_fs_in;
    phase=d_rem_carr_phase;
    for(int i = 0; i < d_current_prn_length_samples; i++) {
        d_carr_sign[i] = gr_complex(cos(phase),sin(phase));
        phase += phase_step;
    }
    d_rem_carr_phase=fmod(phase,TWO_PI);
    d_acc_carrier_phase_rad=d_acc_carrier_phase_rad+d_rem_carr_phase;
}

gps_l1_ca_dll_fll_pll_tracking_cc::~gps_l1_ca_dll_fll_pll_tracking_cc() {
	d_dump_file.close();
    delete[] d_ca_code;
    delete[] d_early_code;
    delete[] d_prompt_code;
    delete[] d_late_code;
    delete[] d_carr_sign;
    delete[] d_Prompt_buffer;
}

/*! Tracking signal processing
 * Notice that this is a class derived from gr_sync_decimator, so each of the ninput_items has vector_length samples
 */

int gps_l1_ca_dll_fll_pll_tracking_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) {

//	if ((unsigned int)ninput_items[0]<(d_vector_length*2))
//	{
//		std::cout<<"End of signal detected\r\n";
//		const int samples_available = ninput_items[0];
//		consume_each(samples_available);
//		return 0;
//	}
	// process vars
	float code_error_chips=0;
	float correlation_time_s=0;
	float PLL_discriminator_hz=0;
	float carr_nco_hz=0;

	d_Prompt_prev=d_Prompt; // for the FLL discriminator
	d_Early=gr_complex(0,0);
	d_Prompt=gr_complex(0,0);
	d_Late=gr_complex(0,0);

	if (d_enable_tracking==true){
		/*!
		 * Receiver signal alignment
		 */
	    if (d_pull_in==true)
	    {
	    	int samples_offset;

	        // 28/11/2011 ACQ to TRK transition BUG CORRECTION
	        float acq_trk_shif_correction_samples;
	        int acq_to_trk_delay_samples;
	        acq_to_trk_delay_samples=d_sample_counter-d_acq_sample_stamp;
	        acq_trk_shif_correction_samples=d_next_prn_length_samples-fmod((float)acq_to_trk_delay_samples,(float)d_next_prn_length_samples);
	        //std::cout<<"acq_trk_shif_correction="<<acq_trk_shif_correction_samples<<"\r\n";

	        samples_offset=round(d_acq_code_phase_samples+acq_trk_shif_correction_samples);
	        // /todo: Check if the sample counter sent to the next block as a time reference should be incremented AFTER sended or BEFORE
	        d_sample_counter_seconds = d_sample_counter_seconds + (((double)samples_offset)/(double)d_fs_in);
	        d_sample_counter=d_sample_counter+samples_offset; //count for the processed samples
	        d_pull_in=false;
	        //std::cout<<" samples_offset="<<samples_offset<<"\r\n";
	        consume_each(samples_offset); //shift input to perform alignement with local replica
	        return 1;
	    }
	    // get the sample in and out pointers
		const gr_complex* in = (gr_complex*) input_items[0]; //block input samples pointer
		double **out = (double **) &output_items[0]; //block output streams pointer

		// check for samples consistency
		for(int i=0;i<d_current_prn_length_samples;i++) {
			if (std::isnan(in[i].real())==true or std::isnan(in[i].imag())==true)// or std::isinf(in[i].real())==true or std::isinf(in[i].imag())==true)
			{
				const int samples_available= ninput_items[0];
				d_sample_counter=d_sample_counter+samples_available;
				LOG_AT_LEVEL(WARNING) << "Detected NaN samples at sample number "<<d_sample_counter;
				consume_each(samples_available);
				return 0;
			}
		}
		// Update the prn length based on code freq (variable) and
		// sampling frequency (fixed)
		// variable code PRN sample block size
	    d_current_prn_length_samples=d_next_prn_length_samples;

		update_local_code();
		update_local_carrier();

		gr_complex bb_signal_sample(0,0);

		// perform Early, Prompt and Late correlation
		/*!
		 * \todo Use SIMD-enabled correlators
		 */
		for(int i=0;i<d_current_prn_length_samples;i++) {
			//Perform the carrier wipe-off
			bb_signal_sample = in[i] * d_carr_sign[i];
			// Now get early, late, and prompt values for each
			d_Early += bb_signal_sample*d_early_code[i];
			d_Prompt += bb_signal_sample*d_prompt_code[i];
			d_Late += bb_signal_sample*d_late_code[i];
		}

		/*!
		 * DLL, FLL, and PLL discriminators
		 */
		// Compute DLL error
		code_error_chips=dll_nc_e_minus_l_normalized(d_Early,d_Late);

		//compute FLL error
		correlation_time_s=((float)d_current_prn_length_samples)/(float)d_fs_in;
		if (d_FLL_wait==1)
		{
		   d_Prompt_prev=d_Prompt;
		   d_FLL_wait=0;
		}else{
		   d_FLL_discriminator_hz=fll_four_quadrant_atan(d_Prompt_prev, d_Prompt, 0, correlation_time_s)/(float)TWO_PI;
		   d_Prompt_prev=d_Prompt;
		   d_FLL_wait=1;
		}

		// Compute PLL error
		 PLL_discriminator_hz=pll_cloop_two_quadrant_atan(d_Prompt)/(float)TWO_PI;

		 /*!
		  * \todo Update FLL assistance algorithm!
		  */
		 if (((float)d_sample_counter-(float)d_acq_sample_stamp)/(float)d_fs_in>3)
		 {
			 d_FLL_discriminator_hz=0; //disconnect the FLL after the initial lock
		 }
		/*!
		 * DLL and FLL+PLL filter and get current carrier Doppler and code frequency
		 */
		 carr_nco_hz=d_carrier_loop_filter.get_carrier_error(d_FLL_discriminator_hz,PLL_discriminator_hz,correlation_time_s);
		 d_carrier_doppler_hz = (float)d_if_freq + carr_nco_hz;
		 d_code_freq_hz= GPS_L1_CA_CODE_RATE_HZ- (((d_carrier_doppler_hz - (float)d_if_freq)*GPS_L1_CA_CODE_RATE_HZ)/GPS_L1_FREQ_HZ)-code_error_chips;

		/*!
		 * \todo Improve the lock detection algorithm!
		 */
		// ####### CN0 ESTIMATION AND LOCK DETECTORS ######
		if (d_cn0_estimation_counter<CN0_ESTIMATION_SAMPLES)
		{
			// fill buffer with prompt correlator output values
			d_Prompt_buffer[d_cn0_estimation_counter]=d_Prompt;
			d_cn0_estimation_counter++;
		}else{
			d_cn0_estimation_counter=0;
			d_CN0_SNV_dB_Hz=gps_l1_ca_CN0_SNV(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES,d_fs_in);
			d_carrier_lock_test=carrier_lock_detector(d_Prompt_buffer,CN0_ESTIMATION_SAMPLES);
			// ###### TRACKING UNLOCK NOTIFICATION #####
			int tracking_message;
	        if (d_carrier_lock_test<d_carrier_lock_threshold or d_carrier_lock_test>MINIMUM_VALID_CN0)
	        {
	             d_carrier_lock_fail_counter++;
	        }else{
	        	if (d_carrier_lock_fail_counter>0) d_carrier_lock_fail_counter--;
	        }
	        if (d_carrier_lock_fail_counter>MAXIMUM_LOCK_FAIL_COUNTER)
	        {
	        	std::cout<<"Channel "<<d_channel << " loss of lock!\r\n";
	        	tracking_message=3; //loss of lock
	        	d_channel_internal_queue->push(tracking_message);
	        	d_carrier_lock_fail_counter=0;
	        	d_enable_tracking=false; // TODO: check if disabling tracking is consistent with the channel state machine

	        }
	        //std::cout<<"d_carrier_lock_fail_counter"<<d_carrier_lock_fail_counter<<"\r\n";
		}

		/*!
		 * \todo Output the CN0
		 */
		// ########### Output the tracking data to navigation and PVT ##########
		// Output channel 0: Prompt correlator output Q
		*out[0]=(double)d_Prompt.real();
		// Output channel 1: Prompt correlator output I
		*out[1]=(double)d_Prompt.imag();
		// Output channel 2: PRN absolute delay [s]
		*out[2]=d_sample_counter_seconds;
		// Output channel 3: d_acc_carrier_phase_rad [rad]
		*out[3]=(double)d_acc_carrier_phase_rad;
		// Output channel 4: PRN code phase [s]
		*out[4]=(double)d_code_phase_samples*(1/(float)d_fs_in);

		// ########## DEBUG OUTPUT
		/*!
		 *  \todo The stop timer has to be moved to the signal source!
		 */
		// debug: Second counter in channel 0
		if (d_channel==0)
		{
			if (floor(d_sample_counter/d_fs_in)!=d_last_seg)
			{
				d_last_seg=floor(d_sample_counter/d_fs_in);
				std::cout<<"Current input signal time="<<d_last_seg<<" [s]"<<std::endl;
				std::cout<<"Tracking CH "<<d_channel<<" CN0="<<d_CN0_SNV_dB_Hz<<" [dB-Hz]"<<std::endl;
				//std::cout<<"TRK CH "<<d_channel<<" Carrier_lock_test="<<d_carrier_lock_test<< std::endl;
				//if (d_last_seg==5) d_carrier_lock_fail_counter=500; //DEBUG: force unlock!
			}
		}else
		{
			if (floor(d_sample_counter/d_fs_in)!=d_last_seg)
			{
				d_last_seg=floor(d_sample_counter/d_fs_in);
				std::cout<<"Tracking CH "<<d_channel<<" CN0="<<d_CN0_SNV_dB_Hz<<" [dB-Hz]"<<std::endl;
				//std::cout<<"TRK CH "<<d_channel<<" Carrier_lock_test="<<d_carrier_lock_test<< std::endl;
			}
		}

		//predict the next loop PRN period length prediction
		float T_chip_seconds;
		float T_prn_seconds;
		float T_prn_samples;
		float K_blk_samples;
		T_chip_seconds=1/d_code_freq_hz;
		T_prn_seconds=T_chip_seconds*GPS_L1_CA_CODE_LENGTH_CHIPS;
		T_prn_samples=T_prn_seconds*(float)d_fs_in;
		d_rem_code_phase_samples=d_next_rem_code_phase_samples;
		K_blk_samples=T_prn_samples+d_rem_code_phase_samples;

		// Update the current PRN delay (code phase in samples)
	    float T_prn_true_seconds = GPS_L1_CA_CODE_LENGTH_CHIPS/GPS_L1_CA_CODE_RATE_HZ;
	    float T_prn_true_samples = T_prn_true_seconds*(float)d_fs_in;
	    d_code_phase_samples=d_code_phase_samples+T_prn_samples-T_prn_true_samples;
	    if (d_code_phase_samples<0)
	    {
	    	d_code_phase_samples=T_prn_true_samples+d_code_phase_samples;
	    }

	    d_code_phase_samples=fmod(d_code_phase_samples,T_prn_true_samples);
	    d_next_prn_length_samples=round(K_blk_samples);//round to a discrete samples
	    d_next_rem_code_phase_samples=K_blk_samples-d_next_prn_length_samples; //rounding error


	}else{
		double **out = (double **) &output_items[0]; //block output streams pointer
		*out[0]=0;
		*out[1]=0;
		*out[2]=0;
		*out[3]=0;
		*out[4]=0;
	}


	if(d_dump) {
		// MULTIPLEXED FILE RECORDING - Record results to file
		float prompt_I;
		float prompt_Q;
		float tmp_E,tmp_P,tmp_L;
		float tmp_float;
		prompt_I=d_Prompt.imag();
		prompt_Q=d_Prompt.real();
		tmp_E=std::abs<float>(d_Early);
		tmp_P=std::abs<float>(d_Prompt);
		tmp_L=std::abs<float>(d_Late);
      	try {
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
			d_dump_file.write((char*)&d_acc_carrier_phase_rad, sizeof(float));

			// carrier and code frequency
			d_dump_file.write((char*)&d_carrier_doppler_hz, sizeof(float));
			d_dump_file.write((char*)&d_code_freq_hz, sizeof(float));

			//PLL commands
			d_dump_file.write((char*)&PLL_discriminator_hz, sizeof(float));
			d_dump_file.write((char*)&carr_nco_hz, sizeof(float));

			//DLL commands
			d_dump_file.write((char*)&code_error_chips, sizeof(float));
			d_dump_file.write((char*)&d_code_phase_samples, sizeof(float));

			// CN0 and carrier lock test
			d_dump_file.write((char*)&d_CN0_SNV_dB_Hz, sizeof(float));
			d_dump_file.write((char*)&d_carrier_lock_test, sizeof(float));

			// AUX vars (for debug purposes)
			tmp_float=0;
			d_dump_file.write((char*)&tmp_float, sizeof(float));
			d_dump_file.write((char*)&d_sample_counter_seconds, sizeof(double));
      	 }
		  catch (std::ifstream::failure e) {
			std::cout << "Exception writing trk dump file "<<e.what()<<"\r\n";
		  }
	}
	consume_each(d_current_prn_length_samples); // this is necesary in gr_block derivates
    d_sample_counter_seconds = d_sample_counter_seconds + (((double)d_current_prn_length_samples)/(double)d_fs_in);
    d_sample_counter+=d_current_prn_length_samples; //count for the processed samples
	return 1; //output tracking result ALWAYS even in the case of d_enable_tracking==false
}


void gps_l1_ca_dll_fll_pll_tracking_cc::set_acq_code_phase(float code_phase) {
	d_acq_code_phase_samples=code_phase;
	LOG_AT_LEVEL(INFO) << "Tracking code phase set to " << d_acq_code_phase_samples;
}

void gps_l1_ca_dll_fll_pll_tracking_cc::set_acq_doppler(float doppler) {
	d_acq_carrier_doppler_hz = doppler;
	LOG_AT_LEVEL(INFO) << "Tracking carrier doppler set to " << d_acq_carrier_doppler_hz;
}

void gps_l1_ca_dll_fll_pll_tracking_cc::set_satellite(unsigned int satellite) {
	d_satellite = satellite;
	LOG_AT_LEVEL(INFO) << "Tracking Satellite set to " << d_satellite;
}

void gps_l1_ca_dll_fll_pll_tracking_cc::set_channel(unsigned int channel) {
	d_channel = channel;
	LOG_AT_LEVEL(INFO) << "Tracking Channel set to " << d_channel;
	// ############# ENABLE DATA FILE LOG #################
	if (d_dump==true)
	{
		if (d_dump_file.is_open()==false)
		{
			try {
				d_dump_filename.append(boost::lexical_cast<std::string>(d_channel));
				d_dump_filename.append(".dat");
				d_dump_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
				d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
				std::cout<<"Tracking dump enabled on channel "<<d_channel<<" Log file: "<<d_dump_filename.c_str()<<std::endl;
			}
			catch (std::ifstream::failure e) {
				std::cout << "channel "<<d_channel <<" Exception opening trk dump file "<<e.what()<<"\r\n";
			}
		}
	}
}

void gps_l1_ca_dll_fll_pll_tracking_cc::set_acq_sample_stamp(unsigned long int sample_stamp)
{
    d_acq_sample_stamp = sample_stamp;
}

void gps_l1_ca_dll_fll_pll_tracking_cc::set_channel_queue(concurrent_queue<int> *channel_internal_queue)
{
    d_channel_internal_queue = channel_internal_queue;
}
