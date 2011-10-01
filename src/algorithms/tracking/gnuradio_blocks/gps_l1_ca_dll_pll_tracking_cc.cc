//! Single-delta GPS L1 CA DLL+PLL tracking
/*!
 * Tracking based on the Kay Borre book MATLAB-based GPS receiver
 */

/**
 * Copyright notice
 */

/**
 * Author: Javier Arribas, 2011. jarribas(at)cttc.es
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gps_l1_ca_dll_pll_tracking_cc.h"
#include "gps_sdr_signal_processing.h"

//#include "gps_sdr_signaldef.h"
#include "gps_sdr_simd.h"
#include "gps_sdr_x86.h"

#include "control_message_factory.h"

#include <iostream>
#include <sstream>
#include <cmath>
#include "math.h"

#include <gnuradio/gr_io_signature.h>



#include <glog/log_severity.h>
#include <glog/logging.h>

using google::LogMessage;

gps_l1_ca_dll_pll_tracking_cc_sptr
gps_l1_ca_dll_pll_make_tracking_cc(unsigned int satellite, long if_freq, long fs_in, unsigned
			int vector_length, gr_msg_queue_sptr queue, bool dump) {

	return gps_l1_ca_dll_pll_tracking_cc_sptr(new gps_l1_ca_dll_pll_tracking_cc(satellite, if_freq,
			fs_in, vector_length, queue, dump));
}


gps_l1_ca_dll_pll_tracking_cc::gps_l1_ca_dll_pll_tracking_cc(unsigned int satellite, long if_freq, long fs_in, unsigned
		int vector_length, gr_msg_queue_sptr queue, bool dump) :
		gr_sync_decimator ("gps_l1_ca_dll_pll_tracking_cc", gr_make_io_signature (1, 1, sizeof(gr_complex)),
				gr_make_io_signature(3, 3, sizeof(float)),vector_length) {
	// initialize internal vars
	d_queue = queue;
	d_dump = dump;
	d_satellite = satellite;
	d_if_freq = if_freq;
	d_fs_in = fs_in;
	d_vector_length = vector_length;

	// Initialize tracking variables ==========================================
	//TODO: Include this setting in configuration file

	//--- DLL variables --------------------------------------------------------
	d_early_late_spc = 0.5; // Define early-late offset (in chips)
	d_pdi_code = 0.001;// Summation interval for code
	d_dllnoisebandwidth=2.0; //Hz
	d_dlldampingratio=0.7;
	calculate_lopp_coef(&d_tau1_code, &d_tau2_code, d_dllnoisebandwidth, d_dlldampingratio,1.0);// Calculate filter coefficient values

	//--- PLL variables --------------------------------------------------------
	d_pdi_carr = 0.001;// Summation interval for carrier
	d_plldampingratio=0.7;
	d_pllnoisebandwidth=25;

	//Calculate filter coefficient values
	calculate_lopp_coef(&d_tau1_carr, &d_tau2_carr, d_pllnoisebandwidth, d_plldampingratio,0.25);// Calculate filter coefficient values

	// Initialization of local code replica

    // Get a vector with the C/A code sampled 1x/chip
	d_code_length=1023;
	d_ca_code=(gr_complex*)malloc(sizeof(gr_complex)*(d_code_length+2));

    // Get space for the resampled early / prompt / late local replicas
    d_early_code=(gr_complex*)malloc(sizeof(gr_complex)*d_vector_length);
    // Get space for the resampled early / prompt / late local replicas
    d_prompt_code=(gr_complex*)malloc(sizeof(gr_complex)*d_vector_length);
    // Get space for the resampled early / prompt / late local replicas
    d_late_code=(gr_complex*)malloc(sizeof(gr_complex)*d_vector_length);

    d_carr_sign=(gr_complex*)malloc(sizeof(gr_complex)*d_vector_length);
    d_bb_sign=(gr_complex*)malloc(sizeof(gr_complex)*d_vector_length);

    //--- Perform initializations ------------------------------

    // define initial code frequency basis of NCO
    d_code_freq      = 1023000; //Hz
    // define residual code phase (in chips)
    d_rem_code_phase  = 0.0;
    // define carrier frequency which is used over whole tracking period
    // it must be set with set_acq_code_phase() and set_acq_doppler()

    // define residual carrier phase
    d_rem_carr_phase  = 0.0;

    // code tracking loop parameters
    d_old_code_nco   = 0.0;
    d_old_code_error = 0.0;

    // carrier/Costas loop parameters
    d_old_carr_nco   = 0.0;
    d_old_carr_error = 0.0;

    d_absolute_code_phase_chips = 0;

    // sample synchronization
    d_sample_counter=0;
    d_acq_sample_stamp=0;

    d_dump_filename="./data/trk_epr.dat";
    d_enable_tracking=false;
	d_last_seg=0;
}

void gps_l1_ca_dll_pll_tracking_cc::calculate_lopp_coef(float* tau1,float* tau2, float lbw, float zeta, float k){
	// Solve natural frequency
	float Wn;
	Wn = lbw*8*zeta / (4*zeta*zeta + 1);

	// solve for t1 & t2
	*tau1 = k / (Wn * Wn);
	*tau2 = (2.0 * zeta) / Wn;
	//std::cout<<"Tau1= "<<*tau1<<std::endl;
	//std::cout<<"Tau2= "<<*tau2<<std::endl;
}

void gps_l1_ca_dll_pll_tracking_cc::update_local_code_refs()
{
	float tcode;
	float max_tcode;
	unsigned int i=0;
	int tmp_index;
	float block_correction;

//	std::cout<<"d_code_phase_step"<<d_code_phase_step<<std::endl;
//	std::cout<<"d_rem_code_phase"<<d_rem_code_phase<<std::endl;
//	std::cout<<"d_blk_size"<<d_blk_size<<std::endl;

	// Define index into early code vector
	max_tcode=((float)d_blk_size-1.0)*d_code_phase_step+d_rem_code_phase-d_early_late_spc;

	for (tcode=d_rem_code_phase-d_early_late_spc;tcode<max_tcode;tcode+=d_code_phase_step)
	{
		tmp_index=ceil(fmod(tcode,d_code_length));
		d_early_code[i]=d_ca_code[tmp_index];
		i++;
//		if (i==d_vector_length){
//			std::cout<<"Break "<<std::endl;
//			break;
//		}
	}
	// Define index into late code vector
	i=0;
	max_tcode=((float)d_blk_size-1.0)*d_code_phase_step+d_rem_code_phase+d_early_late_spc;
	for (tcode=d_rem_code_phase+d_early_late_spc;tcode<max_tcode;tcode+=d_code_phase_step)
	{
		tmp_index=ceil(fmod(tcode,d_code_length));
		d_late_code[i]=d_ca_code[tmp_index];
		i++;
	}
	// Define index into prompt code vector
	i=0;
	max_tcode=((float)d_blk_size-1.0)*d_code_phase_step+d_rem_code_phase;
	for (tcode=d_rem_code_phase;tcode<max_tcode;tcode+=d_code_phase_step)
	{
		tmp_index=ceil(fmod(tcode,d_code_length));
		d_prompt_code[i]=d_ca_code[tmp_index];
		i++;
	}

	block_correction=(float)d_blk_size*(1/(float)d_fs_in)-(float)d_code_length*(1/d_code_freq);
	d_rem_code_phase = (max_tcode + d_code_phase_step - block_correction) - 1023.0;
	//std::cout<<"d_rem_code_phase="<<d_rem_code_phase<<std::endl;
}

void gps_l1_ca_dll_pll_tracking_cc::update_local_carrier()
{
		float phase, phase_step;

		phase_step = (float)TWO_PI*d_carrier_doppler/d_fs_in;
		phase=d_rem_carr_phase;
		for(unsigned int i = 0; i < d_vector_length; i++)	{
			d_carr_sign[i] = std::complex<float>(cos(phase),sin(phase));
			phase += phase_step;
		}
		d_rem_carr_phase=fmod(phase,TWO_PI);
}
gps_l1_ca_dll_pll_tracking_cc::~gps_l1_ca_dll_pll_tracking_cc() {
	d_dump_file.close();

}

void gps_l1_ca_dll_pll_tracking_cc::start_tracking(){
	unsigned long int acq_sample_difference;
	int trk_corrected_code_phase;

	acq_sample_difference=this->d_sample_counter-d_acq_sample_stamp-d_vector_length;

	float velocity_ratio,code_freq_mod,T_prn,T_prn_mod,T_chip_mod;

	const float carrier_freq=1575420000;
	velocity_ratio=(carrier_freq+d_carrier_doppler)/carrier_freq;

	code_freq_mod=velocity_ratio*d_code_freq;

	T_prn=(1/d_code_freq)*(float)d_code_length;

	T_chip_mod=1/code_freq_mod;
	T_prn_mod=T_chip_mod*(float)d_code_length;

	trk_corrected_code_phase=round(fmod((d_code_phase+(float)acq_sample_difference+(T_prn-T_prn_mod)*((float)acq_sample_difference/(float)d_vector_length)*(float)d_fs_in),(float)d_vector_length));

	if (trk_corrected_code_phase<0)
	{
		trk_corrected_code_phase=d_vector_length+trk_corrected_code_phase;
	}
	d_absolute_code_phase_chips=(float)trk_corrected_code_phase;

	/*
	std::cout<<"Acq sample stamp"<<d_acq_sample_stamp<<std::endl;
	std::cout<<"Acq - Trk sample difference "<<acq_sample_difference<<std::endl;
	std::cout<<"Trk local code correction "<<trk_corrected_code_phase<<std::endl;
	std::cout<<"Rounded"<<round((float)d_code_length*(float)trk_corrected_code_phase/(float)d_vector_length);
	 */
	// generate local reference with the acquisition shift corrected
    code_gen_conplex(&d_ca_code[1],d_satellite,1023-round((float)d_code_length*(float)trk_corrected_code_phase/(float)d_vector_length));
    // Then make it possible to do early and late versions
    d_ca_code[0]=d_ca_code[1023];
    d_ca_code[1024]=d_ca_code[1];

	DLOG(INFO) << "Start tracking for satellite "<<this->d_satellite<<" received ";
	d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
	d_enable_tracking=true;
	std::cout<<"Tracking start on channel "<<d_channel<<" for satellite ID "<< this->d_satellite << std::endl;
}

/*! Tracking signal processing
 * Notice that this is a class derived from gr_sync_decimator, so each of the ninput_items has vector_length samples
 */
int gps_l1_ca_dll_pll_tracking_cc::work (int noutput_items,gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) {

	d_sample_counter+=d_vector_length; //count for the processed samples
	if (d_enable_tracking==true){
		float carr_error;
		float carr_nco;
		float code_error;
		float code_nco;
		float tmp_E,tmp_P,tmp_L;

		const gr_complex* in = (gr_complex*) input_items[0];

		float **out = (float **) &output_items[0];

		// Find the size of a "block" or code period in whole samples

		// Update the phasestep based on code freq (variable) and
		// sampling frequency (fixed)
		d_code_phase_step = d_code_freq / d_fs_in;

		// original variable block size (MATLAB)
		//d_blk_size = ceil(((float)d_code_length-d_rem_code_phase) / d_code_phase_step);

		// fixed block size (Javi)
		d_blk_size = ceil((float)d_code_length*(1/1023000.0)*(float)d_fs_in);

		this->update_local_code_refs();
		this->update_local_carrier();

		d_E_I=0.0;
		d_P_I=0.0;
		d_L_I=0.0;

		d_E_Q=0.0;
		d_P_Q=0.0;
		d_L_Q=0.0;

		for(unsigned int i=0;i<d_vector_length;i++) {
			//Perform the carrier wipe-off
			d_bb_sign[i] = in[i] * d_carr_sign[i];
			// Now get early, late, and prompt values for each
			d_E_Q += d_early_code[i].real()*d_bb_sign[i].real();

			d_E_I += d_early_code[i].real()*d_bb_sign[i].imag();

			d_P_Q += d_prompt_code[i].real()*d_bb_sign[i].real();
			d_P_I += d_prompt_code[i].real()*d_bb_sign[i].imag();

			d_L_Q += d_late_code[i].real()*d_bb_sign[i].real();
			d_L_I += d_late_code[i].real()*d_bb_sign[i].imag();
		}

		// Find PLL error and update carrier NCO -
		// Implement carrier loop discriminator (phase detector)
		//carr_error = atan(d_P.imag() / d_P.real()) / TWO_PI;

		carr_error = atan(d_P_Q / d_P_I) / TWO_PI;

		// Implement carrier loop filter and generate NCO command

		carr_nco = d_old_carr_nco+(d_tau2_carr/d_tau1_carr)*(carr_error - d_old_carr_error) + carr_error * (d_pdi_carr/d_tau1_carr);
		d_old_carr_nco   = carr_nco;
		d_old_carr_error = carr_error;

		// Modify carrier freq based on NCO command
		d_carrier_doppler = d_carr_freq_basis + carr_nco;

		// Find DLL error and update code NCO -
		//code_error = (abs(d_E)-abs(d_L))/(abs(d_E)+abs(d_L));

		code_error = (sqrt(d_E_I*d_E_I+d_E_Q*d_E_Q)-sqrt(d_L_I*d_L_I+d_L_Q*d_L_Q))/(sqrt(d_E_I*d_E_I+d_E_Q*d_E_Q)+sqrt(d_L_I*d_L_I+d_L_Q*d_L_Q));

		// Implement code loop filter and generate NCO command
		code_nco = d_old_code_nco + (d_tau2_code/d_tau1_code)*(code_error - d_old_code_error) + code_error * (d_pdi_code/d_tau1_code);
		d_old_code_nco   = code_nco;
		d_old_code_error = code_error;

		d_absolute_code_phase_chips+=d_old_code_error; //accumulate the error to get the pseudorange

		// Modify code freq based on NCO command
		d_code_freq = 1023000 - code_nco;

		// Output the tracking data to navigation and PVT

		// Output channel 1: Prompt correlator output Q
		*out[0]=d_P_Q;
		// Output channel 2: Prompt correlator output I
		*out[1]=d_P_I;
		// Output channel 3: Prompt correlator output
		*out[2]=d_absolute_code_phase_chips;

		//Equivalent to the "absolute spreading code starting position" information of the matlab tracking algorithm

//		std::cout<<"tmp_E= "<<tmp_E<<std::endl;
//		std::cout<<"tmp_P= "<<tmp_P<<std::endl;
//		std::cout<<"tmp_L= "<<tmp_L<<std::endl;
//
//		std::cout<<"trk pllDiscr carr_error= "<<carr_error<<std::endl;
//		std::cout<<"trk dllDiscr code error= "<<code_error<<std::endl;
//		std::cout<<"trk dllDiscrFilt code_nco= "<<code_nco<<std::endl;
//		std::cout<<"trk pllDiscrFilt carr_nco= "<<carr_nco<<std::endl;


		if(d_dump) {
			// MULTIPLEXED FILE RECORDING - Record results to file
			tmp_E=sqrt(d_E_I*d_E_I+d_E_Q*d_E_Q);
			tmp_P=sqrt(d_P_I*d_P_I+d_P_Q*d_P_Q);
			tmp_L=sqrt(d_L_I*d_L_I+d_L_Q*d_L_Q);

			// EPR
			d_dump_file.write((char*)&tmp_E, sizeof(float));
			d_dump_file.write((char*)&tmp_P, sizeof(float));
			d_dump_file.write((char*)&tmp_L, sizeof(float));
			// DLL
			d_dump_file.write((char*)&code_error, sizeof(float));
			d_dump_file.write((char*)&code_nco, sizeof(float));
			//PLL
			d_dump_file.write((char*)&carr_error, sizeof(float));
			d_dump_file.write((char*)&carr_nco, sizeof(float));
			//FREQ AND PHASE
			d_dump_file.write((char*)&d_code_freq, sizeof(float));
			d_dump_file.write((char*)&d_carrier_doppler, sizeof(float));
			// PROMPT I and Q (to analyze navigation symbols)
			d_dump_file.write((char*)&d_P_I, sizeof(float));
			d_dump_file.write((char*)&d_P_Q, sizeof(float));

		}
		// debug: Second counter in channel 0
		if (d_channel==0)
		{
                  if (floor(d_sample_counter/d_fs_in)!=d_last_seg)
                    {
                    d_last_seg=floor(d_sample_counter/d_fs_in);
                    std::cout<<"t="<<d_last_seg<<std::endl;
                    }
		}
		if (d_sample_counter>round((float)this->d_fs_in*39)){ //stop after some seconds debug only!
		d_enable_tracking=false;
		std::cout<<"Stop tracking at sample "<<d_sample_counter<<" and acq at sample "<<d_acq_sample_stamp<<std::endl;
		if(d_queue != gr_msg_queue_sptr()) {
				ControlMessageFactory* cmf = new ControlMessageFactory();
				d_queue->handle(cmf->GetQueueMessage(200,0)); //send stop to the control_thread
				delete cmf;
				std::cout<<"stop sent from tracking" << std::endl;
			}
		}
	}
	//consume_each(1); //not necesary for gr_sync_block derivates
	return 1; //one group of d_vector_lenght samples at time, and output one set of results ALWAYS even in the case of d_enable_tracking==false
}


void gps_l1_ca_dll_pll_tracking_cc::set_acq_code_phase(float code_phase) {
	d_code_phase = code_phase;
	LOG_AT_LEVEL(INFO) << "Tracking code phase set to " << d_code_phase;
}

void gps_l1_ca_dll_pll_tracking_cc::set_acq_doppler(float doppler) {
	d_carrier_doppler = doppler;
	d_carr_freq_basis = doppler;
	LOG_AT_LEVEL(INFO) << "Tracking carrier doppler set to " << d_carrier_doppler;
}

void gps_l1_ca_dll_pll_tracking_cc::set_satellite(unsigned int satellite) {
	d_satellite = satellite;
	LOG_AT_LEVEL(INFO) << "Tracking Satellite set to " << d_satellite;
}

void gps_l1_ca_dll_pll_tracking_cc::set_channel(unsigned int channel) {
	d_channel = channel;
	LOG_AT_LEVEL(INFO) << "Tracking Channel set to " << d_channel;
}
