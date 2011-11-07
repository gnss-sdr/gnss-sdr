/*!
 * \file gps_l1_ca_dll_pll_tracking_cc.cc
 * \brief code DLL + carrier PLL
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in [1]
 * [1] K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency Approach, Birkha user, 2007
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gps_l1_ca_dll_pll_tracking_cc.h"
#include "gps_sdr_signal_processing.h"

#include "gps_sdr_simd.h"
#include "gps_sdr_x86.h"

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

using google::LogMessage;

gps_l1_ca_dll_pll_tracking_cc_sptr
gps_l1_ca_dll_pll_make_tracking_cc(unsigned int satellite, long if_freq, long fs_in, unsigned
			int vector_length, gr_msg_queue_sptr queue, bool dump) {

	return gps_l1_ca_dll_pll_tracking_cc_sptr(new gps_l1_ca_dll_pll_tracking_cc(satellite, if_freq,
			fs_in, vector_length, queue, dump));
}

void gps_l1_ca_dll_pll_tracking_cc::forecast (int noutput_items,
    gr_vector_int &ninput_items_required){
    ninput_items_required[0] =d_vector_length*2; //set the required available samples in each call
}

gps_l1_ca_dll_pll_tracking_cc::gps_l1_ca_dll_pll_tracking_cc(unsigned int satellite, long if_freq, long fs_in, unsigned
		int vector_length, gr_msg_queue_sptr queue, bool dump) :
	    gr_block ("gps_l1_ca_dll_pll_tracking_cc", gr_make_io_signature (1, 1, sizeof(gr_complex)),
	              gr_make_io_signature(5, 5, sizeof(float))) {

		//gr_sync_decimator ("gps_l1_ca_dll_pll_tracking_cc", gr_make_io_signature (1, 1, sizeof(gr_complex)),
		//		gr_make_io_signature(3, 3, sizeof(float)),vector_length) {
	// initialize internal vars
	d_queue = queue;
	d_dump = dump;
	d_satellite = satellite;
	d_if_freq = if_freq;
	d_fs_in = fs_in;
	d_vector_length = vector_length;

	// Initialize tracking variables ==========================================
	/*!
	 * \todo Include PLL and DLL filter setting in configuration file
	 */

	//--- DLL variables --------------------------------------------------------
	d_early_late_spc = 0.5; // Define early-late offset (in chips)
	d_pdi_code = 0.001;// Summation interval for code
	d_dllnoisebandwidth=1; //Hz
	d_dlldampingratio=0.7;
	calculate_lopp_coef(&d_tau1_code, &d_tau2_code, d_dllnoisebandwidth, d_dlldampingratio,1.0);// Calculate filter coefficient values

	//--- PLL variables --------------------------------------------------------
	d_pdi_carr = 0.001;// Summation interval for carrier
	d_plldampingratio=0.7;
	d_pllnoisebandwidth=50;

	//Calculate filter coefficient values
	calculate_lopp_coef(&d_tau1_carr, &d_tau2_carr, d_pllnoisebandwidth, d_plldampingratio,0.25);// Calculate filter coefficient values

	// Initialization of local code replica

	d_code_length=1023;
    // Get space for a vector with the C/A code replica sampled 1x/chip
    d_ca_code=new gr_complex[d_code_length+2];

    // Get space for the resampled early / prompt / late local replicas
    d_early_code= new gr_complex[d_vector_length*2];
    d_prompt_code=new gr_complex[d_vector_length*2];
    d_late_code=new gr_complex[d_vector_length*2];

    // space for carrier wipeoff and signal baseband vectors
    d_carr_sign=new gr_complex[d_vector_length*2];
    d_bb_sign=new gr_complex[d_vector_length*2];

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

    d_absolute_code_phase_samples = 0;

    // sample synchronization
    d_sample_counter=0;
    d_acq_sample_stamp=0;

    d_enable_tracking=false;
    d_pull_in=false;
    d_last_seg=0;

    d_blksize=d_vector_length;
    d_loops_count=0;

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter=0;
    d_P_I_buffer=new float[CN0_ESTIMATION_SAMPLES];
    d_P_Q_buffer=new float[CN0_ESTIMATION_SAMPLES];
    d_carrier_lock_test=1;
    d_SNR_SNV=0;
    d_SNR_SNV_dB_Hz=0;
    d_SNR_MM=0;
    d_carrier_lock_fail_counter=0;
    d_carrier_lock_threshold=5;

}

void gps_l1_ca_dll_pll_tracking_cc::calculate_lopp_coef(float* tau1,float* tau2, float lbw, float zeta, float k){
	// Solve natural frequency
	float Wn;
	Wn = lbw*8*zeta / (4*zeta*zeta + 1);
	// solve for t1 & t2
	*tau1 = k / (Wn * Wn);
	*tau2 = (2.0 * zeta) / Wn;
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

    //compute the code phase chips prediction
    trk_corrected_code_phase=round(fmod((d_code_phase+(float)acq_sample_difference+(T_prn-T_prn_mod)*((float)acq_sample_difference/(float)d_vector_length)*(float)d_fs_in),(float)d_vector_length));

    if (trk_corrected_code_phase<0)
    {
        trk_corrected_code_phase=d_vector_length+trk_corrected_code_phase;
    }
    d_absolute_code_phase_samples=(float)trk_corrected_code_phase;

    // generate local reference ALWAYS starting at chip 1, not corrected
    code_gen_conplex(&d_ca_code[1],d_satellite,0);

    // Then make it possible to do early and late versions
    d_ca_code[0]=d_ca_code[1023];
    d_ca_code[1024]=d_ca_code[1];

    DLOG(INFO) << "Start tracking for satellite "<<this->d_satellite<<" received ";

    if (d_dump==true)
     {
        //std::stringstream d_dump_filename_str;//create a stringstream to form the dump filename
        //d_dump_filename_str<<"./data/trk_epl_CH_"<<this->d_channel<<"_SAT_"<<this->d_satellite<<".dat";
        //d_dump_filename=d_dump_filename_str.str();
    	if (d_dump_file.is_open()==false)
    	{
      	  try {
			d_dump_filename="track_ch"; //base path and name for the tracking log file
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
    d_carrier_lock_fail_counter=0;
    d_pull_in=true;
    d_enable_tracking=true;
    std::cout<<"Tracking start on channel "<<d_channel<<" for satellite ID "<< this->d_satellite+1 << std::endl;
}

void gps_l1_ca_dll_pll_tracking_cc::update_local_code_refs()
{
	float tcode;
	int associated_chip_index;
	// unified loop for E, P, L code vectors
    for (unsigned int i=0;i<d_blksize;i++)
    {
        tcode=i*d_code_phase_step+d_rem_code_phase-d_early_late_spc;
        associated_chip_index=ceil(fmod(tcode,d_code_length));
        d_early_code[i] = d_ca_code[associated_chip_index];
        tcode += d_early_late_spc;
        associated_chip_index = ceil(fmod(tcode, d_code_length));
        d_prompt_code[i] = d_ca_code[associated_chip_index];
        tcode += d_early_late_spc;
        associated_chip_index = ceil(fmod(tcode, d_code_length));
        d_late_code[i] = d_ca_code[associated_chip_index];
    }

    //**** Option 1: Keep the number of samples per PRN period constant and equal to the nominal value
    //****           and record the size mismatch in a var: d_rem_code_phase
    //max_tcode=((float)d_vector_length-1.0)*d_code_phase_step+d_rem_code_phase;
    //d_rem_code_phase = (max_tcode + d_code_phase_step) - 1023.0;
    //d_rem_code_phase = d_rem_code_phase+((float)d_vector_length-1023.0*(1.0/d_code_freq)*(float)d_fs_in)*d_code_phase_step;

    //**** Option 2: Each loop, compute the new PRN sequence code length according to the estimated Doppler
    tcode=d_blksize*d_code_phase_step+d_rem_code_phase;
    d_rem_code_phase = tcode - 1023.0; //prompt remaining code phase
}

void gps_l1_ca_dll_pll_tracking_cc::update_local_carrier()
{
        float phase, phase_step;

        phase_step = (float)TWO_PI*d_carrier_doppler/d_fs_in;
        phase=d_rem_carr_phase;
        for(unsigned int i = 0; i < d_blksize; i++) {
            d_carr_sign[i] = std::complex<float>(cos(phase),sin(phase));
            phase += phase_step;
        }
        d_rem_carr_phase=fmod(phase,TWO_PI);
}

gps_l1_ca_dll_pll_tracking_cc::~gps_l1_ca_dll_pll_tracking_cc() {
    /*!
     * \todo free memory!!
     */
	d_dump_file.close();
}

/*! Tracking signal processing
 * Notice that this is a class derived from gr_sync_decimator, so each of the ninput_items has vector_length samples
 */

int gps_l1_ca_dll_pll_tracking_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items) {
    d_loops_count++;
	if (d_enable_tracking==true){

	    if (d_pull_in==true)
	    {
	        int samples_offset=ceil(d_absolute_code_phase_samples);
	        d_code_phase_ms=(d_absolute_code_phase_samples*1000.0)/(float)d_fs_in;
	        consume_each(samples_offset); //shift input to perform alignement with local replica
	        d_sample_counter+=samples_offset; //count for the processed samples
	        d_pull_in=false;
	        return 1;
	    }
		float carr_error;
		float carr_nco;
		float code_error;
		float code_nco;
		float tmp_E,tmp_P,tmp_L;

		const gr_complex* in = (gr_complex*) input_items[0]; //PRN start block alignement

		float **out = (float **) &output_items[0];

		// Update the phasestep based on code freq (variable) and
		// sampling frequency (fixed)
		// code_phase_step_per_sample = T_sample/T_chip
		d_code_phase_step = d_code_freq / (float)d_fs_in; //[chips]
		// variable code PRN sample block size
	    d_blksize=ceil((1023.0-d_rem_code_phase) / d_code_phase_step); //[samples]


		//float rem_code_phase_samples=d_rem_code_phase/d_code_phase_step;
		//d_absolute_code_phase_samples-=floor(rem_code_phase_samples);

		//d_rem_code_phase=d_code_phase_step*(rem_code_phase_samples-floor(rem_code_phase_samples));

		this->update_local_code_refs();
		this->update_local_carrier();

		d_E_I=0.0;
		d_P_I=0.0;
		d_L_I=0.0;

		d_E_Q=0.0;
		d_P_Q=0.0;
		d_L_Q=0.0;

		// perform Early, Prompt and Late correlation
		for(unsigned int i=0;i<d_blksize;i++) {
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

		//float block_error_samples;
		//block_error_samples=(float)d_vector_length-d_blksize;
		//d_absolute_code_phase_samples=d_absolute_code_phase_samples+block_error_samples;
		// Find PLL error and update carrier NCO -
		// Implement carrier loop discriminator (phase detector)
		//carr_error = atan(d_P.imag() / d_P.real()) / TWO_PI;

		carr_error = atan(d_P_Q / d_P_I) / (float)TWO_PI;

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
		d_old_code_error = code_error; //[chips]

		// Modify code freq based on NCO command
		d_code_freq = 1023000 - code_nco;


		d_code_phase_ms+=(1023000/d_code_freq)-1.0;

		/*!
		 * \todo Code lock detector
		 */

		// ####### CN0 ESTIMATION AND LOCK DETECTORS ######
		if (d_cn0_estimation_counter<CN0_ESTIMATION_SAMPLES)
		{
			// fill buffer with prompt correlator output values
			d_P_I_buffer[d_cn0_estimation_counter]=d_P_I;
			d_P_Q_buffer[d_cn0_estimation_counter]=d_P_Q;
			d_cn0_estimation_counter++;
		}else{
			// estimate CN0 and lock status using buffered values
			   // MATLAB CODE
			   //Psig=((1/N)*sum(abs(imag(x((n-N+1):n)))))^2;
			   //Ptot=(1/N)*sum(abs(x((n-N+1):n)).^2);
			   //M2=Ptot;
			   //M4=(1/N)*sum(abs(x((n-N+1):n)).^4);
			   //SNR_SNV(count)=Psig/(Ptot-Psig);
			   //SNR_MM(count)=sqrt(2*M2^2-M4)/(M2-Psig);
			   // lock detector operation
			   //NBD=sum(abs(imag(x((n-N+1):n))))^2 + sum(abs(real(x((n-N+1):n))))^2;
			   //NBP=sum(imag(x((n-N+1):n)).^2) - sum(real(x((n-N+1):n)).^2);
			   //LOCK(count)=NBD/NBP;
			   //CN0_SNV_dB=10*log10(SNR_SNV)+10*log10(BW)-10*log10(PRN_length);
			float tmp_abs_I,tmp_abs_Q;
			float tmp_sum_abs_I,tmp_sum_abs_Q;
			float tmp_sum_sqr_I,tmp_sum_sqr_Q;
			float Psig,Ptot,M2,M4;
			float NBD,NBP;
			Psig=0;
			Ptot=0;
			NBD=0;
			NBP=0;
			tmp_sum_abs_I=0;
			tmp_sum_abs_Q=0;
			tmp_sum_sqr_I=0;
			tmp_sum_sqr_Q=0;
			for (int i=0;i<CN0_ESTIMATION_SAMPLES;i++)
			{
				tmp_abs_I=std::abs(d_P_I_buffer[i]);
				tmp_abs_Q=std::abs(d_P_Q_buffer[i]);
				Psig+=tmp_abs_I;
				Ptot+=d_P_I_buffer[i]*d_P_I_buffer[i]+d_P_Q_buffer[i]*d_P_Q_buffer[i];
				tmp_sum_abs_I+=tmp_abs_I;
				tmp_sum_abs_Q+=tmp_abs_Q;
				tmp_sum_sqr_I+=(d_P_I_buffer[i]*d_P_I_buffer[i]);
				tmp_sum_sqr_Q+=(d_P_Q_buffer[i]*d_P_Q_buffer[i]);
			}
			Psig=Psig/(float)CN0_ESTIMATION_SAMPLES;
			Psig=Psig*Psig;
			d_SNR_SNV=Psig/(Ptot/(float)CN0_ESTIMATION_SAMPLES-Psig);
			d_SNR_SNV_dB_Hz=10*log10(d_SNR_SNV)+10*log10(d_fs_in/2)-10*log10(d_code_length);
			NBD=tmp_sum_abs_I*tmp_sum_abs_I+tmp_sum_abs_Q*tmp_sum_abs_Q;
			NBP=tmp_sum_sqr_I-tmp_sum_sqr_Q;
			d_carrier_lock_test=NBD/NBP;
			d_cn0_estimation_counter=0;

		}
		// ###### TRACKING UNLOCK NOTIFICATION #####
		int tracking_message;
        if (d_carrier_lock_test<d_carrier_lock_threshold or d_carrier_lock_test>30)
        {
             d_carrier_lock_fail_counter++;
        }else{
        	if (d_carrier_lock_fail_counter>0) d_carrier_lock_fail_counter--;
        }
        if (d_carrier_lock_fail_counter>200)
        {
        	std::cout<<"Channel "<<d_channel << " loss of lock!\r\n";
        	tracking_message=3; //loss of lock
        	d_channel_internal_queue->push(tracking_message);
        	d_carrier_lock_fail_counter=0;
        	d_enable_tracking=false; // TODO: check if disabling tracking is consistent with the channel state machine

        }
        //std::cout<<"d_carrier_lock_fail_counter"<<d_carrier_lock_fail_counter<<"\r\n";

		// Output the tracking data to navigation and PVT
		// Output channel 1: Prompt correlator output Q
		*out[0]=d_P_Q;
		// Output channel 2: Prompt correlator output I
		*out[1]=d_P_I;
		// Output channel 3: PRN absolute delay [ms]
		*out[2]=(float)(((double)d_sample_counter/(double)d_fs_in)*1000.0);

		// Output channel 4: PRN code error [ms]
		*out[3]=d_code_phase_ms;//(code_error*1000.0)/d_code_freq;

		if(d_dump) {
			// MULTIPLEXED FILE RECORDING - Record results to file
			tmp_E=sqrt(d_E_I*d_E_I+d_E_Q*d_E_Q);
			tmp_P=sqrt(d_P_I*d_P_I+d_P_Q*d_P_Q);
			tmp_L=sqrt(d_L_I*d_L_I+d_L_Q*d_L_Q);

	      	try {
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

				// Absolute PRN start sample (MATLAB version)
				//d_dump_file.write((char*)&d_sample_counter, sizeof(unsigned long int));
				//d_dump_file.write((char*)&d_loops_count, sizeof(unsigned long int));
				d_dump_file.write((char*)&d_SNR_SNV_dB_Hz, sizeof(float));
				d_dump_file.write((char*)&d_carrier_lock_test, sizeof(float));
	      	 }
			  catch (std::ifstream::failure e) {
				std::cout << "Exception writing trk dump file "<<e.what()<<"\r\n";
			  }
		}
		// debug: Second counter in channel 0
		if (d_channel==0)
		{
                  if (floor(d_sample_counter/d_fs_in)!=d_last_seg)
                    {
                    d_last_seg=floor(d_sample_counter/d_fs_in);
                    std::cout<<"t="<<d_last_seg<<std::endl;
                    std::cout<<"TRK CH "<<d_channel<<" CN0="<<d_SNR_SNV_dB_Hz<< std::endl;
                    std::cout<<"TRK CH "<<d_channel<<" Carrier_lock_test="<<d_carrier_lock_test<< std::endl;
                    }

			if (d_sample_counter>round((float)this->d_fs_in*70)){ //stop after some seconds debug only!
			d_enable_tracking=false;
			std::cout<<"Stop tracking at sample "<<d_sample_counter<<" and acq at sample "<<d_acq_sample_stamp<<std::endl;
			if(d_queue != gr_msg_queue_sptr()) {
					ControlMessageFactory* cmf = new ControlMessageFactory();
					d_queue->handle(cmf->GetQueueMessage(200,0)); //send stop to the control_thread
					delete cmf;
					std::cout<<"stop sent from tracking";
				}
			}
		}else
		{
			if (floor(d_sample_counter/d_fs_in)!=d_last_seg)
              {
              d_last_seg=floor(d_sample_counter/d_fs_in);
              std::cout<<"TRK CH "<<d_channel<<" CN0="<<d_SNR_SNV_dB_Hz<< std::endl;
              std::cout<<"TRK CH "<<d_channel<<" Carrier_lock_test="<<d_carrier_lock_test<< std::endl;
              }
		}
	}
	consume_each(d_blksize); // this is necesary in gr_block derivates
        d_sample_counter+=d_blksize; //count for the processed samples
	return 1; //output tracking result ALWAYS even in the case of d_enable_tracking==false
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
