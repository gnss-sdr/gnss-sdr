/*!
 * \file galileo_e5a_dll_fll_pll_tracking_cc.h
 * \brief Implementation of a code DLL + carrier PLL
 *  tracking block for Galileo E5a signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 * 		<ul>
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

#include "galileo_e5a_dll_pll_tracking_cc.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <gnuradio/io_signature.h>
#include <gnuradio/fxpt.h>  // fixed point sine and cosine
#include <glog/logging.h>
#include "gnss_synchro.h"
#include "galileo_e5_signal_processing.h"
#include "tracking_discriminators.h"
#include "lock_detectors.h"
#include "Galileo_E5a.h"
#include "Galileo_E1.h"
#include "control_message_factory.h"


/*!
 * \todo Include in definition header file
 */
#define CN0_ESTIMATION_SAMPLES 20
#define MINIMUM_VALID_CN0 25
#define MAXIMUM_LOCK_FAIL_COUNTER 50
#define CARRIER_LOCK_THRESHOLD 0.85


using google::LogMessage;

galileo_e5a_dll_pll_tracking_cc_sptr
galileo_e5a_dll_pll_make_tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float pll_bw_init_hz,
        float dll_bw_init_hz,
        int ti_ms,
        float early_late_space_chips)
{
    return galileo_e5a_dll_pll_tracking_cc_sptr(new Galileo_E5a_Dll_Pll_Tracking_cc(if_freq,
            fs_in, vector_length, queue, dump, dump_filename, pll_bw_hz, dll_bw_hz, pll_bw_init_hz, dll_bw_init_hz, ti_ms, early_late_space_chips));
}



void Galileo_E5a_Dll_Pll_Tracking_cc::forecast (int noutput_items,
        gr_vector_int &ninput_items_required)
{
    ninput_items_required[0] = static_cast<int>(d_vector_length)*2; //set the required available samples in each call
}

Galileo_E5a_Dll_Pll_Tracking_cc::Galileo_E5a_Dll_Pll_Tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        boost::shared_ptr<gr::msg_queue> queue,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float pll_bw_init_hz,
        float dll_bw_init_hz,
        int ti_ms,
        float early_late_space_chips) :
        gr::block("Galileo_E5a_Dll_Pll_Tracking_cc", gr::io_signature::make(1, 1, sizeof(gr_complex)),
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
    d_code_loop_filter = Tracking_2nd_DLL_filter(GALILEO_E5a_CODE_PERIOD);
    d_carrier_loop_filter = Tracking_2nd_PLL_filter(GALILEO_E5a_CODE_PERIOD);
    d_current_ti_ms = 1; // initializes with 1ms of integration time until secondary code lock
    d_ti_ms = ti_ms;
    d_dll_bw_hz = dll_bw_hz;
    d_pll_bw_hz = pll_bw_hz;
    d_dll_bw_init_hz = dll_bw_init_hz;
    d_pll_bw_init_hz = pll_bw_init_hz;

    // Initialize tracking  ==========================================
    d_code_loop_filter.set_DLL_BW(d_dll_bw_init_hz);
    d_carrier_loop_filter.set_PLL_BW(d_pll_bw_init_hz);

    //--- DLL variables --------------------------------------------------------
    d_early_late_spc_chips = early_late_space_chips; // Define early-late offset (in chips)

    // Initialization of local code replica
    // Get space for a vector with the E5a primary code replicas sampled 1x/chip
    d_codeQ = new gr_complex[static_cast<int>(Galileo_E5a_CODE_LENGTH_CHIPS) + 2];
    d_codeI = new gr_complex[static_cast<int>(Galileo_E5a_CODE_LENGTH_CHIPS) + 2];

    d_early_code  = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_late_code   = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_prompt_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_prompt_data_code = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));
    d_carr_sign = static_cast<gr_complex*>(volk_malloc(2 * d_vector_length * sizeof(gr_complex), volk_get_alignment()));

    // correlator outputs (complex number)
    d_Early  = gr_complex(0, 0);
    d_Prompt = gr_complex(0, 0);
    d_Late   = gr_complex(0, 0);
    d_Prompt_data = gr_complex(0, 0);

    //--- Perform initializations ------------------------------
    // define initial code frequency basis of NCO
    d_code_freq_chips = Galileo_E5a_CODE_CHIP_RATE_HZ;
    // define residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    // define residual carrier phase
    d_rem_carr_phase_rad = 0.0;
    //Filter error vars
    d_code_error_filt_secs = 0.0;
    // sample synchronization
    d_sample_counter = 0;
    d_acq_sample_stamp = 0;
    d_last_seg = 0;
    d_first_transition = false;

    d_secondary_lock = false;
    d_secondary_delay = 0;
    d_integration_counter = 0;

    d_current_prn_length_samples = static_cast<int>(d_vector_length);

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[CN0_ESTIMATION_SAMPLES];
    d_carrier_lock_test = 1;
    d_CN0_SNV_dB_Hz = 0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_threshold = CARRIER_LOCK_THRESHOLD;

    d_channel_internal_queue = 0;
    d_acquisition_gnss_synchro = 0;
    d_channel = 0;
    tmp_E = 0;
    tmp_P = 0;
    tmp_L = 0;
    d_acq_code_phase_samples = 0;
    d_acq_carrier_doppler_hz = 0;
    d_carrier_doppler_hz = 0;
    d_acc_carrier_phase_rad = 0;
    d_code_phase_samples = 0;
    d_acc_code_phase_secs = 0;
    d_state = 0;

    systemName["E"] = std::string("Galileo");
}


Galileo_E5a_Dll_Pll_Tracking_cc::~Galileo_E5a_Dll_Pll_Tracking_cc ()
{
    d_dump_file.close();

    volk_free(d_prompt_code);
    volk_free(d_late_code);
    volk_free(d_early_code);
    volk_free(d_carr_sign);
    volk_free(d_prompt_data_code);
    delete[] d_codeI;
    delete[] d_codeQ;
    delete[] d_Prompt_buffer;
}

void Galileo_E5a_Dll_Pll_Tracking_cc::start_tracking()
{
    /*
     *  correct the code phase according to the delay between acq and trk
     */
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp =  d_acquisition_gnss_synchro->Acq_samplestamp_samples;

    long int acq_trk_diff_samples;
    float acq_trk_diff_seconds;
    acq_trk_diff_samples = static_cast<long int>(d_sample_counter) - static_cast<long int>(d_acq_sample_stamp);//-d_vector_length;
    LOG(INFO) << "Number of samples between Acquisition and Tracking =" << acq_trk_diff_samples;
    acq_trk_diff_seconds = static_cast<float>(acq_trk_diff_samples) / static_cast<float>(d_fs_in);
    //doppler effect
    // Fd=(C/(C+Vr))*F
    float radial_velocity;
    radial_velocity = (Galileo_E5a_FREQ_HZ + d_acq_carrier_doppler_hz)/Galileo_E5a_FREQ_HZ;
    // new chip and prn sequence periods based on acq Doppler
    float T_chip_mod_seconds;
    float T_prn_mod_seconds;
    float T_prn_mod_samples;
    d_code_freq_chips = radial_velocity * Galileo_E5a_CODE_CHIP_RATE_HZ;
    T_chip_mod_seconds = 1/d_code_freq_chips;
    T_prn_mod_seconds = T_chip_mod_seconds * Galileo_E5a_CODE_LENGTH_CHIPS;
    T_prn_mod_samples = T_prn_mod_seconds * static_cast<float>(d_fs_in);

    d_current_prn_length_samples = round(T_prn_mod_samples);

    float T_prn_true_seconds = Galileo_E5a_CODE_LENGTH_CHIPS / Galileo_E5a_CODE_CHIP_RATE_HZ;
    float T_prn_true_samples = T_prn_true_seconds * static_cast<float>(d_fs_in);
    float T_prn_diff_seconds;
    T_prn_diff_seconds = T_prn_true_seconds - T_prn_mod_seconds;
    float N_prn_diff;
    N_prn_diff = acq_trk_diff_seconds / T_prn_true_seconds;
    float corrected_acq_phase_samples, delay_correction_samples;
    corrected_acq_phase_samples = fmod((d_acq_code_phase_samples + T_prn_diff_seconds * N_prn_diff * static_cast<float>(d_fs_in)), T_prn_true_samples);
    if (corrected_acq_phase_samples < 0)
        {
            corrected_acq_phase_samples = T_prn_mod_samples + corrected_acq_phase_samples;
        }
    delay_correction_samples = d_acq_code_phase_samples - corrected_acq_phase_samples;

    d_acq_code_phase_samples = corrected_acq_phase_samples;

    d_carrier_doppler_hz = d_acq_carrier_doppler_hz;

    // DLL/PLL filter initialization
    d_carrier_loop_filter.initialize(); // initialize the carrier filter
    d_code_loop_filter.initialize();    // initialize the code filter

    // generate local reference ALWAYS starting at chip 1 (1 sample per chip)
    char sig[3];
    strcpy(sig,"5Q");
    galileo_e5_a_code_gen_complex_primary(&d_codeQ[1], d_acquisition_gnss_synchro->PRN, sig);
    d_codeQ[0] = d_codeQ[static_cast<int>(Galileo_E5a_CODE_LENGTH_CHIPS)];
    d_codeQ[static_cast<int>(Galileo_E5a_CODE_LENGTH_CHIPS) + 1] = d_codeQ[1];

    strcpy(sig,"5I");
    galileo_e5_a_code_gen_complex_primary(&d_codeI[1], d_acquisition_gnss_synchro->PRN, sig);
    d_codeI[0] = d_codeI[static_cast<int>(Galileo_E5a_CODE_LENGTH_CHIPS)];
    d_codeI[static_cast<int>(Galileo_E5a_CODE_LENGTH_CHIPS) + 1] = d_codeI[1];

    d_carrier_lock_fail_counter = 0;
    d_rem_code_phase_samples = 0;
    d_rem_carr_phase_rad = 0;
    d_acc_carrier_phase_rad = 0;
    d_acc_code_phase_secs = 0;

    d_code_phase_samples = d_acq_code_phase_samples;

    std::string sys_ = &d_acquisition_gnss_synchro->System;
    sys = sys_.substr(0,1);

    // DEBUG OUTPUT
    std::cout << "Tracking start on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
    LOG(INFO) << "Starting tracking of satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;


    // enable tracking
    d_state = 1;

    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz
            << " Code Phase correction [samples]=" << delay_correction_samples
            << " PULL-IN Code Phase [samples]=" << d_acq_code_phase_samples;
}

void Galileo_E5a_Dll_Pll_Tracking_cc::acquire_secondary()
{
    // 1. Transform replica to 1 and -1
    int sec_code_signed[Galileo_E5a_Q_SECONDARY_CODE_LENGTH];
    for (unsigned int i = 0; i < Galileo_E5a_Q_SECONDARY_CODE_LENGTH; i++)
	{
	    if (Galileo_E5a_Q_SECONDARY_CODE[d_acquisition_gnss_synchro->PRN-1].at(i) == '0')
		{
		    sec_code_signed[i] = 1;
		}
	    else
		{
		    sec_code_signed[i] = -1;
		}
	}
    // 2. Transform buffer to 1 and -1
    int in_corr[CN0_ESTIMATION_SAMPLES];
    for (unsigned int i = 0; i < CN0_ESTIMATION_SAMPLES; i++)
	{
	    if (d_Prompt_buffer[i].real() >0)
		{
		    in_corr[i] = 1;
		}
	    else
		{
		    in_corr[i] = -1;
		}
	}
    // 3. Serial search
    int out_corr;
    int current_best_ = 0;
    for (unsigned int i = 0; i < Galileo_E5a_Q_SECONDARY_CODE_LENGTH; i++)
	{
	    out_corr = 0;
	    for (unsigned int j = 0; j < CN0_ESTIMATION_SAMPLES; j++)
		{
		    //reverse replica sign since i*i=-1 (conjugated complex)
		    out_corr += in_corr[j] * -sec_code_signed[(j+i) % Galileo_E5a_Q_SECONDARY_CODE_LENGTH];
		}
	    if (abs(out_corr) > current_best_)
		{
		    current_best_ = abs(out_corr);
		    d_secondary_delay = i;
		}
	}
    if (current_best_ == CN0_ESTIMATION_SAMPLES) // all bits correlate
	{
	    d_secondary_lock = true;
	    d_secondary_delay = (d_secondary_delay + CN0_ESTIMATION_SAMPLES - 1) % Galileo_E5a_Q_SECONDARY_CODE_LENGTH;
	}
}

void Galileo_E5a_Dll_Pll_Tracking_cc::update_local_code()
{
    double tcode_chips;
    double rem_code_phase_chips;
    int associated_chip_index;
    int associated_chip_index_data;
    int code_length_chips = static_cast<int>(Galileo_E5a_CODE_LENGTH_CHIPS);
    double code_phase_step_chips;
    int early_late_spc_samples;
    int epl_loop_length_samples;

    // unified loop for E, P, L code vectors
    code_phase_step_chips = static_cast<double>(d_code_freq_chips) / static_cast<double>(d_fs_in);
    rem_code_phase_chips = d_rem_code_phase_samples * (d_code_freq_chips / d_fs_in);
    tcode_chips = -rem_code_phase_chips;

    // Alternative EPL code generation (40% of speed improvement!)
    early_late_spc_samples = round(d_early_late_spc_chips / code_phase_step_chips);
    epl_loop_length_samples = d_current_prn_length_samples + early_late_spc_samples * 2;

    for (int i = 0; i < epl_loop_length_samples; i++)
        {
            associated_chip_index = 1 + round(fmod(tcode_chips - d_early_late_spc_chips, code_length_chips));
            associated_chip_index_data = 1 + round(fmod(tcode_chips, code_length_chips));
            d_early_code[i] = d_codeQ[associated_chip_index];
            d_prompt_data_code[i] = d_codeI[associated_chip_index_data];
            tcode_chips = tcode_chips + code_phase_step_chips;
        }
    memcpy(d_prompt_code, &d_early_code[early_late_spc_samples], d_current_prn_length_samples * sizeof(gr_complex));
    memcpy(d_late_code, &d_early_code[early_late_spc_samples * 2], d_current_prn_length_samples * sizeof(gr_complex));

}


void Galileo_E5a_Dll_Pll_Tracking_cc::update_local_carrier()
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


int Galileo_E5a_Dll_Pll_Tracking_cc::general_work (int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    // process vars
    float carr_error_hz;
    float carr_error_filt_hz;
    float code_error_chips;
    float code_error_filt_chips;
    // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
    Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0]; //block output streams pointer

    // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
    Gnss_Synchro current_synchro_data;
    // Fill the acquisition data
    current_synchro_data = *d_acquisition_gnss_synchro;

    /* States: 	0 Tracking not enabled
     * 		1 Pull-in of primary code (alignment).
     * 		3 Tracking algorithm. Correlates EPL each loop and accumulates the result
     * 					until it reaches integration time.
     */
    switch (d_state)
    {
	case 0:
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
		d_Early = gr_complex(0,0);
		d_Prompt = gr_complex(0,0);
		d_Late = gr_complex(0,0);
		d_Prompt_data = gr_complex(0,0);
                d_acquisition_gnss_synchro->Flag_valid_pseudorange = false;

		*out[0] = *d_acquisition_gnss_synchro;

		break;
	    }
	case 1:
	    {
		int samples_offset;
		float acq_trk_shif_correction_samples;
		int acq_to_trk_delay_samples;
		acq_to_trk_delay_samples = d_sample_counter - d_acq_sample_stamp;
		acq_trk_shif_correction_samples = d_current_prn_length_samples - fmod(static_cast<float>(acq_to_trk_delay_samples),  static_cast<float>(d_current_prn_length_samples));
		samples_offset = round(d_acq_code_phase_samples + acq_trk_shif_correction_samples);
		d_sample_counter = d_sample_counter + samples_offset; //count for the processed samples
		DLOG(INFO) << " samples_offset=" << samples_offset;
		d_state = 2; // start in Ti = 1 code, until secondary code lock.

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
		consume_each(samples_offset); //shift input to perform alignment with local replica
		return 1;
		break;
	    }
	case 2:
	    {
		// Block input data and block output stream pointers
		const gr_complex* in = (gr_complex*) input_items[0]; //PRN start block alignment
		gr_complex sec_sign_Q;
		gr_complex sec_sign_I;
		// Secondary code Chip
		if (d_secondary_lock)
		    {
//			sec_sign_Q = gr_complex((Galileo_E5a_Q_SECONDARY_CODE[d_acquisition_gnss_synchro->PRN-1].at(d_secondary_delay)=='0' ? 1 : -1),0);
//			sec_sign_I = gr_complex((Galileo_E5a_I_SECONDARY_CODE.at(d_secondary_delay%Galileo_E5a_I_SECONDARY_CODE_LENGTH)=='0' ? 1 : -1),0);
			sec_sign_Q = gr_complex((Galileo_E5a_Q_SECONDARY_CODE[d_acquisition_gnss_synchro->PRN-1].at(d_secondary_delay) == '0' ? -1 : 1), 0);
			sec_sign_I = gr_complex((Galileo_E5a_I_SECONDARY_CODE.at(d_secondary_delay % Galileo_E5a_I_SECONDARY_CODE_LENGTH) == '0' ? -1 : 1), 0);
		    }
		else
		    {
			sec_sign_Q = gr_complex(1.0, 0.0);
			sec_sign_I = gr_complex(1.0, 0.0);
		    }
		// Reset integration counter
		if (d_integration_counter == d_current_ti_ms)
		    {
			d_integration_counter = 0;
		    }
		//Generate local code and carrier replicas (using \hat{f}_d(k-1))
		if (d_integration_counter == 0)
		    {
			update_local_code();
			update_local_carrier();
			// Reset accumulated values
			d_Early = gr_complex(0,0);
			d_Prompt = gr_complex(0,0);
			d_Late = gr_complex(0,0);
		    }
		gr_complex single_early;
		gr_complex single_prompt;
		gr_complex single_late;

		// perform carrier wipe-off and compute Early, Prompt and Late
		// correlation of 1 primary code
		d_correlator.Carrier_wipeoff_and_EPL_volk_IQ(d_current_prn_length_samples,
		                                             in,
		                                             d_carr_sign,
		                                             d_early_code,
		                                             d_prompt_code,
		                                             d_late_code,
		                                             d_prompt_data_code,
		                                             &single_early,
		                                             &single_prompt,
		                                             &single_late,
		                                             &d_Prompt_data);

		// Accumulate results (coherent integration since there are no bit transitions in pilot signal)
		d_Early += single_early * sec_sign_Q;
		d_Prompt += single_prompt * sec_sign_Q;
		d_Late += single_late * sec_sign_Q;
		d_Prompt_data *= sec_sign_I;
		d_integration_counter++;

		// check for samples consistency (this should be done before in the receiver / here only if the source is a file)
		if (std::isnan((d_Prompt).real()) == true or std::isnan((d_Prompt).imag()) == true ) // or std::isinf(in[i].real())==true or std::isinf(in[i].imag())==true)
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

			*out[0] = current_synchro_data;

			return 1;
		    }
		// ################## PLL ##########################################################
		// PLL discriminator
		if (d_integration_counter == d_current_ti_ms)
		    {
			if (d_secondary_lock == true)
			    {
				carr_error_hz = pll_four_quadrant_atan(d_Prompt) / static_cast<float>(GALILEO_PI) * 2;
			    }
			else
			    {
				carr_error_hz = pll_cloop_two_quadrant_atan(d_Prompt) / static_cast<float>(GALILEO_PI) * 2;
			    }

			// Carrier discriminator filter
			carr_error_filt_hz = d_carrier_loop_filter.get_carrier_nco(carr_error_hz);
			// New carrier Doppler frequency estimation
			d_carrier_doppler_hz = d_acq_carrier_doppler_hz + carr_error_filt_hz;
			// New code Doppler frequency estimation
			d_code_freq_chips = Galileo_E5a_CODE_CHIP_RATE_HZ + ((d_carrier_doppler_hz * Galileo_E5a_CODE_CHIP_RATE_HZ) / Galileo_E5a_FREQ_HZ);
		    }
		//carrier phase accumulator for (K) doppler estimation
		d_acc_carrier_phase_rad = d_acc_carrier_phase_rad + 2*GALILEO_PI * d_carrier_doppler_hz * GALILEO_E5a_CODE_PERIOD;
		//remanent carrier phase to prevent overflow in the code NCO
		d_rem_carr_phase_rad = d_rem_carr_phase_rad + 2*GALILEO_PI * d_carrier_doppler_hz * GALILEO_E5a_CODE_PERIOD;
		d_rem_carr_phase_rad = fmod(d_rem_carr_phase_rad, 2*GALILEO_PI);

		// ################## DLL ##########################################################
		if (d_integration_counter == d_current_ti_ms)
		    {
			// DLL discriminator
			code_error_chips = dll_nc_e_minus_l_normalized(d_Early, d_Late); //[chips/Ti]
			// Code discriminator filter
			code_error_filt_chips = d_code_loop_filter.get_code_nco(code_error_chips); //[chips/second]
			//Code phase accumulator
			d_code_error_filt_secs = (GALILEO_E5a_CODE_PERIOD * code_error_filt_chips) / Galileo_E5a_CODE_CHIP_RATE_HZ; //[seconds]
		    }
		d_acc_code_phase_secs = d_acc_code_phase_secs + d_code_error_filt_secs;

		// ################## CARRIER AND CODE NCO BUFFER ALIGNMENT #######################
		// keep alignment parameters for the next input buffer
		double T_chip_seconds;
		double T_prn_seconds;
		double T_prn_samples;
		double K_blk_samples;
		// Compute the next buffer length based in the new period of the PRN sequence and the code phase error estimation
		T_chip_seconds = 1 / static_cast<double>(d_code_freq_chips);
		T_prn_seconds = T_chip_seconds * Galileo_E5a_CODE_LENGTH_CHIPS;
		T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
		K_blk_samples = T_prn_samples + d_rem_code_phase_samples + d_code_error_filt_secs * static_cast<double>(d_fs_in);
		d_current_prn_length_samples = round(K_blk_samples); //round to a discrete samples
		d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample

		// ####### CN0 ESTIMATION AND LOCK DETECTORS ######
		if (d_cn0_estimation_counter < CN0_ESTIMATION_SAMPLES-1)
		    {
			// fill buffer with prompt correlator output values
			d_Prompt_buffer[d_cn0_estimation_counter] = d_Prompt;
			d_cn0_estimation_counter++;
		    }
		else
		    {
			d_Prompt_buffer[d_cn0_estimation_counter] = d_Prompt;
			// ATTEMPT SECONDARY CODE ACQUISITION
			if (d_secondary_lock == false)
			    {
				acquire_secondary(); // changes d_secondary_lock and d_secondary_delay
				if (d_secondary_lock == true)
				    {
					std::cout << "Secondary code locked." << std::endl;
					d_current_ti_ms = d_ti_ms;
					// Change loop parameters ==========================================
					d_code_loop_filter.set_pdi(d_current_ti_ms * GALILEO_E5a_CODE_PERIOD);
					d_carrier_loop_filter.set_pdi(d_current_ti_ms * GALILEO_E5a_CODE_PERIOD);
					d_code_loop_filter.set_DLL_BW(d_dll_bw_hz);
					d_carrier_loop_filter.set_PLL_BW(d_pll_bw_hz);
				    }
				else
				    {
					std::cout << "Secondary code delay couldn't be resolved." << std::endl;
					d_carrier_lock_fail_counter++;
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
						d_state = 0; // TODO: check if disabling tracking is consistent with the channel state machine
					    }
				    }
			    }
			else // Secondary lock achieved, monitor carrier lock.
			    {
				// Code lock indicator
				d_CN0_SNV_dB_Hz = cn0_svn_estimator(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES, d_fs_in,d_current_ti_ms * Galileo_E5a_CODE_LENGTH_CHIPS);
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
						d_state = 0;
					    }
				    }
			    }
			d_cn0_estimation_counter = 0;
		    }
		if (d_secondary_lock && (d_secondary_delay % Galileo_E5a_I_SECONDARY_CODE_LENGTH) == 0)
		    {
			d_first_transition = true;
		    }
		// ########### Output the tracking data to navigation and PVT ##########
		// The first Prompt output not equal to 0 is synchronized with the transition of a navigation data bit.
		if (d_secondary_lock && d_first_transition)
		    {
			current_synchro_data.Prompt_I = static_cast<double>((d_Prompt_data).real());
			current_synchro_data.Prompt_Q = static_cast<double>((d_Prompt_data).imag());
			// Tracking_timestamp_secs is aligned with the PRN start sample
			current_synchro_data.Tracking_timestamp_secs = (static_cast<double>(d_sample_counter) + static_cast<double>(d_current_prn_length_samples) + static_cast<double>(d_rem_code_phase_samples)) / static_cast<double>(d_fs_in);
			// This tracking block aligns the Tracking_timestamp_secs with the start sample of the PRN, thus, Code_phase_secs=0
			current_synchro_data.Code_phase_secs = 0;
			current_synchro_data.Carrier_phase_rads = static_cast<double>(d_acc_carrier_phase_rad);
			current_synchro_data.Carrier_Doppler_hz = static_cast<double>(d_carrier_doppler_hz);
			current_synchro_data.CN0_dB_hz = static_cast<double>(d_CN0_SNV_dB_Hz);
            current_synchro_data.Flag_valid_tracking = false;


            // ########## DEBUG OUTPUT
			   /*!
				*  \todo The stop timer has to be moved to the signal source!
				*/
			   // debug: Second counter in channel 0
			   if (d_channel == 0)
				   {
					   if (floor(d_sample_counter / d_fs_in) != d_last_seg)
						   {
							   d_last_seg = floor(d_sample_counter / d_fs_in);
							   std::cout << "Current input signal time = " << d_last_seg << " [s]" << std::endl;
							   std::cout  << "Galileo E5 Tracking CH " << d_channel <<  ": Satellite "
									<< Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << ", CN0 = " << d_CN0_SNV_dB_Hz << " [dB-Hz] "<<"Doppler="<<d_carrier_doppler_hz<<" [Hz]"<< std::endl;
							   //if (d_last_seg==5) d_carrier_lock_fail_counter=500; //DEBUG: force unlock!
						   }
				   }
			   else
				   {
					   if (floor(d_sample_counter / d_fs_in) != d_last_seg)
						   {
							   d_last_seg = floor(d_sample_counter / d_fs_in);
							   std::cout  << "Galileo E5 Tracking CH " << d_channel <<  ": Satellite "
							   << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN)
							   << ", CN0 = " << d_CN0_SNV_dB_Hz << " [dB-Hz] "<<"Doppler="<<d_carrier_doppler_hz<<" [Hz]"<< std::endl;
							   //std::cout<<"TRK CH "<<d_channel<<" Carrier_lock_test="<<d_carrier_lock_test<< std::endl;
						   }
				   }


		    }
		else
		    {
			// make an output to not stop the rest of the processing blocks
			current_synchro_data.Prompt_I = 0.0;
			current_synchro_data.Prompt_Q = 0.0;
			current_synchro_data.Tracking_timestamp_secs = static_cast<double>(d_sample_counter) /  static_cast<double>(d_fs_in);
			current_synchro_data.Carrier_phase_rads = 0.0;
			current_synchro_data.Code_phase_secs = 0.0;
			current_synchro_data.CN0_dB_hz = 0.0;
			current_synchro_data.Flag_valid_tracking = false;

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
		    }
		*out[0] = current_synchro_data;
		break;
	    }
    }

    if(d_dump)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            float prompt_I;
            float prompt_Q;
            float tmp_float;
            double tmp_double;
            prompt_I = (d_Prompt_data).real();
            prompt_Q = (d_Prompt_data).imag();
	    if (d_integration_counter == d_current_ti_ms)
        	{
        	    tmp_E = std::abs<float>(d_Early);
        	    tmp_P = std::abs<float>(d_Prompt);
        	    tmp_L = std::abs<float>(d_Late);
        	}
            try
            {
        	// EPR
        	d_dump_file.write(reinterpret_cast<char*>(&tmp_E), sizeof(float));
        	d_dump_file.write(reinterpret_cast<char*>(&tmp_P), sizeof(float));
        	d_dump_file.write(reinterpret_cast<char*>(&tmp_L), sizeof(float));
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
                LOG(WARNING) << "Exception writing trk dump file " << e.what();
            }
        }

    d_secondary_delay = (d_secondary_delay + 1) % Galileo_E5a_Q_SECONDARY_CODE_LENGTH;
    d_sample_counter += d_current_prn_length_samples; //count for the processed samples
    consume_each(d_current_prn_length_samples); // this is necessary in gr::block derivates
    return 1; //output tracking result ALWAYS even in the case of d_enable_tracking==false
}

void Galileo_E5a_Dll_Pll_Tracking_cc::set_channel(unsigned int channel)
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
                            LOG(INFO) << "Tracking dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str() << std::endl;
                    }
                    catch (std::ifstream::failure e)
                    {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what() << std::endl;
                    }
                }
        }
}



void Galileo_E5a_Dll_Pll_Tracking_cc::set_channel_queue(concurrent_queue<int> *channel_internal_queue)
{
    d_channel_internal_queue = channel_internal_queue;
}


void Galileo_E5a_Dll_Pll_Tracking_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
}

