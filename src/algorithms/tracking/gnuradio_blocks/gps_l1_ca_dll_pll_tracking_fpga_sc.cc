/*!
 * \file gps_l1_ca_dll_pll_tracking_cc.cc
 * \brief Implementation of a code DLL + carrier PLL tracking block
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
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

#include "gps_l1_ca_dll_pll_tracking_fpga_sc.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include "gps_sdr_signal_processing.h"
#include "tracking_discriminators.h"
#include "lock_detectors.h"
#include "GPS_L1_CA.h"
#include "control_message_factory.h"


/*!
 * \todo Include in definition header file
 */
#define CN0_ESTIMATION_SAMPLES 20
#define MINIMUM_VALID_CN0 25
#define MAXIMUM_LOCK_FAIL_COUNTER 50
#define CARRIER_LOCK_THRESHOLD 0.85


using google::LogMessage;

gps_l1_ca_dll_pll_tracking_fpga_sc_sptr
gps_l1_ca_dll_pll_make_tracking_fpga_sc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float early_late_space_chips,
        std::string device_name, 
        unsigned int device_base)
{
    return gps_l1_ca_dll_pll_tracking_fpga_sc_sptr(new Gps_L1_Ca_Dll_Pll_Tracking_fpga_sc(if_freq,
            fs_in, vector_length, dump, dump_filename, pll_bw_hz, dll_bw_hz, early_late_space_chips, device_name, device_base));
}

Gps_L1_Ca_Dll_Pll_Tracking_fpga_sc::Gps_L1_Ca_Dll_Pll_Tracking_fpga_sc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float early_late_space_chips,
        std::string device_name, 
        unsigned int device_base) :
                gr::block("Gps_L1_Ca_Dll_Pll_Tracking_fpga_sc", gr::io_signature::make(0, 0, sizeof(lv_16sc_t)),
                        gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    // Telemetry bit synchronization message port input
    this->message_port_register_in(pmt::mp("preamble_timestamp_s"));
    this->message_port_register_out(pmt::mp("events"));

    // initialize internal vars
    d_dump = dump;
    d_if_freq = if_freq;
    d_fs_in = fs_in;
    d_vector_length = vector_length;
    d_dump_filename = dump_filename;
    d_current_prn_length_samples = static_cast<int>(d_vector_length);
    d_correlation_length_samples = static_cast<int>(d_vector_length);
    
    // Initialize tracking  ==========================================
    d_code_loop_filter.set_DLL_BW(dll_bw_hz);
    d_carrier_loop_filter.set_PLL_BW(pll_bw_hz);

    //--- DLL variables --------------------------------------------------------
    d_early_late_spc_chips = early_late_space_chips; // Define early-late offset (in chips)

    // Initialization of local code replica
    // Get space for a vector with the C/A code replica sampled 1x/chip
    //d_ca_code = static_cast<float*>(volk_gnsssdr_malloc(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS) * sizeof(float), volk_gnsssdr_get_alignment()));
    //d_ca_code_16sc = static_cast<lv_16sc_t*>(volk_gnsssdr_malloc(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS) * sizeof(lv_16sc_t), volk_gnsssdr_get_alignment()));
    //d_ca_code_16sc = static_cast<int*>(volk_gnsssdr_malloc(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS) * sizeof(int), volk_gnsssdr_get_alignment()));

    // correlator outputs (scalar)
    d_n_correlator_taps = 3; // Early, Prompt, and Late
    d_correlator_outs = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    for (int n = 0; n < d_n_correlator_taps; n++)
        {
            d_correlator_outs[n] = gr_complex(0,0);
        }
    d_local_code_shift_chips = static_cast<float*>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(float), volk_gnsssdr_get_alignment()));

    // Set TAPs delay values [chips]
    d_local_code_shift_chips[0] = - d_early_late_spc_chips;
    d_local_code_shift_chips[1] = 0.0;
    d_local_code_shift_chips[2] = d_early_late_spc_chips;

    // create multicorrelator class
    multicorrelator_fpga_8sc = std::make_shared <fpga_multicorrelator_8sc>(d_n_correlator_taps, device_name, device_base);

    //--- Perform initializations ------------------------------
    // define initial code frequency basis of NCO
    d_code_freq_chips = GPS_L1_CA_CODE_RATE_HZ;
    // define residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    // define residual carrier phase
    d_rem_carr_phase_rad = 0.0;

    // sample synchronization
    d_sample_counter = 0;
    d_acq_sample_stamp = 0;

    d_enable_tracking = false;
    d_pull_in = false;

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[CN0_ESTIMATION_SAMPLES];
    d_carrier_lock_test = 1;
    d_CN0_SNV_dB_Hz = 0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_threshold = CARRIER_LOCK_THRESHOLD;

    systemName["G"] = std::string("GPS");
    systemName["S"] = std::string("SBAS");

    d_acquisition_gnss_synchro = 0;
    d_channel = 0;
    d_acq_code_phase_samples = 0.0;
    d_acq_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_code_phase_samples = 0.0;
    d_rem_code_phase_chips = 0.0;
    d_code_phase_step_chips = 0.0;
    d_carrier_phase_step_rad = 0.0;

    set_relative_rate(1.0 / static_cast<double>(d_vector_length));
}

void Gps_L1_Ca_Dll_Pll_Tracking_fpga_sc::start_tracking()
{
    /*
     *  correct the code phase according to the delay between acq and trk
     */
     
    //printf("TRK : start tracking for satellite %d\n", d_acquisition_gnss_synchro->PRN);
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp = d_acquisition_gnss_synchro->Acq_samplestamp_samples;
    long int acq_trk_diff_samples;
    double acq_trk_diff_seconds;
    acq_trk_diff_samples = static_cast<long int>(d_sample_counter) - static_cast<long int>(d_acq_sample_stamp); //-d_vector_length;
    DLOG(INFO) << "Number of samples between Acquisition and Tracking = " << acq_trk_diff_samples;
    acq_trk_diff_seconds = static_cast<float>(acq_trk_diff_samples) / static_cast<float>(d_fs_in);
    // Doppler effect
    // Fd=(C/(C+Vr))*F
    double radial_velocity = (GPS_L1_FREQ_HZ + d_acq_carrier_doppler_hz) / GPS_L1_FREQ_HZ;
    // new chip and prn sequence periods based on acq Doppler
    double T_chip_mod_seconds;
    double T_prn_mod_seconds;
    double T_prn_mod_samples;
    d_code_freq_chips = radial_velocity * GPS_L1_CA_CODE_RATE_HZ;
    d_code_phase_step_chips = static_cast<double>(d_code_freq_chips) / static_cast<double>(d_fs_in);
    T_chip_mod_seconds = 1/d_code_freq_chips;
    T_prn_mod_seconds = T_chip_mod_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
    T_prn_mod_samples = T_prn_mod_seconds * static_cast<double>(d_fs_in);
    d_current_prn_length_samples = round(T_prn_mod_samples);
    double T_prn_true_seconds = GPS_L1_CA_CODE_LENGTH_CHIPS / GPS_L1_CA_CODE_RATE_HZ;
    double T_prn_true_samples = T_prn_true_seconds * static_cast<double>(d_fs_in);
    double T_prn_diff_seconds = T_prn_true_seconds - T_prn_mod_seconds;
    double N_prn_diff = acq_trk_diff_seconds / T_prn_true_seconds;
    double corrected_acq_phase_samples, delay_correction_samples;
    corrected_acq_phase_samples = fmod((d_acq_code_phase_samples + T_prn_diff_seconds * N_prn_diff * static_cast<double>(d_fs_in)), T_prn_true_samples);
    if (corrected_acq_phase_samples < 0)
        {
            corrected_acq_phase_samples = T_prn_mod_samples + corrected_acq_phase_samples;
        }
    delay_correction_samples = d_acq_code_phase_samples - corrected_acq_phase_samples;
    d_acq_code_phase_samples = corrected_acq_phase_samples;
    d_carrier_doppler_hz = d_acq_carrier_doppler_hz;
    d_carrier_phase_step_rad = GPS_TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);
    // DLL/PLL filter initialization
    d_carrier_loop_filter.initialize(); // initialize the carrier filter
    d_code_loop_filter.initialize();    // initialize the code filter
    // generate local reference ALWAYS starting at chip 1 (1 sample per chip)
    //gps_l1_ca_code_gen_float(d_ca_code, d_acquisition_gnss_synchro->PRN, 0);
    //gps_l1_ca_code_gen_int(d_ca_code_16sc, d_acquisition_gnss_synchro->PRN, 0);
/*	for (int n = 0; n < static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS); n++)
	{
		d_ca_code_16sc[n] = d_ca_code[n];
	} */
    //multicorrelator_fpga_8sc->set_local_code_and_taps(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS), d_ca_code_16sc, d_local_code_shift_chips, d_acquisition_gnss_synchro->PRN);
    multicorrelator_fpga_8sc->set_local_code_and_taps(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS), d_local_code_shift_chips, d_acquisition_gnss_synchro->PRN);
    for (int n = 0; n < d_n_correlator_taps; n++)
        {
            d_correlator_outs[n] = gr_complex(0,0);
        }
    d_carrier_lock_fail_counter = 0;
    d_rem_code_phase_samples = 0;
    d_rem_carr_phase_rad = 0.0;
    d_rem_code_phase_chips = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_code_phase_samples = d_acq_code_phase_samples;
    std::string sys_ = &d_acquisition_gnss_synchro->System;
    sys = sys_.substr(0,1);
    std::cout << "Tracking of GPS L1 C/A signal started on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
    LOG(INFO) << "Starting tracking of satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;
    // enable tracking
    d_pull_in = true;
    d_enable_tracking = true; //do it in the end to avoid starting running tracking before finishing this function
    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz
            << " Code Phase correction [samples]=" << delay_correction_samples
            << " PULL-IN Code Phase [samples]=" << d_acq_code_phase_samples;
}

Gps_L1_Ca_Dll_Pll_Tracking_fpga_sc::~Gps_L1_Ca_Dll_Pll_Tracking_fpga_sc()
{
    if (d_dump_file.is_open())
        {
            try
            {
                    d_dump_file.close();
            }
            catch(const std::exception & ex)
            {
                    LOG(WARNING) << "Exception in destructor " << ex.what();
            }
        }
    try
    {
            volk_gnsssdr_free(d_local_code_shift_chips);
            volk_gnsssdr_free(d_correlator_outs);
            delete[] d_Prompt_buffer;
            multicorrelator_fpga_8sc->free();
    }
    catch(const std::exception & ex)
    {
            LOG(WARNING) << "Exception in destructor " << ex.what();
    }
}

int Gps_L1_Ca_Dll_Pll_Tracking_fpga_sc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
	
    unsigned absolute_samples_offset;
    // process vars
    double carr_error_hz = 0.0;
    double carr_error_filt_hz = 0.0;
    double code_error_chips = 0.0;
    double code_error_filt_chips = 0.0;

	int next_prn_length_samples = d_current_prn_length_samples;

    // Block input data and block output stream pointers
    Gnss_Synchro **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);

    // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
    Gnss_Synchro current_synchro_data = Gnss_Synchro();

    if (d_enable_tracking == true)
        {
            // Fill the acquisition data
            current_synchro_data = *d_acquisition_gnss_synchro;
            // Receiver signal alignment
            if (d_pull_in == true)
                {
                    d_pull_in = false;
                    multicorrelator_fpga_8sc->lock_channel();
					unsigned counter_value = multicorrelator_fpga_8sc->read_sample_counter();
					unsigned num_frames = ceil((counter_value - current_synchro_data.Acq_samplestamp_samples - current_synchro_data.Acq_delay_samples)/d_correlation_length_samples);
					absolute_samples_offset = current_synchro_data.Acq_delay_samples + current_synchro_data.Acq_samplestamp_samples + num_frames*d_correlation_length_samples;
					multicorrelator_fpga_8sc->set_initial_sample(absolute_samples_offset);
					d_sample_counter = absolute_samples_offset;
					current_synchro_data.Tracking_sample_counter = absolute_samples_offset;
                }
            else
                {
					// continue as from the previous point
					d_sample_counter = d_sample_counter_next;
                }
			d_sample_counter_next = d_sample_counter +  d_current_prn_length_samples;

            // ################# CARRIER WIPEOFF AND CORRELATORS ##############################
            // perform carrier wipe-off and compute Early, Prompt and Late correlation
			multicorrelator_fpga_8sc->set_output_vectors(d_correlator_outs);
			multicorrelator_fpga_8sc->Carrier_wipeoff_multicorrelator_resampler(
			d_rem_carr_phase_rad, d_carrier_phase_step_rad,
			d_rem_code_phase_chips, d_code_phase_step_chips,
			d_current_prn_length_samples);			
            
            // ################## PLL ##########################################################
            // PLL discriminator
            // Update PLL discriminator [rads/Ti -> Secs/Ti]
            carr_error_hz = pll_cloop_two_quadrant_atan(d_correlator_outs[1]) / GPS_TWO_PI; // prompt output
            // Carrier discriminator filter
            carr_error_filt_hz = d_carrier_loop_filter.get_carrier_nco(carr_error_hz);
            // New carrier Doppler frequency estimation
            d_carrier_doppler_hz = d_acq_carrier_doppler_hz + carr_error_filt_hz;
            // New code Doppler frequency estimation
            d_code_freq_chips = GPS_L1_CA_CODE_RATE_HZ + ((d_carrier_doppler_hz * GPS_L1_CA_CODE_RATE_HZ) / GPS_L1_FREQ_HZ);

            // ################## DLL ##########################################################
            // DLL discriminator
            code_error_chips = dll_nc_e_minus_l_normalized(d_correlator_outs[0], d_correlator_outs[2]); // [chips/Ti] //early and late
            // Code discriminator filter
            code_error_filt_chips = d_code_loop_filter.get_code_nco(code_error_chips); // [chips/second]
            double T_chip_seconds = 1.0 / static_cast<double>(d_code_freq_chips);
            double T_prn_seconds = T_chip_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
            double code_error_filt_secs = (T_prn_seconds * code_error_filt_chips*T_chip_seconds); //[seconds]

            // ################## CARRIER AND CODE NCO BUFFER ALIGNEMENT #######################
            // keep alignment parameters for the next input buffer
            // Compute the next buffer length based in the new period of the PRN sequence and the code phase error estimation
            double T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
            double K_blk_samples = T_prn_samples + d_rem_code_phase_samples + code_error_filt_secs * static_cast<double>(d_fs_in);
			next_prn_length_samples = round(K_blk_samples);

            //################### PLL COMMANDS #################################################
            // carrier phase step (NCO phase increment per sample) [rads/sample]
            d_carrier_phase_step_rad = GPS_TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);
            // remnant carrier phase to prevent overflow in the code NCO
            d_rem_carr_phase_rad = d_rem_carr_phase_rad + d_carrier_phase_step_rad * d_current_prn_length_samples;
            d_rem_carr_phase_rad = fmod(d_rem_carr_phase_rad, GPS_TWO_PI);
            // carrier phase accumulator
            d_acc_carrier_phase_rad -= d_carrier_phase_step_rad * d_current_prn_length_samples;

            //################### DLL COMMANDS #################################################
            // code phase step (Code resampler phase increment per sample) [chips/sample]
            d_code_phase_step_chips = d_code_freq_chips / static_cast<double>(d_fs_in);
            // remnant code phase [chips]
            d_rem_code_phase_samples = K_blk_samples - next_prn_length_samples; // rounding error < 1 sample
            d_rem_code_phase_chips = d_code_freq_chips * (d_rem_code_phase_samples / static_cast<double>(d_fs_in));

            // ####### CN0 ESTIMATION AND LOCK DETECTORS ######
            if (d_cn0_estimation_counter < CN0_ESTIMATION_SAMPLES)
                {
                    // fill buffer with prompt correlator output values
                    d_Prompt_buffer[d_cn0_estimation_counter] = d_correlator_outs[1]; //prompt
                    d_cn0_estimation_counter++;
                }
            else
                {
                    d_cn0_estimation_counter = 0;
                    // Code lock indicator
                    d_CN0_SNV_dB_Hz = cn0_svn_estimator(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES, d_fs_in, GPS_L1_CA_CODE_LENGTH_CHIPS);
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
                            this->message_port_pub(pmt::mp("events"), pmt::from_long(3)); // 3 -> loss of lock  
                            d_carrier_lock_fail_counter = 0;
                            d_enable_tracking = false; // TODO: check if disabling tracking is consistent with the channel state machine
                            multicorrelator_fpga_8sc->unlock_channel();  
                        }
                } 

            // ########### Output the tracking data to navigation and PVT ##########
            current_synchro_data.Prompt_I = static_cast<double>((d_correlator_outs[1]).real());
            current_synchro_data.Prompt_Q = static_cast<double>((d_correlator_outs[1]).imag());
            current_synchro_data.Tracking_sample_counter = d_sample_counter + d_current_prn_length_samples;
            current_synchro_data.Code_phase_samples = d_rem_code_phase_samples;
            current_synchro_data.Carrier_phase_rads = d_acc_carrier_phase_rad;
            current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
            current_synchro_data.CN0_dB_hz = d_CN0_SNV_dB_Hz;
            current_synchro_data.Flag_valid_symbol_output = true;
            current_synchro_data.correlation_length_ms = 1;
        }
    else
        {
            for (int n = 0; n < d_n_correlator_taps; n++)
                {
                    d_correlator_outs[n] = gr_complex(0,0);
                }

            current_synchro_data.Tracking_sample_counter =d_sample_counter + d_current_prn_length_samples;
            current_synchro_data.System = {'G'};
            current_synchro_data.correlation_length_ms = 1;
        }

    //assign the GNURadio block output data
    current_synchro_data.fs = d_fs_in;
    *out[0] = current_synchro_data;
    if (d_enable_tracking == true) // in the FPGA case dump data only when tracking is enabled, otherwise the dumped data is useless
    {
		if(d_dump)
			{
				// MULTIPLEXED FILE RECORDING - Record results to file
				float prompt_I;
				float prompt_Q;
				float tmp_E, tmp_P, tmp_L;
				double tmp_double;
				unsigned long int tmp_long;
				prompt_I = d_correlator_outs[1].real();
				prompt_Q = d_correlator_outs[1].imag();
				tmp_E = std::abs<float>(d_correlator_outs[0]);
				tmp_P = std::abs<float>(d_correlator_outs[1]);
				tmp_L = std::abs<float>(d_correlator_outs[2]);
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
						tmp_long = d_sample_counter + d_current_prn_length_samples;
						d_dump_file.write(reinterpret_cast<char*>(&tmp_long), sizeof(unsigned long int));
						// accumulated carrier phase
						d_dump_file.write(reinterpret_cast<char*>(&d_acc_carrier_phase_rad), sizeof(double));

						// carrier and code frequency
						d_dump_file.write(reinterpret_cast<char*>(&d_carrier_doppler_hz), sizeof(double));
						d_dump_file.write(reinterpret_cast<char*>(&d_code_freq_chips), sizeof(double));

						// PLL commands
						d_dump_file.write(reinterpret_cast<char*>(&carr_error_hz), sizeof(double));
						d_dump_file.write(reinterpret_cast<char*>(&carr_error_filt_hz), sizeof(double));

						// DLL commands
						d_dump_file.write(reinterpret_cast<char*>(&code_error_chips), sizeof(double));
						d_dump_file.write(reinterpret_cast<char*>(&code_error_filt_chips), sizeof(double));

						// CN0 and carrier lock test
						d_dump_file.write(reinterpret_cast<char*>(&d_CN0_SNV_dB_Hz), sizeof(double));
						d_dump_file.write(reinterpret_cast<char*>(&d_carrier_lock_test), sizeof(double));

						// AUX vars (for debug purposes)
						tmp_double = d_rem_code_phase_samples;
						d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
						tmp_double = static_cast<double>(d_sample_counter);
						d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));

						// PRN
						unsigned int prn_ = d_acquisition_gnss_synchro->PRN;
						d_dump_file.write(reinterpret_cast<char*>(&prn_), sizeof(unsigned int));
						
				}
				catch (const std::ifstream::failure &e)
				{
						LOG(WARNING) << "Exception writing trk dump file " << e.what(); 
				}
				

			} 
	}
	
	
	d_current_prn_length_samples = next_prn_length_samples;
    d_sample_counter += d_current_prn_length_samples; // count for the processed samples
    
    if (d_enable_tracking == true)
        {
            return 1; 
        }
    else
        {
            return 0;
        }

}



void Gps_L1_Ca_Dll_Pll_Tracking_fpga_sc::set_channel(unsigned int channel)
{
    d_channel = channel;
    multicorrelator_fpga_8sc->set_channel(d_channel);
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
                    catch (const std::ifstream::failure &e)
                    {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                    }
                }
        }
}


void Gps_L1_Ca_Dll_Pll_Tracking_fpga_sc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
}

void Gps_L1_Ca_Dll_Pll_Tracking_fpga_sc::reset(void)
{
    multicorrelator_fpga_8sc->unlock_channel();
}
