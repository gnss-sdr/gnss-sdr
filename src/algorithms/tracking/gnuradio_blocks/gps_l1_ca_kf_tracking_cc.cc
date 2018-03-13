/*!
 * \file gps_l1_ca_kf_tracking_cc.cc
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

#include "gps_l1_ca_kf_tracking_cc.h"
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
#define MAXIMUM_LOCK_FAIL_COUNTER 5000
#define CARRIER_LOCK_THRESHOLD 0.85


using google::LogMessage;

gps_l1_ca_kf_tracking_cc_sptr
gps_l1_ca_kf_make_tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float early_late_space_chips)
{
    return gps_l1_ca_kf_tracking_cc_sptr(new Gps_L1_Ca_Kf_Tracking_cc(if_freq,
            fs_in, vector_length, dump, dump_filename, pll_bw_hz, dll_bw_hz, early_late_space_chips));
}



void Gps_L1_Ca_Kf_Tracking_cc::forecast (int noutput_items,
        gr_vector_int &ninput_items_required)
{
    if (noutput_items != 0)
        {
            ninput_items_required[0] = static_cast<int>(d_vector_length) * 2; //set the required available samples in each call
        }
}



Gps_L1_Ca_Kf_Tracking_cc::Gps_L1_Ca_Kf_Tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float pll_bw_hz,
        float dll_bw_hz,
        float early_late_space_chips) :
                gr::block("Gps_L1_Ca_Kf_Tracking_cc", gr::io_signature::make(1, 1, sizeof(gr_complex)),
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

    // Initialize tracking  ==========================================
    d_code_loop_filter.set_DLL_BW(dll_bw_hz);
    d_carrier_loop_filter.set_PLL_BW(pll_bw_hz);

    //--- DLL variables --------------------------------------------------------
    d_early_late_spc_chips = early_late_space_chips; // Define early-late offset (in chips)

    // Initialization of local code replica
    // Get space for a vector with the C/A code replica sampled 1x/chip
    d_ca_code = static_cast<float*>(volk_gnsssdr_malloc(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS) * sizeof(float), volk_gnsssdr_get_alignment()));

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

    multicorrelator_cpu.init(2 * d_current_prn_length_samples, d_n_correlator_taps);

    //--- Perform initializations ------------------------------
    // define initial code frequency basis of NCO
    d_code_freq_chips = GPS_L1_CA_CODE_RATE_HZ;
    // define residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    // define residual carrier phase
    d_rem_carr_phase_rad = 0.0;

    // sample synchronization
    d_sample_counter = 0;
    //d_sample_counter_seconds = 0;
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

    // Kalman filter initialization (receiver initialization)

    double CN_dB_Hz=40;
    double CN_lin=pow(10,CN_dB_Hz/10.0);

    double sigma2_phase_detector_cycles2;
    sigma2_phase_detector_cycles2=(1.0/(2.0*CN_lin*GPS_L1_CA_CODE_PERIOD))*(1.0+1.0/(2.0*CN_lin*GPS_L1_CA_CODE_PERIOD));

    //covariances (static)
    double sigma2_carrier_phase=GPS_TWO_PI/4.0;
    double sigma2_doppler=250;

    kf_P_x_ini=arma::zeros(2,2);
    kf_P_x_ini(0,0)=sigma2_carrier_phase;
    kf_P_x_ini(1,1)=sigma2_doppler;

    kf_R=arma::zeros(1,1);
    kf_R(0,0)=sigma2_phase_detector_cycles2;

    //arma::colvec G={pow(GPS_L1_CA_CODE_PERIOD,3)/6.0, pow(GPS_L1_CA_CODE_PERIOD,2)/2.0,GPS_L1_CA_CODE_PERIOD};
    kf_Q=arma::zeros(2,2);
    kf_Q(0,0)=1e-14;
    kf_Q(1,1)=1e-2;

//    kf_Q=arma::diagmat(pow(GPS_L1_CA_CODE_PERIOD,6)*kf_Q);
    //std::cout<<"kf_Q="<<kf_Q<<std::endl;

    kf_F=arma::zeros(2,2);
    kf_F(0,0)=1.0;
    kf_F(0,1)=GPS_TWO_PI*GPS_L1_CA_CODE_PERIOD;
    kf_F(1,0)=0.0;
    kf_F(1,1)=1.0;

    kf_H=arma::zeros(1,2);
    kf_H(0,0)=1.0;

    kf_x=arma::zeros(2,1);
    kf_y=arma::zeros(1,1);

}


void Gps_L1_Ca_Kf_Tracking_cc::start_tracking()
{
    /*
     *  correct the code phase according to the delay between acq and trk
     */
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
    gps_l1_ca_code_gen_float(d_ca_code, d_acquisition_gnss_synchro->PRN, 0);

    multicorrelator_cpu.set_local_code_and_taps(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS), d_ca_code, d_local_code_shift_chips);
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

    // DEBUG OUTPUT
    std::cout << "Tracking of GPS L1 C/A signal started on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
    LOG(INFO) << "Starting tracking of satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;

    // enable tracking
    d_pull_in = true;
    d_enable_tracking = true;

    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz
            << " Code Phase correction [samples]=" << delay_correction_samples
            << " PULL-IN Code Phase [samples]=" << d_acq_code_phase_samples;

}


Gps_L1_Ca_Kf_Tracking_cc::~Gps_L1_Ca_Kf_Tracking_cc()
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
            volk_gnsssdr_free(d_ca_code);
            delete[] d_Prompt_buffer;
            multicorrelator_cpu.free();
    }
    catch(const std::exception & ex)
    {
            LOG(WARNING) << "Exception in destructor " << ex.what();
    }
}



int Gps_L1_Ca_Kf_Tracking_cc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    // process vars
    double carr_phase_error_rad = 0.0;
    double carr_phase_error_filt_rad = 0.0;
    double code_error_chips = 0.0;
    double code_error_filt_chips = 0.0;

    // Block input data and block output stream pointers
    const gr_complex* in = reinterpret_cast<const gr_complex *>(input_items[0]);
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
                    int samples_offset;
                    double acq_trk_shif_correction_samples;
                    int acq_to_trk_delay_samples;
                    acq_to_trk_delay_samples = d_sample_counter - d_acq_sample_stamp;
                    acq_trk_shif_correction_samples = d_current_prn_length_samples - fmod(static_cast<float>(acq_to_trk_delay_samples), static_cast<float>(d_current_prn_length_samples));
                    samples_offset = round(d_acq_code_phase_samples + acq_trk_shif_correction_samples);
                    current_synchro_data.Tracking_sample_counter = d_sample_counter + samples_offset;
                    d_sample_counter = d_sample_counter + samples_offset; // count for the processed samples
                    d_pull_in = false;
                    // take into account the carrier cycles accumulated in the pull in signal alignment
                    d_acc_carrier_phase_rad -= d_carrier_phase_step_rad * samples_offset;
                    current_synchro_data.Carrier_phase_rads = d_acc_carrier_phase_rad;
                    current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
                    current_synchro_data.fs = d_fs_in;
                    current_synchro_data.correlation_length_ms = 1;
                    *out[0] = current_synchro_data;
                    //Kalman filter initialization reset
                    kf_P_x=kf_P_x_ini;
                    //Update Kalman states based on acquisition information
                    kf_x(0)=0.0;
                    kf_x(1)=current_synchro_data.Carrier_Doppler_hz;

                    consume_each(samples_offset); // shift input to perform alignment with local replica
                    return 1;
                }

            // ################# CARRIER WIPEOFF AND CORRELATORS ##############################
            // perform carrier wipe-off and compute Early, Prompt and Late correlation
            multicorrelator_cpu.set_input_output_vectors(d_correlator_outs, in);
            multicorrelator_cpu.Carrier_wipeoff_multicorrelator_resampler(d_rem_carr_phase_rad,
                    d_carrier_phase_step_rad,
                    d_rem_code_phase_chips,
                    d_code_phase_step_chips,
                    d_current_prn_length_samples);

            // remnant carrier phase to prevent overflow in the code NCO
            //d_rem_carr_phase_rad = d_rem_carr_phase_rad + d_carrier_phase_step_rad * d_current_prn_length_samples;
            //d_rem_carr_phase_rad = fmod(d_rem_carr_phase_rad, GPS_TWO_PI);

            // ################## Kalman Carrier Tracking ######################################

            //Kalman state prediction (time update)
			kf_x_pre=kf_F*kf_x; //state prediction
			kf_P_x_pre=kf_F*kf_P_x*kf_F.t()+kf_Q; //state error covariance prediction

            // Update discriminator [rads/Ti]
            carr_phase_error_rad = pll_cloop_two_quadrant_atan(d_correlator_outs[1]); // prompt output

			//Kalman estimation (measuremant update)
			//kf_y_pre=kf_H*kf_x_pre; //measurement prediction

			kf_P_y=kf_H*kf_P_x_pre*kf_H.t()+kf_R; //innovation covariance matrix
			kf_K=(kf_P_x_pre*kf_H.t())*arma::inv(kf_P_y); //Kalman gain

			kf_y(0)=carr_phase_error_rad; //measurement vector
			//kf_x=kf_x_pre+kf_K*(kf_y-kf_y_pre); //updated state estimation
			kf_x=kf_x_pre+kf_K*kf_y; //updated state estimation

			kf_x(0)=fmod(arma::as_scalar(kf_x(0)), GPS_TWO_PI);
			//kf_P_x=kf_P_x_pre-kf_K*kf_H*kf_P_x_pre; //update state estimation error covariance matrix
			kf_P_x=(arma::eye(2,2)-kf_K*kf_H)*kf_P_x_pre; //update state estimation error covariance matrix

			d_rem_carr_phase_rad=kf_x(0); //set a new carrier Phase estimation to the NCO
			d_carrier_doppler_hz=kf_x(1); //set a new carrier Doppler estimation to the NCO

			carr_phase_error_filt_rad=d_rem_carr_phase_rad;

            // ################## DLL ##########################################################
			// New code Doppler frequency estimation based on carrier frequency estimation
            d_code_freq_chips = GPS_L1_CA_CODE_RATE_HZ + ((d_carrier_doppler_hz * GPS_L1_CA_CODE_RATE_HZ) / GPS_L1_FREQ_HZ);
            // DLL discriminator
            code_error_chips = dll_nc_e_minus_l_normalized(d_correlator_outs[0], d_correlator_outs[2]); // [chips/Ti] //early and late
            // Code discriminator filter
            code_error_filt_chips = d_code_loop_filter.get_code_nco(code_error_chips); // [chips/second]
            double T_chip_seconds = 1.0 / static_cast<double>(d_code_freq_chips);
            double T_prn_seconds = T_chip_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
            double code_error_filt_secs = (T_prn_seconds * code_error_filt_chips*T_chip_seconds); //[seconds]
            //double code_error_filt_secs = (GPS_L1_CA_CODE_PERIOD * code_error_filt_chips) / GPS_L1_CA_CODE_RATE_HZ; // [seconds]

            // ################## CARRIER AND CODE NCO BUFFER ALIGNEMENT #######################
            // keep alignment parameters for the next input buffer
            // Compute the next buffer length based in the new period of the PRN sequence and the code phase error estimation
            //double T_chip_seconds =  1.0 / static_cast<double>(d_code_freq_chips);
            //double T_prn_seconds = T_chip_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
            double T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
            double K_blk_samples = T_prn_samples + d_rem_code_phase_samples + code_error_filt_secs * static_cast<double>(d_fs_in);
            d_current_prn_length_samples = round(K_blk_samples); // round to a discrete number of samples

            //################### NCO COMMANDS #################################################
            // carrier phase step (NCO phase increment per sample) [rads/sample]
            d_carrier_phase_step_rad = GPS_TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);
            // carrier phase accumulator
            d_acc_carrier_phase_rad -= d_carrier_phase_step_rad * d_current_prn_length_samples;

            //################### DLL COMMANDS #################################################
            // code phase step (Code resampler phase increment per sample) [chips/sample]
            d_code_phase_step_chips = d_code_freq_chips / static_cast<double>(d_fs_in);
            // remnant code phase [chips]
            d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; // rounding error < 1 sample
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
                    //std::cout<<"Channel "<<d_channel<<" CN0: " <<d_CN0_SNV_dB_Hz<<"[dB_Hz]\n";
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
                    d_dump_file.write(reinterpret_cast<char*>(&carr_phase_error_rad), sizeof(double));
                    d_dump_file.write(reinterpret_cast<char*>(&carr_phase_error_filt_rad), sizeof(double));

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

    consume_each(d_current_prn_length_samples); // this is necessary in gr::block derivates
    d_sample_counter += d_current_prn_length_samples; // count for the processed samples
    return 1; // output tracking result ALWAYS even in the case of d_enable_tracking==false
}



void Gps_L1_Ca_Kf_Tracking_cc::set_channel(unsigned int channel)
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
                    catch (const std::ifstream::failure &e)
                    {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                    }
                }
        }
}


void Gps_L1_Ca_Kf_Tracking_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
}
