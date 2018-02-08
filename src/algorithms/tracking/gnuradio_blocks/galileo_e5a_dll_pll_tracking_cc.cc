/*!
 * \file galileo_e5a_dll_pll_tracking_cc.h
 * \brief Implementation of a code DLL + carrier PLL
 *  tracking block for Galileo E5a signals
 * \author Marc Sales, 2014. marcsales92(at)gmail.com
 * \based on work from:
 *          <ul>
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
#include <glog/logging.h>
#include <matio.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
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
            fs_in, vector_length, dump, dump_filename, pll_bw_hz, dll_bw_hz, pll_bw_init_hz, dll_bw_init_hz, ti_ms, early_late_space_chips));
}



void Galileo_E5a_Dll_Pll_Tracking_cc::forecast (int noutput_items, gr_vector_int &ninput_items_required)
{
    if (noutput_items != 0)
        {
            ninput_items_required[0] = static_cast<int>(d_vector_length) * 2; //set the required available samples in each call
        }
}


Galileo_E5a_Dll_Pll_Tracking_cc::Galileo_E5a_Dll_Pll_Tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
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
    // Telemetry bit synchronization message port input
    this->message_port_register_in(pmt::mp("preamble_timestamp_s"));
    this->message_port_register_out(pmt::mp("events"));
    this->set_relative_rate(1.0 / vector_length);
    // initialize internal vars
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
    d_codeQ = static_cast<gr_complex*>(volk_gnsssdr_malloc(Galileo_E5a_CODE_LENGTH_CHIPS * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_codeI = static_cast<gr_complex*>(volk_gnsssdr_malloc(Galileo_E5a_CODE_LENGTH_CHIPS * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    // correlator Q outputs (scalar)
    d_n_correlator_taps = 3; //  Early, Prompt, Late
    d_correlator_outs = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_n_correlator_taps*sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    for (int n = 0; n < d_n_correlator_taps; n++)
        {
            d_correlator_outs[n] = gr_complex(0,0);
        }

    // map memory pointers of correlator outputs
    d_Single_Early = &d_correlator_outs[0];
    d_Single_Prompt = &d_correlator_outs[1];
    d_Single_Late = &d_correlator_outs[2];

    d_local_code_shift_chips = static_cast<float*>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(float), volk_gnsssdr_get_alignment()));
    // Set TAPs delay values [chips]
    d_local_code_shift_chips[0] = - d_early_late_spc_chips;
    d_local_code_shift_chips[1] = 0.0;
    d_local_code_shift_chips[2] = d_early_late_spc_chips;

    multicorrelator_cpu_Q.init(2 * d_vector_length, d_n_correlator_taps);

    // correlator I single output for data (scalar)
    d_Single_Prompt_data=static_cast<gr_complex*>(volk_gnsssdr_malloc(sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    *d_Single_Prompt_data = gr_complex(0,0);
    multicorrelator_cpu_I.init(2 * d_vector_length, 1); // single correlator for data channel

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

    d_rem_code_phase_chips = 0.0;
    d_code_phase_step_chips = 0.0;
    d_carrier_phase_step_rad = 0.0;

    systemName["E"] = std::string("Galileo");
}


Galileo_E5a_Dll_Pll_Tracking_cc::~Galileo_E5a_Dll_Pll_Tracking_cc()
{
    if (d_dump_file.is_open())
        {
            try
            {
                    d_dump_file.close();
            }
            catch(const std::exception & ex)
            {
                    LOG(WARNING)<<"Exception in destructor "<<ex.what();
            }
        }

    if(d_dump)
        {
            if(d_channel == 0)
                {
                    std::cout << "Writing .mat files ...";
                }
            Galileo_E5a_Dll_Pll_Tracking_cc::save_matfile();
            if(d_channel == 0)
                {
                    std::cout << " done." << std::endl;
                }
        }

    try
    {
            delete[] d_codeI;
            delete[] d_codeQ;
            delete[] d_Prompt_buffer;
            volk_gnsssdr_free(d_local_code_shift_chips);
            volk_gnsssdr_free(d_correlator_outs);
            volk_gnsssdr_free(d_Single_Prompt_data);
            multicorrelator_cpu_Q.free();
            multicorrelator_cpu_I.free();
    }
    catch(const std::exception & ex)
    {
            LOG(WARNING)<<"Exception in destructor "<<ex.what();
    }
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
    double acq_trk_diff_seconds;
    acq_trk_diff_samples = static_cast<long int>(d_sample_counter) - static_cast<long int>(d_acq_sample_stamp);//-d_vector_length;
    LOG(INFO) << "Number of samples between Acquisition and Tracking =" << acq_trk_diff_samples;
    acq_trk_diff_seconds = static_cast<float>(acq_trk_diff_samples) / static_cast<float>(d_fs_in);
    //doppler effect
    // Fd=(C/(C+Vr))*F
    double radial_velocity;
    radial_velocity = (Galileo_E5a_FREQ_HZ + d_acq_carrier_doppler_hz)/Galileo_E5a_FREQ_HZ;
    // new chip and prn sequence periods based on acq Doppler
    double T_chip_mod_seconds;
    double T_prn_mod_seconds;
    double T_prn_mod_samples;
    d_code_freq_chips = radial_velocity * Galileo_E5a_CODE_CHIP_RATE_HZ;
    T_chip_mod_seconds = 1/d_code_freq_chips;
    T_prn_mod_seconds = T_chip_mod_seconds * Galileo_E5a_CODE_LENGTH_CHIPS;
    T_prn_mod_samples = T_prn_mod_seconds * static_cast<float>(d_fs_in);

    d_current_prn_length_samples = round(T_prn_mod_samples);

    double T_prn_true_seconds = Galileo_E5a_CODE_LENGTH_CHIPS / Galileo_E5a_CODE_CHIP_RATE_HZ;
    double T_prn_true_samples = T_prn_true_seconds * static_cast<float>(d_fs_in);
    double T_prn_diff_seconds;
    T_prn_diff_seconds = T_prn_true_seconds - T_prn_mod_seconds;
    double N_prn_diff;
    N_prn_diff = acq_trk_diff_seconds / T_prn_true_seconds;
    double corrected_acq_phase_samples, delay_correction_samples;
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
    galileo_e5_a_code_gen_complex_primary(d_codeQ, d_acquisition_gnss_synchro->PRN, sig);

    strcpy(sig,"5I");
    galileo_e5_a_code_gen_complex_primary(d_codeI, d_acquisition_gnss_synchro->PRN, sig);

    d_carrier_lock_fail_counter = 0;
    d_rem_code_phase_samples = 0;
    d_rem_carr_phase_rad = 0;
    d_acc_carrier_phase_rad = 0;
    d_acc_code_phase_secs = 0;

    d_code_phase_samples = d_acq_code_phase_samples;

    std::string sys_ = &d_acquisition_gnss_synchro->System;
    sys = sys_.substr(0,1);

    // DEBUG OUTPUT
    std::cout << "Tracking of Galileo E5a signal started on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
    LOG(INFO) << "Galileo E5a starting tracking of satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;

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
            if (Galileo_E5a_Q_SECONDARY_CODE[d_acquisition_gnss_synchro->PRN - 1].at(i) == '0')
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
                    out_corr += in_corr[j] * -sec_code_signed[(j + i) % Galileo_E5a_Q_SECONDARY_CODE_LENGTH];
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


int Galileo_E5a_Dll_Pll_Tracking_cc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    // process vars
    double carr_error_hz;
    double carr_error_filt_hz;
    double code_error_chips;
    double code_error_filt_chips;

    // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
    Gnss_Synchro **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]); //block output streams pointer

    // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
    Gnss_Synchro current_synchro_data;
    // Fill the acquisition data
    current_synchro_data = *d_acquisition_gnss_synchro;

    /* States:     0 Tracking not enabled
     *         1 Pull-in of primary code (alignment).
     *         3 Tracking algorithm. Correlates EPL each loop and accumulates the result
     *                     until it reaches integration time.
     */
    switch (d_state)
    {
    case 0:
        {
            d_Early = gr_complex(0,0);
            d_Prompt = gr_complex(0,0);
            d_Late = gr_complex(0,0);
            d_Prompt_data = gr_complex(0,0);
            current_synchro_data.Tracking_sample_counter = d_sample_counter;
            break;
        }
    case 1:
        {
            int samples_offset;
            double acq_trk_shif_correction_samples;
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
            current_synchro_data.Tracking_sample_counter = d_sample_counter;
            current_synchro_data.Carrier_phase_rads = 0.0;
            current_synchro_data.CN0_dB_hz = 0.0;
            current_synchro_data.fs = d_fs_in;
            consume_each(samples_offset); //shift input to perform alignment with local replica
            return 0;
            break;
        }
    case 2:
        {
            // Block input data and block output stream pointers
            const gr_complex* in = reinterpret_cast<const gr_complex *>(input_items[0]); //PRN start block alignment
            gr_complex sec_sign_Q;
            gr_complex sec_sign_I;
            // Secondary code Chip
            if (d_secondary_lock)
                {
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
                    // Reset accumulated values
                    d_Early = gr_complex(0,0);
                    d_Prompt = gr_complex(0,0);
                    d_Late = gr_complex(0,0);
                }

            // perform carrier wipe-off and compute Early, Prompt and Late
            // correlation of 1 primary code

            multicorrelator_cpu_Q.set_local_code_and_taps(Galileo_E5a_CODE_LENGTH_CHIPS, d_codeQ, d_local_code_shift_chips);
            multicorrelator_cpu_I.set_local_code_and_taps(Galileo_E5a_CODE_LENGTH_CHIPS, d_codeI, &d_local_code_shift_chips[1]);

            // ################# CARRIER WIPEOFF AND CORRELATORS ##############################
            // perform carrier wipe-off and compute Early, Prompt and Late correlation
            multicorrelator_cpu_Q.set_input_output_vectors(d_correlator_outs,in);
            multicorrelator_cpu_I.set_input_output_vectors(d_Single_Prompt_data,in);

            double carr_phase_step_rad = GALILEO_TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);
            double code_phase_step_chips = d_code_freq_chips / (static_cast<double>(d_fs_in));
            double rem_code_phase_chips = d_rem_code_phase_samples * (d_code_freq_chips / d_fs_in);
            multicorrelator_cpu_Q.Carrier_wipeoff_multicorrelator_resampler(
                    d_rem_carr_phase_rad,
                    carr_phase_step_rad,
                    rem_code_phase_chips,
                    code_phase_step_chips,
                    d_current_prn_length_samples);

            multicorrelator_cpu_I.Carrier_wipeoff_multicorrelator_resampler(
                    d_rem_carr_phase_rad,
                    carr_phase_step_rad,
                    rem_code_phase_chips,
                    code_phase_step_chips,
                    d_current_prn_length_samples);

            // Accumulate results (coherent integration since there are no bit transitions in pilot signal)
            d_Early += (*d_Single_Early) * sec_sign_Q;
            d_Prompt += (*d_Single_Prompt) * sec_sign_Q;
            d_Late += (*d_Single_Late) * sec_sign_Q;
            d_Prompt_data=(*d_Single_Prompt_data);
            d_Prompt_data *= sec_sign_I;
            d_integration_counter++;

            // ################## PLL ##########################################################
            // PLL discriminator
            if (d_integration_counter == d_current_ti_ms)
                {
                    if (d_secondary_lock == true)
                        {
                            carr_error_hz = pll_four_quadrant_atan(d_Prompt) / GALILEO_PI * 2.0;
                        }
                    else
                        {
                            carr_error_hz = pll_cloop_two_quadrant_atan(d_Prompt) / GALILEO_PI * 2.0;
                        }

                    // Carrier discriminator filter
                    carr_error_filt_hz = d_carrier_loop_filter.get_carrier_nco(carr_error_hz);
                    // New carrier Doppler frequency estimation
                    d_carrier_doppler_hz = d_acq_carrier_doppler_hz + carr_error_filt_hz;
                    // New code Doppler frequency estimation
                    d_code_freq_chips = Galileo_E5a_CODE_CHIP_RATE_HZ + ((d_carrier_doppler_hz * Galileo_E5a_CODE_CHIP_RATE_HZ) / Galileo_E5a_FREQ_HZ);
                }
            // carrier phase accumulator for (K) doppler estimation
            d_acc_carrier_phase_rad -= 2.0 * GALILEO_PI * d_carrier_doppler_hz * GALILEO_E5a_CODE_PERIOD;
            // remnant carrier phase to prevent overflow in the code NCO
            d_rem_carr_phase_rad = d_rem_carr_phase_rad + 2.0 * GALILEO_PI * d_carrier_doppler_hz * GALILEO_E5a_CODE_PERIOD;
            d_rem_carr_phase_rad = fmod(d_rem_carr_phase_rad, 2.0 * GALILEO_PI);

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
            T_chip_seconds = 1.0 / d_code_freq_chips;
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
                                    std::cout << "Galileo E5a secondary code locked for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
                                    d_current_ti_ms = d_ti_ms;
                                    // Change loop parameters ==========================================
                                    d_code_loop_filter.set_pdi(d_current_ti_ms * GALILEO_E5a_CODE_PERIOD);
                                    d_carrier_loop_filter.set_pdi(d_current_ti_ms * GALILEO_E5a_CODE_PERIOD);
                                    d_code_loop_filter.set_DLL_BW(d_dll_bw_hz);
                                    d_carrier_loop_filter.set_PLL_BW(d_pll_bw_hz);
                                }
                            else
                                {
                                    //std::cout << "Secondary code delay couldn't be resolved." << std::endl;
                                    d_carrier_lock_fail_counter++;
                                    if (d_carrier_lock_fail_counter > MAXIMUM_LOCK_FAIL_COUNTER)
                                        {
                                            std::cout << "Loss of lock in channel " << d_channel << "!" << std::endl;
                                            LOG(INFO) << "Loss of lock in channel " << d_channel << "!";
                                            this->message_port_pub(pmt::mp("events"), pmt::from_long(3));//3 -> loss of lock
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
                                            this->message_port_pub(pmt::mp("events"), pmt::from_long(3));//3 -> loss of lock
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
                    current_synchro_data.Prompt_I = static_cast<double>(d_Prompt_data.real());
                    current_synchro_data.Prompt_Q = static_cast<double>(d_Prompt_data.imag());
                    current_synchro_data.Tracking_sample_counter = d_sample_counter + d_current_prn_length_samples;
                    current_synchro_data.Code_phase_samples = d_rem_code_phase_samples;
                    current_synchro_data.Carrier_phase_rads = d_acc_carrier_phase_rad;
                    current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
                    current_synchro_data.CN0_dB_hz = d_CN0_SNV_dB_Hz;
                    current_synchro_data.Flag_valid_symbol_output = true;
                }
            else
                {
                    // make an output to not stop the rest of the processing blocks
                    current_synchro_data.Prompt_I = 0.0;
                    current_synchro_data.Prompt_Q = 0.0;
                    current_synchro_data.Tracking_sample_counter = d_sample_counter;
                    current_synchro_data.Carrier_phase_rads = 0.0;
                    current_synchro_data.CN0_dB_hz = 0.0;
                    current_synchro_data.Flag_valid_symbol_output = false;
                }

            break;
        }
    }

    current_synchro_data.fs = d_fs_in;
    current_synchro_data.correlation_length_ms = GALILEO_E5a_CODE_PERIOD_MS;
    if(current_synchro_data.Flag_valid_symbol_output) { *out[0] = current_synchro_data; }

    if(d_dump)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            float prompt_I;
            float prompt_Q;
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
                    //tmp_float=(float)d_sample_counter;
                    d_dump_file.write(reinterpret_cast<char*>(&d_sample_counter), sizeof(unsigned long int));
                    // accumulated carrier phase
                    d_dump_file.write(reinterpret_cast<char*>(&d_acc_carrier_phase_rad), sizeof(double));

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
                    tmp_double = d_rem_code_phase_samples;
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                    tmp_double = static_cast<double>(d_sample_counter + d_current_prn_length_samples);
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));

                    // PRN
                    unsigned int prn_ = d_acquisition_gnss_synchro->PRN;
                    d_dump_file.write(reinterpret_cast<char*>(&prn_), sizeof(unsigned int));

            }
            catch (const std::ifstream::failure & e)
            {
                    LOG(WARNING) << "Exception writing trk dump file " << e.what();
            }
        }

    d_secondary_delay = (d_secondary_delay + 1) % Galileo_E5a_Q_SECONDARY_CODE_LENGTH;
    d_sample_counter += d_current_prn_length_samples;
    consume_each(d_current_prn_length_samples);

    if(current_synchro_data.Flag_valid_symbol_output) { return 1; }
    else { return 0; }
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
                            LOG(INFO) << "Tracking dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                    }
                    catch (const std::ifstream::failure &e)
                    {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                    }
                }
        }
}


int Galileo_E5a_Dll_Pll_Tracking_cc::save_matfile()
{
    // READ DUMP FILE
    std::ifstream::pos_type size;
    int number_of_double_vars = 11;
    int number_of_float_vars = 5;
    int epoch_size_bytes = sizeof(unsigned long int) + sizeof(double) * number_of_double_vars +
            sizeof(float) * number_of_float_vars + sizeof(unsigned int);
    std::ifstream dump_file;
    dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
    {
            dump_file.open(d_dump_filename.c_str(), std::ios::binary | std::ios::ate);
    }
    catch(const std::ifstream::failure &e)
    {
            std::cerr << "Problem opening dump file:" <<  e.what() << std::endl;
            return 1;
    }
    // count number of epochs and rewind
    long int num_epoch = 0;
    if (dump_file.is_open())
        {
            size = dump_file.tellg();
            num_epoch = static_cast<long int>(size) / static_cast<long int>(epoch_size_bytes);
            dump_file.seekg(0, std::ios::beg);
        }
    else
        {
            return 1;
        }
    float * abs_E = new float [num_epoch];
    float * abs_P = new float [num_epoch];
    float * abs_L = new float [num_epoch];
    float * Prompt_I = new float [num_epoch];
    float * Prompt_Q = new float [num_epoch];
    unsigned long int * PRN_start_sample_count = new unsigned long int [num_epoch];
    double * acc_carrier_phase_rad = new double [num_epoch];
    double * carrier_doppler_hz = new double [num_epoch];
    double * code_freq_chips = new double [num_epoch];
    double * carr_error_hz = new double [num_epoch];
    double * carr_error_filt_hz = new double [num_epoch];
    double * code_error_chips = new double [num_epoch];
    double * code_error_filt_chips = new double [num_epoch];
    double * CN0_SNV_dB_Hz = new double [num_epoch];
    double * carrier_lock_test = new double [num_epoch];
    double * aux1 = new double [num_epoch];
    double * aux2 = new double [num_epoch];
    unsigned int * PRN = new unsigned int [num_epoch];

    try
    {
            if (dump_file.is_open())
                {
                    for(long int i = 0; i < num_epoch; i++)
                        {
                            dump_file.read(reinterpret_cast<char *>(&abs_E[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_P[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_L[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&Prompt_I[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&Prompt_Q[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&PRN_start_sample_count[i]), sizeof(unsigned long int));
                            dump_file.read(reinterpret_cast<char *>(&acc_carrier_phase_rad[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&carrier_doppler_hz[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&code_freq_chips[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&carr_error_hz[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&carr_error_filt_hz[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&code_error_chips[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&code_error_filt_chips[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&CN0_SNV_dB_Hz[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&carrier_lock_test[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&aux1[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&aux2[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&PRN[i]), sizeof(unsigned int));
                        }
                }
            dump_file.close();
    }
    catch (const std::ifstream::failure &e)
    {
            std::cerr << "Problem reading dump file:" <<  e.what() << std::endl;
            delete[] abs_E;
            delete[] abs_P;
            delete[] abs_L;
            delete[] Prompt_I;
            delete[] Prompt_Q;
            delete[] PRN_start_sample_count;
            delete[] acc_carrier_phase_rad;
            delete[] carrier_doppler_hz;
            delete[] code_freq_chips;
            delete[] carr_error_hz;
            delete[] carr_error_filt_hz;
            delete[] code_error_chips;
            delete[] code_error_filt_chips;
            delete[] CN0_SNV_dB_Hz;
            delete[] carrier_lock_test;
            delete[] aux1;
            delete[] aux2;
            delete[] PRN;
            return 1;
    }

    // WRITE MAT FILE
    mat_t *matfp;
    matvar_t *matvar;
    std::string filename = d_dump_filename;
    filename.erase(filename.length() - 4, 4);
    filename.append(".mat");
    matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT73);
    if(reinterpret_cast<long*>(matfp) != NULL)
        {
            size_t dims[2] = {1, static_cast<size_t>(num_epoch)};
            matvar = Mat_VarCreate("abs_E", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, abs_E, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_P", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, abs_P, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_L", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, abs_L, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Prompt_I", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, Prompt_I, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Prompt_Q", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, Prompt_Q, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN_start_sample_count", MAT_C_UINT64, MAT_T_UINT64, 2, dims, PRN_start_sample_count, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("acc_carrier_phase_rad", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, acc_carrier_phase_rad, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carrier_doppler_hz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, carrier_doppler_hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_freq_chips", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, code_freq_chips, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carr_error_hz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, carr_error_hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carr_error_filt_hz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, carr_error_filt_hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_error_chips", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, code_error_chips, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_error_filt_chips", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, code_error_filt_chips, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("CN0_SNV_dB_Hz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, CN0_SNV_dB_Hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carrier_lock_test", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, carrier_lock_test, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("aux1", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, aux1, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("aux2", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, aux2, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN", MAT_C_UINT32, MAT_T_UINT32, 2, dims, PRN, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);
        }
    Mat_Close(matfp);
    delete[] abs_E;
    delete[] abs_P;
    delete[] abs_L;
    delete[] Prompt_I;
    delete[] Prompt_Q;
    delete[] PRN_start_sample_count;
    delete[] acc_carrier_phase_rad;
    delete[] carrier_doppler_hz;
    delete[] code_freq_chips;
    delete[] carr_error_hz;
    delete[] carr_error_filt_hz;
    delete[] code_error_chips;
    delete[] code_error_filt_chips;
    delete[] CN0_SNV_dB_Hz;
    delete[] carrier_lock_test;
    delete[] aux1;
    delete[] aux2;
    delete[] PRN;
    return 0;
}


void Galileo_E5a_Dll_Pll_Tracking_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
}
