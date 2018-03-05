/*!
 * \file galileo_e1_tcp_connector_tracking_cc.cc
 * \brief Implementation of a TCP connector block based on Code DLL + carrier PLL
 * \author David Pubill, 2012. dpubill(at)cttc.es
 *         Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * [1] K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkha user, 2007
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

#include "galileo_e1_tcp_connector_tracking_cc.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include "galileo_e1_signal_processing.h"
#include "tracking_discriminators.h"
#include "lock_detectors.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "control_message_factory.h"
#include "gnss_sdr_flags.h"
#include "tcp_communication.h"
#include "tcp_packet_data.h"


using google::LogMessage;

galileo_e1_tcp_connector_tracking_cc_sptr galileo_e1_tcp_connector_make_tracking_cc(
    long if_freq,
    long fs_in,
    unsigned int vector_length,
    bool dump,
    std::string dump_filename,
    float pll_bw_hz,
    float dll_bw_hz,
    float early_late_space_chips,
    float very_early_late_space_chips,
    size_t port_ch0)
{
    return galileo_e1_tcp_connector_tracking_cc_sptr(new Galileo_E1_Tcp_Connector_Tracking_cc(if_freq,
        fs_in, vector_length, dump, dump_filename, pll_bw_hz, dll_bw_hz, early_late_space_chips, very_early_late_space_chips, port_ch0));
}


void Galileo_E1_Tcp_Connector_Tracking_cc::forecast(int noutput_items,
    gr_vector_int &ninput_items_required)
{
    if (noutput_items != 0)
        {
            ninput_items_required[0] = static_cast<int>(d_vector_length) * 2;  // set the required available samples in each call
        }
}


Galileo_E1_Tcp_Connector_Tracking_cc::Galileo_E1_Tcp_Connector_Tracking_cc(
    long if_freq,
    long fs_in,
    unsigned int vector_length,
    bool dump,
    std::string dump_filename,
    float pll_bw_hz __attribute__((unused)),
    float dll_bw_hz __attribute__((unused)),
    float early_late_space_chips,
    float very_early_late_space_chips,
    size_t port_ch0) : gr::block("Galileo_E1_Tcp_Connector_Tracking_cc", gr::io_signature::make(1, 1, sizeof(gr_complex)),
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

    // Initialize tracking  ==========================================
    //--- DLL variables --------------------------------------------------------
    d_early_late_spc_chips = early_late_space_chips;            // Define early-late offset (in chips)
    d_very_early_late_spc_chips = very_early_late_space_chips;  // Define very-early-late offset (in chips)

    //--- TCP CONNECTOR variables --------------------------------------------------------
    d_port_ch0 = port_ch0;
    d_port = 0;
    d_listen_connection = true;
    d_control_id = 0;

    // Initialization of local code replica
    // Get space for a vector with the sinboc(1,1) replica sampled 2x/chip
    d_ca_code = static_cast<gr_complex *>(volk_gnsssdr_malloc((2 * Galileo_E1_B_CODE_LENGTH_CHIPS) * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    // correlator outputs (scalar)
    d_n_correlator_taps = 5;  // Very-Early, Early, Prompt, Late, Very-Late
    d_correlator_outs = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    for (int n = 0; n < d_n_correlator_taps; n++)
        {
            d_correlator_outs[n] = gr_complex(0, 0);
        }
    // map memory pointers of correlator outputs
    d_Very_Early = &d_correlator_outs[0];
    d_Early = &d_correlator_outs[1];
    d_Prompt = &d_correlator_outs[2];
    d_Late = &d_correlator_outs[3];
    d_Very_Late = &d_correlator_outs[4];

    d_local_code_shift_chips = static_cast<float *>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(float), volk_gnsssdr_get_alignment()));
    // Set TAPs delay values [chips]
    d_local_code_shift_chips[0] = -d_very_early_late_spc_chips;
    d_local_code_shift_chips[1] = -d_early_late_spc_chips;
    d_local_code_shift_chips[2] = 0.0;
    d_local_code_shift_chips[3] = d_early_late_spc_chips;
    d_local_code_shift_chips[4] = d_very_early_late_spc_chips;

    d_correlation_length_samples = d_vector_length;

    multicorrelator_cpu.init(2 * d_correlation_length_samples, d_n_correlator_taps);

    //--- Perform initializations ------------------------------
    // define initial code frequency basis of NCO
    d_code_freq_chips = Galileo_E1_CODE_CHIP_RATE_HZ;
    // define residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    // define residual carrier phase
    d_rem_carr_phase_rad = 0.0;

    // sample synchronization
    d_sample_counter = 0;
    d_acq_sample_stamp = 0;

    d_enable_tracking = false;
    d_pull_in = false;

    d_current_prn_length_samples = static_cast<int>(d_vector_length);

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[FLAGS_cn0_samples];
    d_carrier_lock_test = 1;
    d_CN0_SNV_dB_Hz = 0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_threshold = FLAGS_carrier_lock_th;
    systemName["E"] = std::string("Galileo");

    d_acquisition_gnss_synchro = 0;
    d_channel = 0;
    d_next_rem_code_phase_samples = 0;
    d_acq_code_phase_samples = 0.0;
    d_acq_carrier_doppler_hz = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_acc_code_phase_secs = 0.0;
    d_code_phase_samples = 0;
    d_next_prn_length_samples = 0;
    d_carrier_doppler_hz = 0.0;
}


void Galileo_E1_Tcp_Connector_Tracking_cc::start_tracking()
{
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp = d_acquisition_gnss_synchro->Acq_samplestamp_samples;

    // generate local reference ALWAYS starting at chip 1 (2 samples per chip)
    galileo_e1_code_gen_complex_sampled(d_ca_code,
        d_acquisition_gnss_synchro->Signal,
        false,
        d_acquisition_gnss_synchro->PRN,
        2 * Galileo_E1_CODE_CHIP_RATE_HZ,
        0);

    multicorrelator_cpu.set_local_code_and_taps(static_cast<int>(2 * Galileo_E1_B_CODE_LENGTH_CHIPS), d_ca_code, d_local_code_shift_chips);
    for (int n = 0; n < d_n_correlator_taps; n++)
        {
            d_correlator_outs[n] = gr_complex(0, 0);
        }

    d_carrier_lock_fail_counter = 0;
    d_rem_code_phase_samples = 0.0;
    d_rem_carr_phase_rad = 0;
    d_acc_carrier_phase_rad = 0;

    d_acc_code_phase_secs = 0;
    d_carrier_doppler_hz = d_acq_carrier_doppler_hz;
    d_current_prn_length_samples = d_vector_length;

    std::string sys_ = &d_acquisition_gnss_synchro->System;
    sys = sys_.substr(0, 1);

    // DEBUG OUTPUT
    std::cout << "Tracking of Galileo E1 signal started on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << std::endl;
    LOG(INFO) << "Starting tracking of satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;

    // enable tracking
    d_pull_in = true;
    d_enable_tracking = true;

    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz << " PULL-IN Code Phase [samples]=" << d_acq_code_phase_samples;
}


Galileo_E1_Tcp_Connector_Tracking_cc::~Galileo_E1_Tcp_Connector_Tracking_cc()
{
    if (d_dump_file.is_open())
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
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
            d_tcp_com.close_tcp_connection(d_port);
            multicorrelator_cpu.free();
        }
    catch (const std::exception &ex)
        {
            LOG(WARNING) << "Exception in destructor " << ex.what();
        }
}


int Galileo_E1_Tcp_Connector_Tracking_cc::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    // process vars
    float carr_error_filt_hz;
    float code_error_filt_chips;

    tcp_packet_data tcp_data;
    // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
    Gnss_Synchro current_synchro_data = Gnss_Synchro();
    // Block input data and block output stream pointers
    const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);
    Gnss_Synchro **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);
    if (d_enable_tracking == true)
        {
            // Fill the acquisition data
            current_synchro_data = *d_acquisition_gnss_synchro;
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
                    current_synchro_data.Tracking_sample_counter = d_sample_counter + samples_offset;
                    current_synchro_data.fs = d_fs_in;
                    *out[0] = current_synchro_data;
                    d_sample_counter = d_sample_counter + samples_offset;  //count for the processed samples
                    d_pull_in = false;
                    consume_each(samples_offset);  //shift input to perform alignment with local replica
                    return 1;
                }

            // ################# CARRIER WIPEOFF AND CORRELATORS ##############################
            // perform carrier wipe-off and compute Early, Prompt and Late correlation
            multicorrelator_cpu.set_input_output_vectors(d_correlator_outs, in);

            double carr_phase_step_rad = GALILEO_TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);
            double code_phase_step_half_chips = (2.0 * d_code_freq_chips) / (static_cast<double>(d_fs_in));
            double rem_code_phase_half_chips = d_rem_code_phase_samples * (2.0 * d_code_freq_chips / d_fs_in);
            multicorrelator_cpu.Carrier_wipeoff_multicorrelator_resampler(
                d_rem_carr_phase_rad,
                carr_phase_step_rad,
                rem_code_phase_half_chips,
                code_phase_step_half_chips,
                d_correlation_length_samples);

            // ################## TCP CONNECTOR ##########################################################
            //! Variable used for control
            d_control_id++;

            //! Send and receive a TCP packet
            boost::array<float, NUM_TX_VARIABLES_GALILEO_E1> tx_variables_array = {{d_control_id,
                (*d_Very_Early).real(),
                (*d_Very_Early).imag(),
                (*d_Early).real(),
                (*d_Early).imag(),
                (*d_Late).real(),
                (*d_Late).imag(),
                (*d_Very_Late).real(),
                (*d_Very_Late).imag(),
                (*d_Prompt).real(),
                (*d_Prompt).imag(),
                d_acq_carrier_doppler_hz,
                1}};
            d_tcp_com.send_receive_tcp_packet_galileo_e1(tx_variables_array, &tcp_data);

            // ################## PLL ##########################################################
            // PLL discriminator, carrier loop filter implementation and NCO command generation (TCP_connector)
            carr_error_filt_hz = tcp_data.proc_pack_carr_error;
            // New carrier Doppler frequency estimation
            d_carrier_doppler_hz = d_acq_carrier_doppler_hz + carr_error_filt_hz;
            // New code Doppler frequency estimation
            d_code_freq_chips = Galileo_E1_CODE_CHIP_RATE_HZ + ((d_carrier_doppler_hz * Galileo_E1_CODE_CHIP_RATE_HZ) / Galileo_E1_FREQ_HZ);
            //carrier phase accumulator for (K) doppler estimation
            d_acc_carrier_phase_rad -= GPS_TWO_PI * d_carrier_doppler_hz * Galileo_E1_CODE_PERIOD;
            //remnant carrier phase to prevent overflow in the code NCO
            d_rem_carr_phase_rad = d_rem_carr_phase_rad + GPS_TWO_PI * d_carrier_doppler_hz * Galileo_E1_CODE_PERIOD;
            d_rem_carr_phase_rad = fmod(d_rem_carr_phase_rad, GPS_TWO_PI);

            // ################## DLL ##########################################################
            // DLL discriminator, carrier loop filter implementation and NCO command generation (TCP_connector)
            code_error_filt_chips = tcp_data.proc_pack_code_error;
            //Code phase accumulator
            float code_error_filt_secs;
            code_error_filt_secs = (Galileo_E1_CODE_PERIOD * code_error_filt_chips) / Galileo_E1_CODE_CHIP_RATE_HZ;  //[seconds]
            d_acc_code_phase_secs = d_acc_code_phase_secs + code_error_filt_secs;

            // ################## CARRIER AND CODE NCO BUFFER ALIGNEMENT #######################
            // keep alignment parameters for the next input buffer
            double T_chip_seconds;
            double T_prn_seconds;
            double T_prn_samples;
            double K_blk_samples;
            // Compute the next buffer lenght based in the new period of the PRN sequence and the code phase error estimation
            T_chip_seconds = 1 / static_cast<double>(d_code_freq_chips);
            T_prn_seconds = T_chip_seconds * Galileo_E1_B_CODE_LENGTH_CHIPS;
            T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
            K_blk_samples = T_prn_samples + d_rem_code_phase_samples + code_error_filt_secs * static_cast<double>(d_fs_in);
            d_current_prn_length_samples = round(K_blk_samples);  //round to a discrete samples
            //d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample

            // ####### CN0 ESTIMATION AND LOCK DETECTORS ######
            if (d_cn0_estimation_counter < FLAGS_cn0_samples)
                {
                    // fill buffer with prompt correlator output values
                    d_Prompt_buffer[d_cn0_estimation_counter] = *d_Prompt;
                    d_cn0_estimation_counter++;
                }
            else
                {
                    d_cn0_estimation_counter = 0;

                    // Code lock indicator
                    d_CN0_SNV_dB_Hz = cn0_svn_estimator(d_Prompt_buffer, FLAGS_cn0_samples, d_fs_in, Galileo_E1_B_CODE_LENGTH_CHIPS);

                    // Carrier lock indicator
                    d_carrier_lock_test = carrier_lock_detector(d_Prompt_buffer, FLAGS_cn0_samples);

                    // Loss of lock detection
                    if (d_carrier_lock_test < d_carrier_lock_threshold or d_CN0_SNV_dB_Hz < FLAGS_cn0_min)
                        {
                            d_carrier_lock_fail_counter++;
                        }
                    else
                        {
                            if (d_carrier_lock_fail_counter > 0) d_carrier_lock_fail_counter--;
                        }
                    if (d_carrier_lock_fail_counter > FLAGS_max_lock_fail)
                        {
                            std::cout << "Loss of lock in channel " << d_channel << "!" << std::endl;
                            LOG(INFO) << "Loss of lock in channel " << d_channel << "!";
                            this->message_port_pub(pmt::mp("events"), pmt::from_long(3));  //3 -> loss of lock

                            d_carrier_lock_fail_counter = 0;
                            d_enable_tracking = false;  // TODO: check if disabling tracking is consistent with the channel state machine
                        }
                }

            // ########### Output the tracking data to navigation and PVT ##########
            current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt).real());
            current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt).imag());
            // Tracking_timestamp_secs is aligned with the PRN start sample
            current_synchro_data.Tracking_sample_counter = d_sample_counter + d_current_prn_length_samples;
            current_synchro_data.Code_phase_samples = d_rem_code_phase_samples;
            d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples;  //rounding error < 1 sample
            current_synchro_data.Carrier_phase_rads = static_cast<double>(d_acc_carrier_phase_rad);
            current_synchro_data.Carrier_Doppler_hz = static_cast<double>(d_carrier_doppler_hz);
            current_synchro_data.CN0_dB_hz = static_cast<double>(d_CN0_SNV_dB_Hz);
            current_synchro_data.Flag_valid_symbol_output = true;
            current_synchro_data.correlation_length_ms = 4;
        }
    else
        {
            *d_Early = gr_complex(0, 0);
            *d_Prompt = gr_complex(0, 0);
            *d_Late = gr_complex(0, 0);
            current_synchro_data.Tracking_sample_counter = d_sample_counter + d_current_prn_length_samples;
            //! When tracking is disabled an array of 1's is sent to maintain the TCP connection
            boost::array<float, NUM_TX_VARIABLES_GALILEO_E1> tx_variables_array = {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0}};
            d_tcp_com.send_receive_tcp_packet_galileo_e1(tx_variables_array, &tcp_data);
        }
    //assign the GNURadio block output data
    current_synchro_data.System = {'E'};
    std::string str_aux = "1B";
    const char *str = str_aux.c_str();  // get a C style null terminated string
    std::memcpy(static_cast<void *>(current_synchro_data.Signal), str, 3);

    current_synchro_data.fs = d_fs_in;
    *out[0] = current_synchro_data;

    if (d_dump)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            float prompt_I;
            float prompt_Q;
            float tmp_VE, tmp_E, tmp_P, tmp_L, tmp_VL;
            float tmp_float;
            tmp_float = 0;
            double tmp_double;
            prompt_I = (*d_Prompt).real();
            prompt_Q = (*d_Prompt).imag();
            tmp_VE = std::abs<float>(*d_Very_Early);
            tmp_E = std::abs<float>(*d_Early);
            tmp_P = std::abs<float>(*d_Prompt);
            tmp_L = std::abs<float>(*d_Late);
            tmp_VL = std::abs<float>(*d_Very_Late);

            try
                {
                    // EPR
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_VE), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_E), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_P), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_L), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_VL), sizeof(float));
                    // PROMPT I and Q (to analyze navigation symbols)
                    d_dump_file.write(reinterpret_cast<char *>(&prompt_I), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&prompt_Q), sizeof(float));
                    // PRN start sample stamp
                    d_dump_file.write(reinterpret_cast<char *>(&d_sample_counter), sizeof(unsigned long int));
                    // accumulated carrier phase
                    d_dump_file.write(reinterpret_cast<char *>(&d_acc_carrier_phase_rad), sizeof(float));

                    // carrier and code frequency
                    d_dump_file.write(reinterpret_cast<char *>(&d_carrier_doppler_hz), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&d_code_freq_chips), sizeof(float));

                    //PLL commands
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&carr_error_filt_hz), sizeof(float));

                    //DLL commands
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&code_error_filt_chips), sizeof(float));

                    // CN0 and carrier lock test
                    d_dump_file.write(reinterpret_cast<char *>(&d_CN0_SNV_dB_Hz), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&d_carrier_lock_test), sizeof(float));

                    // AUX vars (for debug purposes)
                    tmp_float = d_rem_code_phase_samples;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_double = static_cast<double>(d_sample_counter + d_current_prn_length_samples);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));

                    // PRN
                    unsigned int prn_ = d_acquisition_gnss_synchro->PRN;
                    d_dump_file.write(reinterpret_cast<char *>(&prn_), sizeof(unsigned int));
                }
            catch (const std::ifstream::failure &e)
                {
                    LOG(WARNING) << "Exception writing trk dump file " << e.what();
                }
        }
    consume_each(d_current_prn_length_samples);        // this is needed in gr::block derivates
    d_sample_counter += d_current_prn_length_samples;  //count for the processed samples

    if (d_enable_tracking)
        {
            return 1;
        }
    else
        {
            return 0;
        }
}


void Galileo_E1_Tcp_Connector_Tracking_cc::set_channel(unsigned int channel)
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
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Tracking dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                        }
                }
        }

    //! Listen for connections on a TCP port
    if (d_listen_connection == true)
        {
            d_port = d_port_ch0 + d_channel;
            d_listen_connection = d_tcp_com.listen_tcp_connection(d_port, d_port_ch0);
        }
}


void Galileo_E1_Tcp_Connector_Tracking_cc::set_gnss_synchro(Gnss_Synchro *p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
}
