/*!
 * \file gps_l1_ca_tcp_connector_tracking_cc.cc
 * \brief Implementation of a TCP connector block based on Code DLL + carrier PLL
 * \author David Pubill, 2012. dpubill(at)cttc.es
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
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

#include "gps_l1_ca_tcp_connector_tracking_cc.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include "gps_sdr_signal_processing.h"
#include "tracking_discriminators.h"
#include "lock_detectors.h"
#include "GPS_L1_CA.h"
#include "control_message_factory.h"
#include "tcp_communication.h"
#include "tcp_packet_data.h"

/*!
 * \todo Include in definition header file
 */
#define CN0_ESTIMATION_SAMPLES 20
#define MINIMUM_VALID_CN0 25
#define MAXIMUM_LOCK_FAIL_COUNTER 50
#define CARRIER_LOCK_THRESHOLD 0.85

using google::LogMessage;

gps_l1_ca_tcp_connector_tracking_cc_sptr
gps_l1_ca_tcp_connector_make_tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float early_late_space_chips,
        size_t port_ch0)
{
    return gps_l1_ca_tcp_connector_tracking_cc_sptr(new Gps_L1_Ca_Tcp_Connector_Tracking_cc(if_freq,
            fs_in, vector_length, dump, dump_filename, early_late_space_chips, port_ch0));
}



void Gps_L1_Ca_Tcp_Connector_Tracking_cc::forecast (int noutput_items,
        gr_vector_int &ninput_items_required)
{
    if (noutput_items != 0)
        {
            ninput_items_required[0] = static_cast<int>(d_vector_length) * 2; //set the required available samples in each call
        }
}



Gps_L1_Ca_Tcp_Connector_Tracking_cc::Gps_L1_Ca_Tcp_Connector_Tracking_cc(
        long if_freq,
        long fs_in,
        unsigned int vector_length,
        bool dump,
        std::string dump_filename,
        float early_late_space_chips,
        size_t port_ch0) :
        gr::block("Gps_L1_Ca_Tcp_Connector_Tracking_cc", gr::io_signature::make(1, 1, sizeof(gr_complex)),
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

    //--- DLL variables --------------------------------------------------------
    d_early_late_spc_chips = early_late_space_chips; // Define early-late offset (in chips)

    //--- TCP CONNECTOR variables --------------------------------------------------------
    d_port_ch0 = port_ch0;
    d_port = 0;
    d_listen_connection = true;
    d_control_id = 0;

    // Initialization of local code replica
    // Get space for a vector with the C/A code replica sampled 1x/chip
    d_ca_code = static_cast<gr_complex*>(volk_gnsssdr_malloc((GPS_L1_CA_CODE_LENGTH_CHIPS) * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    // correlator outputs (scalar)
    d_n_correlator_taps = 3; // Very-Early, Early, Prompt, Late, Very-Late
    d_correlator_outs = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    for (int n = 0; n < d_n_correlator_taps; n++)
       {
          d_correlator_outs[n] = gr_complex(0,0);
       }
    // map memory pointers of correlator outputs
    d_Early = &d_correlator_outs[0];
    d_Prompt = &d_correlator_outs[1];
    d_Late = &d_correlator_outs[2];

    d_local_code_shift_chips = static_cast<float*>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(float), volk_gnsssdr_get_alignment()));
    // Set TAPs delay values [chips]
    d_local_code_shift_chips[0] = - d_early_late_spc_chips;
    d_local_code_shift_chips[1] = 0.0;
    d_local_code_shift_chips[2] = d_early_late_spc_chips;

    d_correlation_length_samples = d_vector_length;

    multicorrelator_cpu.init(2 * d_correlation_length_samples, d_n_correlator_taps);

    //--- Perform initializations ------------------------------
    // define initial code frequency basis of NCO
    d_code_freq_hz = GPS_L1_CA_CODE_RATE_HZ;
    // define residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    // define residual carrier phase
    d_rem_carr_phase_rad = 0.0;

    // sample synchronization
    d_sample_counter = 0;
    d_sample_counter_seconds = 0;
    d_acq_sample_stamp = 0;

    d_enable_tracking = false;
    d_pull_in = false;

    d_current_prn_length_samples = static_cast<int>(d_vector_length);

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[CN0_ESTIMATION_SAMPLES];
    d_carrier_lock_test = 1;
    d_CN0_SNV_dB_Hz = 0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_threshold = CARRIER_LOCK_THRESHOLD;

    systemName["G"] = std::string("GPS");
    systemName["R"] = std::string("GLONASS");
    systemName["S"] = std::string("SBAS");
    systemName["E"] = std::string("Galileo");
    systemName["C"] = std::string("Compass");

    d_acquisition_gnss_synchro = 0;
    d_channel = 0;
    d_next_rem_code_phase_samples = 0;
    d_acq_code_phase_samples = 0.0;
    d_acq_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_code_phase_samples = 0;
    d_next_prn_length_samples = 0;
    d_code_phase_step_chips = 0.0;
}


void Gps_L1_Ca_Tcp_Connector_Tracking_cc::start_tracking()
{
    /*
     *  correct the code phase according to the delay between acq and trk
     */

    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp =  d_acquisition_gnss_synchro->Acq_samplestamp_samples;

    long int acq_trk_diff_samples;
    float acq_trk_diff_seconds;
    acq_trk_diff_samples =  static_cast<long int>(d_sample_counter) -  static_cast<long int>(d_acq_sample_stamp);
    std::cout << "acq_trk_diff_samples=" << acq_trk_diff_samples << std::endl;
    acq_trk_diff_seconds = static_cast<float>(acq_trk_diff_samples) / static_cast<float>(d_fs_in);
    //doppler effect
    // Fd=(C/(C+Vr))*F
    float radial_velocity;
    radial_velocity = (GPS_L1_FREQ_HZ + d_acq_carrier_doppler_hz)/GPS_L1_FREQ_HZ;
    // new chip and prn sequence periods based on acq Doppler
    float T_chip_mod_seconds;
    float T_prn_mod_seconds;
    float T_prn_mod_samples;
    d_code_freq_hz = radial_velocity * GPS_L1_CA_CODE_RATE_HZ;
    d_code_phase_step_chips = static_cast<double>(d_code_freq_hz) / static_cast<double>(d_fs_in);
    T_chip_mod_seconds = 1/d_code_freq_hz;
    T_prn_mod_seconds = T_chip_mod_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
    T_prn_mod_samples = T_prn_mod_seconds * static_cast<float>(d_fs_in);

    d_next_prn_length_samples = round(T_prn_mod_samples);

    float T_prn_true_seconds = GPS_L1_CA_CODE_LENGTH_CHIPS / GPS_L1_CA_CODE_RATE_HZ;
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

    // generate local reference ALWAYS starting at chip 1 (1 sample per chip)
    gps_l1_ca_code_gen_complex(d_ca_code, d_acquisition_gnss_synchro->PRN, 0);

    multicorrelator_cpu.set_local_code_and_taps(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS), d_ca_code, d_local_code_shift_chips);
    for (int n = 0; n < d_n_correlator_taps; n++)
        {
            d_correlator_outs[n] = gr_complex(0,0);
        }

    d_carrier_lock_fail_counter = 0;
    d_rem_code_phase_samples = 0;
    d_rem_carr_phase_rad = 0;
    d_rem_code_phase_samples = 0;
    d_next_rem_code_phase_samples = 0;
    d_acc_carrier_phase_rad = 0;

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


Gps_L1_Ca_Tcp_Connector_Tracking_cc::~Gps_L1_Ca_Tcp_Connector_Tracking_cc()
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
            d_tcp_com.close_tcp_connection(d_port);
            delete[] d_Prompt_buffer;
            multicorrelator_cpu.free();
    }
    catch(const std::exception & ex)
    {
            LOG(WARNING) << "Exception in destructor " << ex.what();
    }
}


int Gps_L1_Ca_Tcp_Connector_Tracking_cc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    // process vars
    float carr_error;
    float carr_nco;
    float code_error;
    float code_nco;

    tcp_packet_data tcp_data;
    // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
    Gnss_Synchro current_synchro_data = Gnss_Synchro();

    // Block input data and block output stream pointers
    const gr_complex* in = reinterpret_cast<const gr_complex *>(input_items[0]);
    Gnss_Synchro **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);

    if (d_enable_tracking == true)
        {
            // Fill the acquisition data
            current_synchro_data = *d_acquisition_gnss_synchro;
            /*
             * Receiver signal alignment
             */
            if (d_pull_in == true)
                {
                    int samples_offset;

                    // 28/11/2011 ACQ to TRK transition BUG CORRECTION
                    float acq_trk_shif_correction_samples;
                    int acq_to_trk_delay_samples;
                    acq_to_trk_delay_samples = d_sample_counter - d_acq_sample_stamp;
                    acq_trk_shif_correction_samples = d_next_prn_length_samples - fmod(static_cast<float>(acq_to_trk_delay_samples), static_cast<float>(d_next_prn_length_samples));
                    samples_offset = round(d_acq_code_phase_samples + acq_trk_shif_correction_samples);
                    current_synchro_data.Tracking_sample_counter = d_sample_counter + samples_offset;
                    current_synchro_data.fs = d_fs_in;
                    *out[0] = current_synchro_data;
                    d_sample_counter_seconds = d_sample_counter_seconds + (static_cast<double>(samples_offset) / static_cast<double>(d_fs_in));
                    d_sample_counter = d_sample_counter + samples_offset; //count for the processed samples
                    d_pull_in = false;
                    consume_each(samples_offset); //shift input to perform alignement with local replica
                    return 1;
                }

            // Update the prn length based on code freq (variable) and
            // sampling frequency (fixed)
            // variable code PRN sample block size
            d_current_prn_length_samples = d_next_prn_length_samples;

            // ################# CARRIER WIPEOFF AND CORRELATORS ##############################
            // perform carrier wipe-off and compute Early, Prompt and Late correlation
            multicorrelator_cpu.set_input_output_vectors(d_correlator_outs,in);

            double carr_phase_step_rad = GPS_TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);
            double rem_code_phase_chips = d_rem_code_phase_samples * (d_code_freq_hz / d_fs_in);

            multicorrelator_cpu.Carrier_wipeoff_multicorrelator_resampler(d_rem_carr_phase_rad,
                    carr_phase_step_rad,
                    rem_code_phase_chips,
                    d_code_phase_step_chips,
                    d_current_prn_length_samples);

            //! Variable used for control
            d_control_id++;

            //! Send and receive a TCP packet
            boost::array<float, NUM_TX_VARIABLES_GPS_L1_CA> tx_variables_array = {{d_control_id,
                                                                                   (*d_Early).real(),
                                                                                   (*d_Early).imag(),
                                                                                   (*d_Late).real(),
                                                                                   (*d_Late).imag(),
                                                                                   (*d_Prompt).real(),
                                                                                   (*d_Prompt).imag(),
                                                                                   d_acq_carrier_doppler_hz,
                                                                                   1}};
            d_tcp_com.send_receive_tcp_packet_gps_l1_ca(tx_variables_array, &tcp_data);

            //! Recover the tracking data
            code_error = tcp_data.proc_pack_code_error;
            carr_error = tcp_data.proc_pack_carr_error;
            // Modify carrier freq based on NCO command
            d_carrier_doppler_hz = tcp_data.proc_pack_carrier_doppler_hz;
            // Modify code freq based on NCO command
            code_nco = 1/(1/GPS_L1_CA_CODE_RATE_HZ - code_error/GPS_L1_CA_CODE_LENGTH_CHIPS);
            d_code_freq_hz = code_nco;

            // Update the phasestep based on code freq (variable) and
            // sampling frequency (fixed)
            d_code_phase_step_chips = d_code_freq_hz / static_cast<float>(d_fs_in); //[chips]
            // variable code PRN sample block size
            double T_chip_seconds;
            double T_prn_seconds;
            double T_prn_samples;
            double K_blk_samples;
            T_chip_seconds = 1.0 / static_cast<double>(d_code_freq_hz);
            T_prn_seconds = T_chip_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
            T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
            d_rem_code_phase_samples = d_next_rem_code_phase_samples;
            K_blk_samples = T_prn_samples + d_rem_code_phase_samples;//-code_error*(double)d_fs_in;

            // Update the current PRN delay (code phase in samples)
            double T_prn_true_seconds = GPS_L1_CA_CODE_LENGTH_CHIPS / GPS_L1_CA_CODE_RATE_HZ;
            double T_prn_true_samples = T_prn_true_seconds * static_cast<double>(d_fs_in);
            d_code_phase_samples = d_code_phase_samples + T_prn_samples - T_prn_true_samples;
            if (d_code_phase_samples < 0)
                {
                    d_code_phase_samples = T_prn_true_samples + d_code_phase_samples;
                }

            d_code_phase_samples = fmod(d_code_phase_samples, T_prn_true_samples);
            d_next_prn_length_samples = round(K_blk_samples); //round to a discrete samples
            d_next_rem_code_phase_samples = K_blk_samples - d_next_prn_length_samples; //rounding error

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
                    d_CN0_SNV_dB_Hz = cn0_svn_estimator(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES, d_fs_in, GPS_L1_CA_CODE_LENGTH_CHIPS);
                    d_carrier_lock_test = carrier_lock_detector(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES);

                    // ###### TRACKING UNLOCK NOTIFICATION #####
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
                            this->message_port_pub(pmt::mp("events"), pmt::from_long(3));//3 -> loss of lock
                            d_carrier_lock_fail_counter = 0;
                            d_enable_tracking = false; // TODO: check if disabling tracking is consistent with the channel state machine

                        }
                }

            // ########### Output the tracking data to navigation and PVT ##########

            current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt).real());
            current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt).imag());
            //compute remnant code phase samples AFTER the Tracking timestamp
            d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample
            current_synchro_data.Tracking_sample_counter = d_sample_counter + d_current_prn_length_samples;
            current_synchro_data.Code_phase_samples = d_rem_code_phase_samples;
            current_synchro_data.Carrier_phase_rads = static_cast<double>(d_acc_carrier_phase_rad);
            current_synchro_data.Carrier_Doppler_hz = static_cast<double>(d_carrier_doppler_hz);
            current_synchro_data.CN0_dB_hz = static_cast<double>(d_CN0_SNV_dB_Hz);
            current_synchro_data.Flag_valid_symbol_output = true;
            current_synchro_data.correlation_length_ms=1;
        }
    else
        {

            *d_Early = gr_complex(0,0);
            *d_Prompt = gr_complex(0,0);
            *d_Late = gr_complex(0,0);
            // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
            current_synchro_data.Tracking_sample_counter = d_sample_counter + d_correlation_length_samples;
            //! When tracking is disabled an array of 1's is sent to maintain the TCP connection
            boost::array<float, NUM_TX_VARIABLES_GPS_L1_CA> tx_variables_array = {{1,1,1,1,1,1,1,1,0}};
            d_tcp_com.send_receive_tcp_packet_gps_l1_ca(tx_variables_array, &tcp_data);
        }

    //assign the GNURadio block output data
    current_synchro_data.System = {'G'};
    std::string str_aux = "1C";
    const char * str = str_aux.c_str(); // get a C style null terminated string
    std::memcpy(static_cast<void*>(current_synchro_data.Signal), str, 3);

    current_synchro_data.fs = d_fs_in;
    *out[0] = current_synchro_data;

    if(d_dump)
        {
            // MULTIPLEXED FILE RECORDING - Record results to file
            float prompt_I;
            float prompt_Q;
            float tmp_E, tmp_P, tmp_L;
            float tmp_float;
            prompt_I = (*d_Prompt).real();
            prompt_Q = (*d_Prompt).imag();
            tmp_E = std::abs<float>(*d_Early);
            tmp_P = std::abs<float>(*d_Prompt);
            tmp_L = std::abs<float>(*d_Late);
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
                    d_dump_file.write(reinterpret_cast<char*>(&d_acc_carrier_phase_rad), sizeof(float));

                    // carrier and code frequency
                    d_dump_file.write(reinterpret_cast<char*>(&d_carrier_doppler_hz), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&d_code_freq_hz), sizeof(float));

                    //PLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&carr_error), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&carr_nco), sizeof(float));

                    //DLL commands
                    d_dump_file.write(reinterpret_cast<char*>(&code_error), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&code_nco), sizeof(float));

                    // CN0 and carrier lock test
                    d_dump_file.write(reinterpret_cast<char*>(&d_CN0_SNV_dB_Hz), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&d_carrier_lock_test), sizeof(float));

                    // AUX vars (for debug purposes)
                    tmp_float = 0;
                    d_dump_file.write(reinterpret_cast<char*>(&tmp_float), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char*>(&d_sample_counter_seconds), sizeof(double));

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
    d_sample_counter_seconds = d_sample_counter_seconds + ( static_cast<double>(d_current_prn_length_samples) / static_cast<double>(d_fs_in) );
    d_sample_counter += d_current_prn_length_samples; //count for the processed samples

    if (d_enable_tracking)
    {
        return 1;
    }else{
        return 0;
    }
}



void Gps_L1_Ca_Tcp_Connector_Tracking_cc::set_channel(unsigned int channel)
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

    //! Listen for connections on a TCP port
    if (d_listen_connection == true)
        {
            d_port = d_port_ch0 + d_channel;
            d_listen_connection = d_tcp_com.listen_tcp_connection(d_port, d_port_ch0);
        }
}


void Gps_L1_Ca_Tcp_Connector_Tracking_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;

}
