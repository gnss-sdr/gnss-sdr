/*!
 * \file dll_pll_veml_tracking.cc
 * \brief Implementation of a code DLL + carrier PLL tracking block.
 * \author Antonio Ramos, 2018 antonio.ramosdet(at)gmail.com
 * 		   Javier Arribas, 2018. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * [1] K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "dll_pll_veml_tracking.h"
#include "tracking_discriminators.h"
#include "lock_detectors.h"
#include "control_message_factory.h"
#include "MATH_CONSTANTS.h"
#include "Galileo_E1.h"
#include "galileo_e1_signal_processing.h"
#include "Galileo_E5a.h"
#include "galileo_e5_signal_processing.h"
#include "GPS_L1_CA.h"
#include "gps_sdr_signal_processing.h"
#include "GPS_L2C.h"
#include "gps_l2c_signal.h"
#include "GPS_L5.h"
#include "gps_l5_signal.h"
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <matio.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

using google::LogMessage;

dll_pll_veml_tracking_sptr dll_pll_veml_make_tracking(const Dll_Pll_Conf &conf_)
{
    return dll_pll_veml_tracking_sptr(new dll_pll_veml_tracking(conf_));
}


void dll_pll_veml_tracking::forecast(int noutput_items,
    gr_vector_int &ninput_items_required)
{
    if (noutput_items != 0)
        {
            ninput_items_required[0] = static_cast<int>(trk_parameters.vector_length) * 2;
        }
}


dll_pll_veml_tracking::dll_pll_veml_tracking(const Dll_Pll_Conf &conf_) : gr::block("dll_pll_veml_tracking", gr::io_signature::make(1, 1, sizeof(gr_complex)),
                                                                              gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    trk_parameters = conf_;
    // Telemetry bit synchronization message port input
    this->message_port_register_out(pmt::mp("events"));
    this->set_relative_rate(1.0 / static_cast<double>(trk_parameters.vector_length));

    // Telemetry bit synchronization message port input (mainly for GPS L1 CA)
    this->message_port_register_in(pmt::mp("preamble_samplestamp"));

    // initialize internal vars
    d_veml = false;
    d_cloop = true;
    d_code_chip_rate = 0.0;
    d_secondary_code_length = 0;
    d_secondary_code_string = nullptr;
    d_gps_l1ca_preambles_symbols = nullptr;
    signal_type = std::string(trk_parameters.signal);

    std::map<std::string, std::string> map_signal_pretty_name;
    map_signal_pretty_name["1C"] = "L1 C/A";
    map_signal_pretty_name["1B"] = "E1";
    map_signal_pretty_name["1G"] = "L1 C/A";
    map_signal_pretty_name["2S"] = "L2C";
    map_signal_pretty_name["2G"] = "L2 C/A";
    map_signal_pretty_name["5X"] = "E5a";
    map_signal_pretty_name["L5"] = "L5";

    signal_pretty_name = map_signal_pretty_name[signal_type];

    if (trk_parameters.system == 'G')
        {
            systemName = "GPS";
            if (signal_type.compare("1C") == 0)
                {
                    d_signal_carrier_freq = GPS_L1_FREQ_HZ;
                    d_code_period = GPS_L1_CA_CODE_PERIOD;
                    d_code_chip_rate = GPS_L1_CA_CODE_RATE_HZ;
                    d_symbols_per_bit = GPS_CA_TELEMETRY_SYMBOLS_PER_BIT;
                    d_correlation_length_ms = 1;
                    d_code_samples_per_chip = 1;
                    d_code_length_chips = static_cast<uint32_t>(GPS_L1_CA_CODE_LENGTH_CHIPS);
                    // GPS L1 C/A does not have pilot component nor secondary code
                    d_secondary = false;
                    trk_parameters.track_pilot = false;
                    interchange_iq = false;

                    // set the preamble
                    uint16_t preambles_bits[GPS_CA_PREAMBLE_LENGTH_BITS] = GPS_PREAMBLE;

                    // preamble bits to sampled symbols
                    d_gps_l1ca_preambles_symbols = static_cast<int32_t *>(volk_gnsssdr_malloc(GPS_CA_PREAMBLE_LENGTH_SYMBOLS * sizeof(int32_t), volk_gnsssdr_get_alignment()));
                    int32_t n = 0;
                    for (int32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_BITS; i++)
                        {
                            for (uint32_t j = 0; j < GPS_CA_TELEMETRY_SYMBOLS_PER_BIT; j++)
                                {
                                    if (preambles_bits[i] == 1)
                                        {
                                            d_gps_l1ca_preambles_symbols[n] = 1;
                                        }
                                    else
                                        {
                                            d_gps_l1ca_preambles_symbols[n] = -1;
                                        }
                                    n++;
                                }
                        }
                    d_symbol_history.resize(GPS_CA_PREAMBLE_LENGTH_SYMBOLS);  // Change fixed buffer size
                    d_symbol_history.clear();                                 // Clear all the elements in the buffer
                }
            else if (signal_type.compare("2S") == 0)
                {
                    d_signal_carrier_freq = GPS_L2_FREQ_HZ;
                    d_code_period = GPS_L2_M_PERIOD;
                    d_code_chip_rate = GPS_L2_M_CODE_RATE_HZ;
                    d_code_length_chips = static_cast<uint32_t>(GPS_L2_M_CODE_LENGTH_CHIPS);
                    d_symbols_per_bit = GPS_L2_SAMPLES_PER_SYMBOL;
                    d_correlation_length_ms = 20;
                    d_code_samples_per_chip = 1;
                    // GPS L2 does not have pilot component nor secondary code
                    d_secondary = false;
                    trk_parameters.track_pilot = false;
                    interchange_iq = false;
                }
            else if (signal_type.compare("L5") == 0)
                {
                    d_signal_carrier_freq = GPS_L5_FREQ_HZ;
                    d_code_period = GPS_L5i_PERIOD;
                    d_code_chip_rate = GPS_L5i_CODE_RATE_HZ;
                    d_symbols_per_bit = GPS_L5_SAMPLES_PER_SYMBOL;
                    d_correlation_length_ms = 1;
                    d_code_samples_per_chip = 1;
                    d_code_length_chips = static_cast<uint32_t>(GPS_L5i_CODE_LENGTH_CHIPS);
                    d_secondary = true;
                    if (trk_parameters.track_pilot)
                        {
                            d_secondary_code_length = static_cast<uint32_t>(GPS_L5q_NH_CODE_LENGTH);
                            d_secondary_code_string = const_cast<std::string *>(&GPS_L5q_NH_CODE_STR);
                            signal_pretty_name = signal_pretty_name + "Q";
                            interchange_iq = true;
                        }
                    else
                        {
                            d_secondary_code_length = static_cast<uint32_t>(GPS_L5i_NH_CODE_LENGTH);
                            d_secondary_code_string = const_cast<std::string *>(&GPS_L5i_NH_CODE_STR);
                            signal_pretty_name = signal_pretty_name + "I";
                            interchange_iq = false;
                        }
                }
            else
                {
                    LOG(WARNING) << "Invalid Signal argument when instantiating tracking blocks";
                    std::cerr << "Invalid Signal argument when instantiating tracking blocks" << std::endl;
                    d_correlation_length_ms = 1;
                    d_secondary = false;
                    interchange_iq = false;
                    d_signal_carrier_freq = 0.0;
                    d_code_period = 0.0;
                    d_code_length_chips = 0;
                    d_code_samples_per_chip = 0;
                    d_symbols_per_bit = 0;
                }
        }
    else if (trk_parameters.system == 'E')
        {
            systemName = "Galileo";
            if (signal_type.compare("1B") == 0)
                {
                    d_signal_carrier_freq = Galileo_E1_FREQ_HZ;
                    d_code_period = Galileo_E1_CODE_PERIOD;
                    d_code_chip_rate = Galileo_E1_CODE_CHIP_RATE_HZ;
                    d_code_length_chips = static_cast<uint32_t>(Galileo_E1_B_CODE_LENGTH_CHIPS);
                    d_symbols_per_bit = 1;
                    d_correlation_length_ms = 4;
                    d_code_samples_per_chip = 2;  // CBOC disabled: 2 samples per chip. CBOC enabled: 12 samples per chip
                    d_veml = true;
                    if (trk_parameters.track_pilot)
                        {
                            d_secondary = true;
                            d_secondary_code_length = static_cast<uint32_t>(Galileo_E1_C_SECONDARY_CODE_LENGTH);
                            d_secondary_code_string = const_cast<std::string *>(&Galileo_E1_C_SECONDARY_CODE);
                            signal_pretty_name = signal_pretty_name + "C";
                        }
                    else
                        {
                            d_secondary = false;
                            signal_pretty_name = signal_pretty_name + "B";
                        }
                    interchange_iq = false;  // Note that E1-B and E1-C are in anti-phase, NOT IN QUADRATURE. See Galileo ICD.
                }
            else if (signal_type.compare("5X") == 0)
                {
                    d_signal_carrier_freq = Galileo_E5a_FREQ_HZ;
                    d_code_period = GALILEO_E5a_CODE_PERIOD;
                    d_code_chip_rate = Galileo_E5a_CODE_CHIP_RATE_HZ;
                    d_symbols_per_bit = 20;
                    d_correlation_length_ms = 1;
                    d_code_samples_per_chip = 1;
                    d_code_length_chips = static_cast<uint32_t>(Galileo_E5a_CODE_LENGTH_CHIPS);
                    d_secondary = true;
                    if (trk_parameters.track_pilot)
                        {
                            d_secondary_code_length = static_cast<uint32_t>(Galileo_E5a_Q_SECONDARY_CODE_LENGTH);
                            signal_pretty_name = signal_pretty_name + "Q";
                            interchange_iq = true;
                        }
                    else
                        {
                            d_secondary_code_length = static_cast<uint32_t>(Galileo_E5a_I_SECONDARY_CODE_LENGTH);
                            d_secondary_code_string = const_cast<std::string *>(&Galileo_E5a_I_SECONDARY_CODE);
                            signal_pretty_name = signal_pretty_name + "I";
                            interchange_iq = false;
                        }
                }
            else
                {
                    LOG(WARNING) << "Invalid Signal argument when instantiating tracking blocks";
                    std::cout << "Invalid Signal argument when instantiating tracking blocks" << std::endl;
                    d_correlation_length_ms = 1;
                    d_secondary = false;
                    interchange_iq = false;
                    d_signal_carrier_freq = 0.0;
                    d_code_period = 0.0;
                    d_code_length_chips = 0;
                    d_code_samples_per_chip = 0;
                    d_symbols_per_bit = 0;
                }
        }
    else
        {
            LOG(WARNING) << "Invalid System argument when instantiating tracking blocks";
            std::cerr << "Invalid System argument when instantiating tracking blocks" << std::endl;
            d_correlation_length_ms = 1;
            d_secondary = false;
            interchange_iq = false;
            d_signal_carrier_freq = 0.0;
            d_code_period = 0.0;
            d_code_length_chips = 0;
            d_code_samples_per_chip = 0;
            d_symbols_per_bit = 0;
        }
    T_chip_seconds = 0.0;
    T_prn_seconds = 0.0;
    T_prn_samples = 0.0;
    K_blk_samples = 0.0;

    // Initialize tracking  ==========================================

    d_code_loop_filter.set_DLL_BW(trk_parameters.dll_bw_hz);
    d_carrier_loop_filter.set_PLL_BW(trk_parameters.pll_bw_hz);
    d_code_loop_filter = Tracking_2nd_DLL_filter(static_cast<float>(d_code_period));
    d_carrier_loop_filter = Tracking_2nd_PLL_filter(static_cast<float>(d_code_period));

    // Initialization of local code replica
    // Get space for a vector with the sinboc(1,1) replica sampled 2x/chip
    d_tracking_code = static_cast<float *>(volk_gnsssdr_malloc(2 * d_code_length_chips * sizeof(float), volk_gnsssdr_get_alignment()));
    // correlator outputs (scalar)
    if (d_veml)
        {
            // Very-Early, Early, Prompt, Late, Very-Late
            d_n_correlator_taps = 5;
        }
    else
        {
            // Early, Prompt, Late
            d_n_correlator_taps = 3;
        }

    d_correlator_outs = static_cast<gr_complex *>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_local_code_shift_chips = static_cast<float *>(volk_gnsssdr_malloc(d_n_correlator_taps * sizeof(float), volk_gnsssdr_get_alignment()));


    // map memory pointers of correlator outputs
    if (d_veml)
        {
            d_Very_Early = &d_correlator_outs[0];
            d_Early = &d_correlator_outs[1];
            d_Prompt = &d_correlator_outs[2];
            d_Late = &d_correlator_outs[3];
            d_Very_Late = &d_correlator_outs[4];
            d_local_code_shift_chips[0] = -trk_parameters.very_early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
            d_local_code_shift_chips[1] = -trk_parameters.early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
            d_local_code_shift_chips[2] = 0.0;
            d_local_code_shift_chips[3] = trk_parameters.early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
            d_local_code_shift_chips[4] = trk_parameters.very_early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
            d_prompt_data_shift = &d_local_code_shift_chips[2];
        }
    else
        {
            d_Very_Early = nullptr;
            d_Early = &d_correlator_outs[0];
            d_Prompt = &d_correlator_outs[1];
            d_Late = &d_correlator_outs[2];
            d_Very_Late = nullptr;
            d_local_code_shift_chips[0] = -trk_parameters.early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
            d_local_code_shift_chips[1] = 0.0;
            d_local_code_shift_chips[2] = trk_parameters.early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
            d_prompt_data_shift = &d_local_code_shift_chips[1];
        }

    multicorrelator_cpu.init(2 * trk_parameters.vector_length, d_n_correlator_taps);

    if (trk_parameters.extend_correlation_symbols > 1)
        {
            d_enable_extended_integration = true;
        }
    else
        {
            d_enable_extended_integration = false;
            trk_parameters.extend_correlation_symbols = 1;
        }

    // Enable Data component prompt correlator (slave to Pilot prompt) if tracking uses Pilot signal
    if (trk_parameters.track_pilot)
        {
            // Extra correlator for the data component
            correlator_data_cpu.init(2 * trk_parameters.vector_length, 1);
            d_data_code = static_cast<float *>(volk_gnsssdr_malloc(2 * d_code_length_chips * sizeof(float), volk_gnsssdr_get_alignment()));
        }
    else
        {
            d_data_code = nullptr;
        }

    // --- Initializations ---
    multicorrelator_cpu.set_fast_resampler(trk_parameters.use_fast_resampler);
    // Initial code frequency basis of NCO
    d_code_freq_chips = d_code_chip_rate;
    // Residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    // Residual carrier phase
    d_rem_carr_phase_rad = 0.0;

    // sample synchronization
    d_sample_counter = 0;
    d_acq_sample_stamp = 0;

    d_current_prn_length_samples = static_cast<int>(trk_parameters.vector_length);

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[trk_parameters.cn0_samples];
    d_carrier_lock_test = 1.0;
    d_CN0_SNV_dB_Hz = 0.0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_threshold = trk_parameters.carrier_lock_th;
    d_Prompt_Data = static_cast<gr_complex *>(volk_gnsssdr_malloc(sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    d_acquisition_gnss_synchro = nullptr;
    d_channel = 0;
    d_acq_code_phase_samples = 0.0;
    d_acq_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz = 0.0;
    d_acc_carrier_phase_rad = 0.0;

    d_extend_correlation_symbols_count = 0;
    d_code_phase_step_chips = 0.0;
    d_carrier_phase_step_rad = 0.0;
    d_rem_code_phase_chips = 0.0;
    d_K_blk_samples = 0.0;
    d_code_phase_samples = 0.0;
    d_last_prompt = gr_complex(0.0, 0.0);
    d_state = 0;  // initial state: standby
    clear_tracking_vars();
}


void dll_pll_veml_tracking::start_tracking()
{
    gr::thread::scoped_lock l(d_setlock);

    //  correct the code phase according to the delay between acq and trk
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp = d_acquisition_gnss_synchro->Acq_samplestamp_samples;

    int64_t acq_trk_diff_samples = static_cast<int64_t>(d_sample_counter) - static_cast<int64_t>(d_acq_sample_stamp);
    double acq_trk_diff_seconds = static_cast<double>(acq_trk_diff_samples) / trk_parameters.fs_in;
    DLOG(INFO) << "Number of samples between Acquisition and Tracking = " << acq_trk_diff_samples;
    DLOG(INFO) << "Number of seconds between Acquisition and Tracking = " << acq_trk_diff_seconds;
    // Doppler effect Fd = (C / (C + Vr)) * F
    double radial_velocity = (d_signal_carrier_freq + d_acq_carrier_doppler_hz) / d_signal_carrier_freq;
    // new chip and PRN sequence periods based on acq Doppler
    d_code_freq_chips = radial_velocity * d_code_chip_rate;
    d_code_phase_step_chips = d_code_freq_chips / trk_parameters.fs_in;
    double T_chip_mod_seconds = 1.0 / d_code_freq_chips;
    double T_prn_mod_seconds = T_chip_mod_seconds * static_cast<double>(d_code_length_chips);
    double T_prn_mod_samples = T_prn_mod_seconds * trk_parameters.fs_in;

    //d_current_prn_length_samples = std::round(T_prn_mod_samples);
    d_current_prn_length_samples = std::floor(T_prn_mod_samples);

    double T_prn_true_seconds = static_cast<double>(d_code_length_chips) / d_code_chip_rate;
    double T_prn_true_samples = T_prn_true_seconds * trk_parameters.fs_in;
    double T_prn_diff_seconds = T_prn_true_seconds - T_prn_mod_seconds;
    double N_prn_diff = acq_trk_diff_seconds / T_prn_true_seconds;
    double corrected_acq_phase_samples = std::fmod(d_acq_code_phase_samples + T_prn_diff_seconds * N_prn_diff * trk_parameters.fs_in, T_prn_true_samples);
    if (corrected_acq_phase_samples < 0.0)
        {
            corrected_acq_phase_samples += T_prn_mod_samples;
        }
    double delay_correction_samples = d_acq_code_phase_samples - corrected_acq_phase_samples;

    d_acq_code_phase_samples = corrected_acq_phase_samples;

    d_carrier_doppler_hz = d_acq_carrier_doppler_hz;
    d_carrier_phase_step_rad = PI_2 * d_carrier_doppler_hz / trk_parameters.fs_in;

    // DLL/PLL filter initialization
    d_carrier_loop_filter.initialize();  // initialize the carrier filter
    d_code_loop_filter.initialize();     // initialize the code filter

    if (systemName.compare("GPS") == 0 and signal_type.compare("1C") == 0)
        {
            gps_l1_ca_code_gen_float(d_tracking_code, d_acquisition_gnss_synchro->PRN, 0);
        }
    else if (systemName.compare("GPS") == 0 and signal_type.compare("2S") == 0)
        {
            gps_l2c_m_code_gen_float(d_tracking_code, d_acquisition_gnss_synchro->PRN);
        }
    else if (systemName.compare("GPS") == 0 and signal_type.compare("L5") == 0)
        {
            if (trk_parameters.track_pilot)
                {
                    gps_l5q_code_gen_float(d_tracking_code, d_acquisition_gnss_synchro->PRN);
                    gps_l5i_code_gen_float(d_data_code, d_acquisition_gnss_synchro->PRN);
                    d_Prompt_Data[0] = gr_complex(0.0, 0.0);
                    correlator_data_cpu.set_local_code_and_taps(d_code_length_chips, d_data_code, d_prompt_data_shift);
                }
            else
                {
                    gps_l5i_code_gen_float(d_tracking_code, d_acquisition_gnss_synchro->PRN);
                }
        }
    else if (systemName.compare("Galileo") == 0 and signal_type.compare("1B") == 0)
        {
            if (trk_parameters.track_pilot)
                {
                    char pilot_signal[3] = "1C";
                    galileo_e1_code_gen_sinboc11_float(d_tracking_code, pilot_signal, d_acquisition_gnss_synchro->PRN);
                    galileo_e1_code_gen_sinboc11_float(d_data_code, d_acquisition_gnss_synchro->Signal, d_acquisition_gnss_synchro->PRN);
                    d_Prompt_Data[0] = gr_complex(0.0, 0.0);
                    correlator_data_cpu.set_local_code_and_taps(d_code_samples_per_chip * d_code_length_chips, d_data_code, d_prompt_data_shift);
                }
            else
                {
                    galileo_e1_code_gen_sinboc11_float(d_tracking_code, d_acquisition_gnss_synchro->Signal, d_acquisition_gnss_synchro->PRN);
                }
        }
    else if (systemName.compare("Galileo") == 0 and signal_type.compare("5X") == 0)
        {
            gr_complex *aux_code = static_cast<gr_complex *>(volk_gnsssdr_malloc(sizeof(gr_complex) * d_code_length_chips, volk_gnsssdr_get_alignment()));
            galileo_e5_a_code_gen_complex_primary(aux_code, d_acquisition_gnss_synchro->PRN, const_cast<char *>(signal_type.c_str()));
            if (trk_parameters.track_pilot)
                {
                    d_secondary_code_string = const_cast<std::string *>(&Galileo_E5a_Q_SECONDARY_CODE[d_acquisition_gnss_synchro->PRN - 1]);
                    for (uint32_t i = 0; i < d_code_length_chips; i++)
                        {
                            d_tracking_code[i] = aux_code[i].imag();
                            d_data_code[i] = aux_code[i].real();
                        }
                    d_Prompt_Data[0] = gr_complex(0.0, 0.0);
                    correlator_data_cpu.set_local_code_and_taps(d_code_length_chips, d_data_code, d_prompt_data_shift);
                }
            else
                {
                    for (uint32_t i = 0; i < d_code_length_chips; i++)
                        {
                            d_tracking_code[i] = aux_code[i].real();
                        }
                }
            volk_gnsssdr_free(aux_code);
        }

    multicorrelator_cpu.set_local_code_and_taps(d_code_samples_per_chip * d_code_length_chips, d_tracking_code, d_local_code_shift_chips);
    std::fill_n(d_correlator_outs, d_n_correlator_taps, gr_complex(0.0, 0.0));

    d_carrier_lock_fail_counter = 0;
    d_rem_code_phase_samples = 0.0;
    d_rem_carr_phase_rad = 0.0;
    d_rem_code_phase_chips = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_cn0_estimation_counter = 0;
    d_carrier_lock_test = 1.0;
    d_CN0_SNV_dB_Hz = 0.0;

    if (d_veml)
        {
            d_local_code_shift_chips[0] = -trk_parameters.very_early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
            d_local_code_shift_chips[1] = -trk_parameters.early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
            d_local_code_shift_chips[3] = trk_parameters.early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
            d_local_code_shift_chips[4] = trk_parameters.very_early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
        }
    else
        {
            d_local_code_shift_chips[0] = -trk_parameters.early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
            d_local_code_shift_chips[2] = trk_parameters.early_late_space_chips * static_cast<float>(d_code_samples_per_chip);
        }

    d_code_phase_samples = d_acq_code_phase_samples;
    d_code_loop_filter.set_DLL_BW(trk_parameters.dll_bw_hz);
    d_carrier_loop_filter.set_PLL_BW(trk_parameters.pll_bw_hz);
    d_carrier_loop_filter.set_pdi(static_cast<float>(d_code_period));
    d_code_loop_filter.set_pdi(static_cast<float>(d_code_period));

    // DEBUG OUTPUT
    std::cout << "Tracking of " << systemName << " " << signal_pretty_name << " signal started on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName, d_acquisition_gnss_synchro->PRN) << std::endl;
    LOG(INFO) << "Starting tracking of satellite " << Gnss_Satellite(systemName, d_acquisition_gnss_synchro->PRN) << " on channel " << d_channel;

    // enable tracking pull-in
    d_state = 1;
    d_cloop = true;
    d_Prompt_buffer_deque.clear();
    d_last_prompt = gr_complex(0.0, 0.0);
    LOG(INFO) << "PULL-IN Doppler [Hz] = " << d_carrier_doppler_hz
              << ". Code Phase correction [samples] = " << delay_correction_samples
              << ". PULL-IN Code Phase [samples] = " << d_acq_code_phase_samples;
}


dll_pll_veml_tracking::~dll_pll_veml_tracking()
{
    if (signal_type.compare("1C") == 0)
        {
            volk_gnsssdr_free(d_gps_l1ca_preambles_symbols);
        }

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
    if (trk_parameters.dump)
        {
            if (d_channel == 0)
                {
                    std::cout << "Writing .mat files ...";
                }
            save_matfile();
            if (d_channel == 0)
                {
                    std::cout << " done." << std::endl;
                }
        }
    try
        {
            volk_gnsssdr_free(d_local_code_shift_chips);
            volk_gnsssdr_free(d_correlator_outs);
            volk_gnsssdr_free(d_tracking_code);
            volk_gnsssdr_free(d_Prompt_Data);
            if (trk_parameters.track_pilot)
                {
                    volk_gnsssdr_free(d_data_code);
                    correlator_data_cpu.free();
                }
            delete[] d_Prompt_buffer;
            multicorrelator_cpu.free();
        }
    catch (const std::exception &ex)
        {
            LOG(WARNING) << "Exception in destructor " << ex.what();
        }
}


bool dll_pll_veml_tracking::acquire_secondary()
{
    // ******* preamble correlation ********
    int32_t corr_value = 0;
    for (uint32_t i = 0; i < d_secondary_code_length; i++)
        {
            if (d_Prompt_buffer_deque.at(i).real() < 0.0)  // symbols clipping
                {
                    if (d_secondary_code_string->at(i) == '0')
                        {
                            corr_value++;
                        }
                    else
                        {
                            corr_value--;
                        }
                }
            else
                {
                    if (d_secondary_code_string->at(i) == '0')
                        {
                            corr_value--;
                        }
                    else
                        {
                            corr_value++;
                        }
                }
        }

    if (abs(corr_value) == static_cast<int>(d_secondary_code_length))
        {
            return true;
        }
    else
        {
            return false;
        }
}


bool dll_pll_veml_tracking::cn0_and_tracking_lock_status(double coh_integration_time_s)
{
    // ####### CN0 ESTIMATION AND LOCK DETECTORS ######

    if (d_cn0_estimation_counter < trk_parameters.cn0_samples)
        {
            // fill buffer with prompt correlator output values
            d_Prompt_buffer[d_cn0_estimation_counter] = d_P_accu;
            d_cn0_estimation_counter++;
            return true;
        }
    else
        {
            d_cn0_estimation_counter = 0;
            // Code lock indicator
            d_CN0_SNV_dB_Hz = cn0_svn_estimator(d_Prompt_buffer, trk_parameters.cn0_samples, coh_integration_time_s);
            // Carrier lock indicator
            d_carrier_lock_test = carrier_lock_detector(d_Prompt_buffer, trk_parameters.cn0_samples);
            // Loss of lock detection
            if (d_carrier_lock_test < d_carrier_lock_threshold or d_CN0_SNV_dB_Hz < trk_parameters.cn0_min)
                {
                    d_carrier_lock_fail_counter++;
                }
            else
                {
                    if (d_carrier_lock_fail_counter > 0) d_carrier_lock_fail_counter--;
                }
            if (d_carrier_lock_fail_counter > trk_parameters.max_lock_fail)
                {
                    std::cout << "Loss of lock in channel " << d_channel << "!" << std::endl;
                    LOG(INFO) << "Loss of lock in channel " << d_channel << "!";
                    this->message_port_pub(pmt::mp("events"), pmt::from_long(3));  // 3 -> loss of lock
                    d_carrier_lock_fail_counter = 0;
                    return false;
                }
            else
                {
                    return true;
                }
        }
}


// correlation requires:
// - updated remnant carrier phase in radians (rem_carr_phase_rad)
// - updated remnant code phase in samples (d_rem_code_phase_samples)
// - d_code_freq_chips
// - d_carrier_doppler_hz
void dll_pll_veml_tracking::do_correlation_step(const gr_complex *input_samples)
{
    // ################# CARRIER WIPEOFF AND CORRELATORS ##############################
    // perform carrier wipe-off and compute Early, Prompt and Late correlation
    multicorrelator_cpu.set_input_output_vectors(d_correlator_outs, input_samples);
    multicorrelator_cpu.Carrier_wipeoff_multicorrelator_resampler(
        d_rem_carr_phase_rad,
        d_carrier_phase_step_rad,
        static_cast<float>(d_rem_code_phase_chips) * static_cast<float>(d_code_samples_per_chip),
        static_cast<float>(d_code_phase_step_chips) * static_cast<float>(d_code_samples_per_chip),
        trk_parameters.vector_length);

    // DATA CORRELATOR (if tracking tracks the pilot signal)
    if (trk_parameters.track_pilot)
        {
            correlator_data_cpu.set_input_output_vectors(d_Prompt_Data, input_samples);
            correlator_data_cpu.Carrier_wipeoff_multicorrelator_resampler(
                d_rem_carr_phase_rad,
                d_carrier_phase_step_rad,
                static_cast<float>(d_rem_code_phase_chips) * static_cast<float>(d_code_samples_per_chip),
                static_cast<float>(d_code_phase_step_chips) * static_cast<float>(d_code_samples_per_chip),
                trk_parameters.vector_length);
        }
}


void dll_pll_veml_tracking::run_dll_pll()
{
    // ################## PLL ##########################################################
    // PLL discriminator
    if (d_cloop)
        {
            // Costas loop discriminator, insensitive to 180 deg phase transitions
            d_carr_error_hz = pll_cloop_two_quadrant_atan(d_P_accu) / PI_2;
        }
    else
        {
            // Secondary code acquired. No symbols transition should be present in the signal
            d_carr_error_hz = pll_four_quadrant_atan(d_P_accu) / PI_2;
        }

    // Carrier discriminator filter
    d_carr_error_filt_hz = d_carrier_loop_filter.get_carrier_nco(d_carr_error_hz);
    // New carrier Doppler frequency estimation
    d_carrier_doppler_hz = d_acq_carrier_doppler_hz + d_carr_error_filt_hz;


    // ################## DLL ##########################################################
    // DLL discriminator
    if (d_veml)
        {
            d_code_error_chips = dll_nc_vemlp_normalized(d_VE_accu, d_E_accu, d_L_accu, d_VL_accu);  // [chips/Ti]
        }
    else
        {
            d_code_error_chips = dll_nc_e_minus_l_normalized(d_E_accu, d_L_accu);  // [chips/Ti]
        }
    // Code discriminator filter
    d_code_error_filt_chips = d_code_loop_filter.get_code_nco(d_code_error_chips);  // [chips/second]

    // New code Doppler frequency estimation
    d_code_freq_chips = (1.0 + (d_carrier_doppler_hz / d_signal_carrier_freq)) * d_code_chip_rate - d_code_error_filt_chips;
}


void dll_pll_veml_tracking::clear_tracking_vars()
{
    std::fill_n(d_correlator_outs, d_n_correlator_taps, gr_complex(0.0, 0.0));
    if (trk_parameters.track_pilot) d_Prompt_Data[0] = gr_complex(0.0, 0.0);
    d_carr_error_hz = 0.0;
    d_carr_error_filt_hz = 0.0;
    d_code_error_chips = 0.0;
    d_code_error_filt_chips = 0.0;
    d_current_symbol = 0;
    d_Prompt_buffer_deque.clear();
    d_last_prompt = gr_complex(0.0, 0.0);
}


void dll_pll_veml_tracking::update_tracking_vars()
{
    T_chip_seconds = 1.0 / d_code_freq_chips;
    T_prn_seconds = T_chip_seconds * static_cast<double>(d_code_length_chips);

    // ################## CARRIER AND CODE NCO BUFFER ALIGNMENT #######################
    // keep alignment parameters for the next input buffer
    // Compute the next buffer length based in the new period of the PRN sequence and the code phase error estimation
    T_prn_samples = T_prn_seconds * trk_parameters.fs_in;
    K_blk_samples = T_prn_samples + d_rem_code_phase_samples;
    //d_current_prn_length_samples = static_cast<int>(round(K_blk_samples));  // round to a discrete number of samples
    d_current_prn_length_samples = static_cast<int>(std::floor(K_blk_samples));  // round to a discrete number of samples

    //################### PLL COMMANDS #################################################
    // carrier phase step (NCO phase increment per sample) [rads/sample]
    d_carrier_phase_step_rad = PI_2 * d_carrier_doppler_hz / trk_parameters.fs_in;
    // remnant carrier phase to prevent overflow in the code NCO
    d_rem_carr_phase_rad += d_carrier_phase_step_rad * static_cast<double>(d_current_prn_length_samples);
    d_rem_carr_phase_rad = fmod(d_rem_carr_phase_rad, PI_2);
    // carrier phase accumulator
    d_acc_carrier_phase_rad -= d_carrier_phase_step_rad * static_cast<double>(d_current_prn_length_samples);

    //################### DLL COMMANDS #################################################
    // code phase step (Code resampler phase increment per sample) [chips/sample]
    d_code_phase_step_chips = d_code_freq_chips / trk_parameters.fs_in;
    // remnant code phase [chips]
    d_rem_code_phase_samples = K_blk_samples - static_cast<double>(d_current_prn_length_samples);  // rounding error < 1 sample
    d_rem_code_phase_chips = d_code_freq_chips * d_rem_code_phase_samples / trk_parameters.fs_in;
}


void dll_pll_veml_tracking::save_correlation_results()
{
    if (d_secondary)
        {
            if (d_secondary_code_string->at(d_current_symbol) == '0')
                {
                    if (d_veml)
                        {
                            d_VE_accu += *d_Very_Early;
                            d_VL_accu += *d_Very_Late;
                        }
                    d_E_accu += *d_Early;
                    d_P_accu += *d_Prompt;
                    d_L_accu += *d_Late;
                }
            else
                {
                    if (d_veml)
                        {
                            d_VE_accu -= *d_Very_Early;
                            d_VL_accu -= *d_Very_Late;
                        }
                    d_E_accu -= *d_Early;
                    d_P_accu -= *d_Prompt;
                    d_L_accu -= *d_Late;
                }
            d_current_symbol++;
            // secondary code roll-up
            d_current_symbol %= d_secondary_code_length;
        }
    else
        {
            if (d_veml)
                {
                    d_VE_accu += *d_Very_Early;
                    d_VL_accu += *d_Very_Late;
                }
            d_E_accu += *d_Early;
            d_P_accu += *d_Prompt;
            d_L_accu += *d_Late;
            d_current_symbol++;
            d_current_symbol %= d_symbols_per_bit;
        }
    // If tracking pilot, disable Costas loop
    if (trk_parameters.track_pilot)
        d_cloop = false;
    else
        d_cloop = true;
}


void dll_pll_veml_tracking::log_data(bool integrating)
{
    if (trk_parameters.dump)
        {
            // Dump results to file
            float prompt_I;
            float prompt_Q;
            float tmp_VE, tmp_E, tmp_P, tmp_L, tmp_VL;
            float tmp_float;
            double tmp_double;
            uint64_t tmp_long_int;
            if (trk_parameters.track_pilot)
                {
                    if (interchange_iq)
                        {
                            prompt_I = d_Prompt_Data->imag();
                            prompt_Q = d_Prompt_Data->real();
                        }
                    else
                        {
                            prompt_I = d_Prompt_Data->real();
                            prompt_Q = d_Prompt_Data->imag();
                        }
                }
            else
                {
                    if (interchange_iq)
                        {
                            prompt_I = d_Prompt->imag();
                            prompt_Q = d_Prompt->real();
                        }
                    else
                        {
                            prompt_I = d_Prompt->real();
                            prompt_Q = d_Prompt->imag();
                        }
                }
            if (d_veml)
                {
                    tmp_VE = std::abs<float>(d_VE_accu);
                    tmp_VL = std::abs<float>(d_VL_accu);
                }
            else
                {
                    tmp_VE = 0.0;
                    tmp_VL = 0.0;
                }
            tmp_E = std::abs<float>(d_E_accu);
            tmp_P = std::abs<float>(d_P_accu);
            tmp_L = std::abs<float>(d_L_accu);
            if (integrating)
                {
                    //TODO: Improve this solution!
                    // It compensates the amplitude difference while integrating
                    if (d_extend_correlation_symbols_count > 0)
                        {
                            float scale_factor = static_cast<float>(trk_parameters.extend_correlation_symbols) / static_cast<float>(d_extend_correlation_symbols_count);
                            tmp_VE *= scale_factor;
                            tmp_E *= scale_factor;
                            tmp_P *= scale_factor;
                            tmp_L *= scale_factor;
                            tmp_VL *= scale_factor;
                        }
                }

            try
                {
                    // Dump correlators output
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_VE), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_E), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_P), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_L), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_VL), sizeof(float));
                    // PROMPT I and Q (to analyze navigation symbols)
                    d_dump_file.write(reinterpret_cast<char *>(&prompt_I), sizeof(float));
                    d_dump_file.write(reinterpret_cast<char *>(&prompt_Q), sizeof(float));
                    // PRN start sample stamp
                    tmp_long_int = d_sample_counter + d_current_prn_length_samples;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_long_int), sizeof(uint64_t));
                    // accumulated carrier phase
                    tmp_float = d_acc_carrier_phase_rad;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // carrier and code frequency
                    tmp_float = d_carrier_doppler_hz;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = d_code_freq_chips;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // PLL commands
                    tmp_float = d_carr_error_hz;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = d_carr_error_filt_hz;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // DLL commands
                    tmp_float = d_code_error_chips;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = d_code_error_filt_chips;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // CN0 and carrier lock test
                    tmp_float = d_CN0_SNV_dB_Hz;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_float = d_carrier_lock_test;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    // AUX vars (for debug purposes)
                    tmp_float = d_rem_code_phase_samples;
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_float), sizeof(float));
                    tmp_double = static_cast<double>(d_sample_counter + d_current_prn_length_samples);
                    d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                    // PRN
                    uint32_t prn_ = d_acquisition_gnss_synchro->PRN;
                    d_dump_file.write(reinterpret_cast<char *>(&prn_), sizeof(uint32_t));
                }
            catch (const std::ifstream::failure &e)
                {
                    LOG(WARNING) << "Exception writing trk dump file " << e.what();
                }
        }
}


int32_t dll_pll_veml_tracking::save_matfile()
{
    // READ DUMP FILE
    std::ifstream::pos_type size;
    int32_t number_of_double_vars = 1;
    int32_t number_of_float_vars = 17;
    int32_t epoch_size_bytes = sizeof(uint64_t) + sizeof(double) * number_of_double_vars +
                               sizeof(float) * number_of_float_vars + sizeof(uint32_t);
    std::ifstream dump_file;
    dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try
        {
            dump_file.open(trk_parameters.dump_filename.c_str(), std::ios::binary | std::ios::ate);
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem opening dump file:" << e.what() << std::endl;
            return 1;
        }
    // count number of epochs and rewind
    int64_t num_epoch = 0;
    if (dump_file.is_open())
        {
            size = dump_file.tellg();
            num_epoch = static_cast<int64_t>(size) / static_cast<int64_t>(epoch_size_bytes);
            dump_file.seekg(0, std::ios::beg);
        }
    else
        {
            return 1;
        }
    float *abs_VE = new float[num_epoch];
    float *abs_E = new float[num_epoch];
    float *abs_P = new float[num_epoch];
    float *abs_L = new float[num_epoch];
    float *abs_VL = new float[num_epoch];
    float *Prompt_I = new float[num_epoch];
    float *Prompt_Q = new float[num_epoch];
    uint64_t *PRN_start_sample_count = new uint64_t[num_epoch];
    float *acc_carrier_phase_rad = new float[num_epoch];
    float *carrier_doppler_hz = new float[num_epoch];
    float *code_freq_chips = new float[num_epoch];
    float *carr_error_hz = new float[num_epoch];
    float *carr_error_filt_hz = new float[num_epoch];
    float *code_error_chips = new float[num_epoch];
    float *code_error_filt_chips = new float[num_epoch];
    float *CN0_SNV_dB_Hz = new float[num_epoch];
    float *carrier_lock_test = new float[num_epoch];
    float *aux1 = new float[num_epoch];
    double *aux2 = new double[num_epoch];
    uint32_t *PRN = new uint32_t[num_epoch];

    try
        {
            if (dump_file.is_open())
                {
                    for (int64_t i = 0; i < num_epoch; i++)
                        {
                            dump_file.read(reinterpret_cast<char *>(&abs_VE[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_E[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_P[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_L[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&abs_VL[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&Prompt_I[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&Prompt_Q[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&PRN_start_sample_count[i]), sizeof(uint64_t));
                            dump_file.read(reinterpret_cast<char *>(&acc_carrier_phase_rad[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&carrier_doppler_hz[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&code_freq_chips[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&carr_error_hz[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&carr_error_filt_hz[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&code_error_chips[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&code_error_filt_chips[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&CN0_SNV_dB_Hz[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&carrier_lock_test[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&aux1[i]), sizeof(float));
                            dump_file.read(reinterpret_cast<char *>(&aux2[i]), sizeof(double));
                            dump_file.read(reinterpret_cast<char *>(&PRN[i]), sizeof(uint32_t));
                        }
                }
            dump_file.close();
        }
    catch (const std::ifstream::failure &e)
        {
            std::cerr << "Problem reading dump file:" << e.what() << std::endl;
            delete[] abs_VE;
            delete[] abs_E;
            delete[] abs_P;
            delete[] abs_L;
            delete[] abs_VL;
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
    std::string filename = trk_parameters.dump_filename;
    filename.erase(filename.length() - 4, 4);
    filename.append(".mat");
    matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT73);
    if (reinterpret_cast<int64_t *>(matfp) != NULL)
        {
            size_t dims[2] = {1, static_cast<size_t>(num_epoch)};
            matvar = Mat_VarCreate("abs_VE", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, abs_VE, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_E", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, abs_E, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_P", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, abs_P, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_L", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, abs_L, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("abs_VL", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, abs_VL, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Prompt_I", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, Prompt_I, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Prompt_Q", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, Prompt_Q, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN_start_sample_count", MAT_C_UINT64, MAT_T_UINT64, 2, dims, PRN_start_sample_count, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("acc_carrier_phase_rad", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, acc_carrier_phase_rad, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carrier_doppler_hz", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, carrier_doppler_hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_freq_chips", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, code_freq_chips, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carr_error_hz", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, carr_error_hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carr_error_filt_hz", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, carr_error_filt_hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_error_chips", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, code_error_chips, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("code_error_filt_chips", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, code_error_filt_chips, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("CN0_SNV_dB_Hz", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, CN0_SNV_dB_Hz, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("carrier_lock_test", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, carrier_lock_test, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("aux1", MAT_C_SINGLE, MAT_T_SINGLE, 2, dims, aux1, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("aux2", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, aux2, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN", MAT_C_UINT32, MAT_T_UINT32, 2, dims, PRN, 0);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB);  // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);
        }
    Mat_Close(matfp);
    delete[] abs_VE;
    delete[] abs_E;
    delete[] abs_P;
    delete[] abs_L;
    delete[] abs_VL;
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


void dll_pll_veml_tracking::set_channel(uint32_t channel)
{
    gr::thread::scoped_lock l(d_setlock);
    d_channel = channel;
    LOG(INFO) << "Tracking Channel set to " << d_channel;
    // ############# ENABLE DATA FILE LOG #################
    if (trk_parameters.dump)
        {
            if (!d_dump_file.is_open())
                {
                    try
                        {
                            trk_parameters.dump_filename.append(boost::lexical_cast<std::string>(d_channel));
                            trk_parameters.dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(trk_parameters.dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Tracking dump enabled on channel " << d_channel << " Log file: " << trk_parameters.dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
                        }
                }
        }
}


void dll_pll_veml_tracking::set_gnss_synchro(Gnss_Synchro *p_gnss_synchro)
{
    gr::thread::scoped_lock l(d_setlock);
    d_acquisition_gnss_synchro = p_gnss_synchro;
}


int dll_pll_veml_tracking::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items,
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    gr::thread::scoped_lock l(d_setlock);
    const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);
    Gnss_Synchro **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);
    Gnss_Synchro current_synchro_data = Gnss_Synchro();

    switch (d_state)
        {
        case 0:  // Standby - Consume samples at full throttle, do nothing
            {
                d_sample_counter += ninput_items[0];
                consume_each(ninput_items[0]);
                return 0;
                break;
            }
        case 1:  // Pull-in
            {
                // Signal alignment (skip samples until the incoming signal is aligned with local replica)
                uint64_t acq_to_trk_delay_samples = d_sample_counter - d_acq_sample_stamp;
                double acq_trk_shif_correction_samples = static_cast<double>(d_current_prn_length_samples) - std::fmod(static_cast<double>(acq_to_trk_delay_samples), static_cast<double>(d_current_prn_length_samples));
                int32_t samples_offset = std::round(d_acq_code_phase_samples + acq_trk_shif_correction_samples);
                if (samples_offset < 0)
                    {
                        samples_offset = 0;
                    }
                d_acc_carrier_phase_rad -= d_carrier_phase_step_rad * d_acq_code_phase_samples;
                d_state = 2;
                d_sample_counter += samples_offset;  // count for the processed samples
                consume_each(samples_offset);        // shift input to perform alignment with local replica
                return 0;
            }
        case 2:  // Wide tracking and symbol synchronization
            {
                do_correlation_step(in);
                // Save single correlation step variables
                if (d_veml)
                    {
                        d_VE_accu = *d_Very_Early;
                        d_VL_accu = *d_Very_Late;
                    }
                d_E_accu = *d_Early;
                d_P_accu = *d_Prompt;
                d_L_accu = *d_Late;

                // Check lock status
                if (!cn0_and_tracking_lock_status(d_code_period))
                    {
                        clear_tracking_vars();
                        d_state = 0;  // loss-of-lock detected
                    }
                else
                    {
                        bool next_state = false;
                        // Perform DLL/PLL tracking loop computations. Costas Loop enabled
                        run_dll_pll();
                        update_tracking_vars();

                        // enable write dump file this cycle (valid DLL/PLL cycle)
                        log_data(false);
                        if (d_secondary)
                            {
                                // ####### SECONDARY CODE LOCK #####
                                d_Prompt_buffer_deque.push_back(*d_Prompt);
                                if (d_Prompt_buffer_deque.size() == d_secondary_code_length)
                                    {
                                        next_state = acquire_secondary();
                                        if (next_state)
                                            {
                                                std::cout << systemName << " " << signal_pretty_name << " secondary code locked in channel " << d_channel
                                                          << " for satellite " << Gnss_Satellite(systemName, d_acquisition_gnss_synchro->PRN) << std::endl;
                                            }

                                        d_Prompt_buffer_deque.pop_front();
                                    }
                            }
                        else if (d_symbols_per_bit > 1)  //Signal does not have secondary code. Search a bit transition by sign change
                            {
                                float current_tracking_time_s = static_cast<float>(d_sample_counter - d_acq_sample_stamp) / trk_parameters.fs_in;
                                if (current_tracking_time_s > 10)
                                    {
                                        d_symbol_history.push_back(d_Prompt->real());
                                        //******* preamble correlation ********
                                        int32_t corr_value = 0;
                                        if ((d_symbol_history.size() == GPS_CA_PREAMBLE_LENGTH_SYMBOLS))  // and (d_make_correlation or !d_flag_frame_sync))
                                            {
                                                for (uint32_t i = 0; i < GPS_CA_PREAMBLE_LENGTH_SYMBOLS; i++)
                                                    {
                                                        if (d_symbol_history.at(i) < 0)  // symbols clipping
                                                            {
                                                                corr_value -= d_gps_l1ca_preambles_symbols[i];
                                                            }
                                                        else
                                                            {
                                                                corr_value += d_gps_l1ca_preambles_symbols[i];
                                                            }
                                                    }
                                            }
                                        if (corr_value == GPS_CA_PREAMBLE_LENGTH_SYMBOLS)
                                            {
                                                //std::cout << "Preamble detected at tracking!" << std::endl;
                                                next_state = true;
                                            }
                                        else
                                            {
                                                next_state = false;
                                            }
                                    }
                                else
                                    {
                                        next_state = false;
                                    }
                            }
                        else
                            {
                                next_state = true;
                            }

                        // ########### Output the tracking results to Telemetry block ##########
                        if (interchange_iq)
                            {
                                if (trk_parameters.track_pilot)
                                    {
                                        // Note that data and pilot components are in quadrature. I and Q are interchanged
                                        current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt_Data).imag());
                                        current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt_Data).real());
                                    }
                                else
                                    {
                                        current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt).imag());
                                        current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt).real());
                                    }
                            }
                        else
                            {
                                if (trk_parameters.track_pilot)
                                    {
                                        // Note that data and pilot components are in quadrature. I and Q are interchanged
                                        current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt_Data).real());
                                        current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt_Data).imag());
                                    }
                                else
                                    {
                                        current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt).real());
                                        current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt).imag());
                                    }
                            }
                        current_synchro_data.Code_phase_samples = d_rem_code_phase_samples;
                        current_synchro_data.Carrier_phase_rads = d_acc_carrier_phase_rad;
                        current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
                        current_synchro_data.CN0_dB_hz = d_CN0_SNV_dB_Hz;
                        current_synchro_data.Flag_valid_symbol_output = true;
                        current_synchro_data.correlation_length_ms = d_correlation_length_ms;

                        if (next_state)
                            {  // reset extended correlator
                                d_VE_accu = gr_complex(0.0, 0.0);
                                d_E_accu = gr_complex(0.0, 0.0);
                                d_P_accu = gr_complex(0.0, 0.0);
                                d_L_accu = gr_complex(0.0, 0.0);
                                d_VL_accu = gr_complex(0.0, 0.0);
                                d_last_prompt = gr_complex(0.0, 0.0);
                                d_Prompt_buffer_deque.clear();
                                d_current_symbol = 0;

                                if (d_enable_extended_integration)
                                    {
                                        // UPDATE INTEGRATION TIME
                                        d_extend_correlation_symbols_count = 0;
                                        float new_correlation_time = static_cast<float>(trk_parameters.extend_correlation_symbols) * static_cast<float>(d_code_period);
                                        d_carrier_loop_filter.set_pdi(new_correlation_time);
                                        d_code_loop_filter.set_pdi(new_correlation_time);
                                        d_state = 3;  // next state is the extended correlator integrator
                                        LOG(INFO) << "Enabled " << trk_parameters.extend_correlation_symbols * static_cast<int>(d_code_period * 1000.0) << " ms extended correlator in channel "
                                                  << d_channel
                                                  << " for satellite " << Gnss_Satellite(systemName, d_acquisition_gnss_synchro->PRN);
                                        std::cout << "Enabled " << trk_parameters.extend_correlation_symbols * static_cast<int>(d_code_period * 1000.0) << " ms extended correlator in channel "
                                                  << d_channel
                                                  << " for satellite " << Gnss_Satellite(systemName, d_acquisition_gnss_synchro->PRN) << std::endl;
                                        // Set narrow taps delay values [chips]
                                        d_code_loop_filter.set_DLL_BW(trk_parameters.dll_bw_narrow_hz);
                                        d_carrier_loop_filter.set_PLL_BW(trk_parameters.pll_bw_narrow_hz);
                                        if (d_veml)
                                            {
                                                d_local_code_shift_chips[0] = -trk_parameters.very_early_late_space_narrow_chips * static_cast<float>(d_code_samples_per_chip);
                                                d_local_code_shift_chips[1] = -trk_parameters.early_late_space_narrow_chips * static_cast<float>(d_code_samples_per_chip);
                                                d_local_code_shift_chips[3] = trk_parameters.early_late_space_narrow_chips * static_cast<float>(d_code_samples_per_chip);
                                                d_local_code_shift_chips[4] = trk_parameters.very_early_late_space_narrow_chips * static_cast<float>(d_code_samples_per_chip);
                                            }
                                        else
                                            {
                                                d_local_code_shift_chips[0] = -trk_parameters.early_late_space_narrow_chips * static_cast<float>(d_code_samples_per_chip);
                                                d_local_code_shift_chips[2] = trk_parameters.early_late_space_narrow_chips * static_cast<float>(d_code_samples_per_chip);
                                            }
                                    }
                                else
                                    {
                                        d_state = 4;
                                    }
                            }
                    }
                break;
            }
        case 3:  // coherent integration (correlation time extension)
            {
                // Fill the acquisition data
                current_synchro_data = *d_acquisition_gnss_synchro;
                // perform a correlation step
                do_correlation_step(in);
                update_tracking_vars();
                save_correlation_results();

                // ########### Output the tracking results to Telemetry block ##########
                if (interchange_iq)
                    {
                        if (trk_parameters.track_pilot)
                            {
                                // Note that data and pilot components are in quadrature. I and Q are interchanged
                                current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt_Data).imag());
                                current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt_Data).real());
                            }
                        else
                            {
                                current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt).imag());
                                current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt).real());
                            }
                    }
                else
                    {
                        if (trk_parameters.track_pilot)
                            {
                                // Note that data and pilot components are in quadrature. I and Q are interchanged
                                current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt_Data).real());
                                current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt_Data).imag());
                            }
                        else
                            {
                                current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt).real());
                                current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt).imag());
                            }
                    }
                current_synchro_data.Code_phase_samples = d_rem_code_phase_samples;
                current_synchro_data.Carrier_phase_rads = d_acc_carrier_phase_rad;
                current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
                current_synchro_data.CN0_dB_hz = d_CN0_SNV_dB_Hz;
                current_synchro_data.Flag_valid_symbol_output = true;
                current_synchro_data.correlation_length_ms = d_correlation_length_ms;
                d_extend_correlation_symbols_count++;
                if (d_extend_correlation_symbols_count == (trk_parameters.extend_correlation_symbols - 1))
                    {
                        d_extend_correlation_symbols_count = 0;
                        d_state = 4;
                    }
                log_data(true);
                break;
            }
        case 4:  // narrow tracking
            {
                // Fill the acquisition data
                current_synchro_data = *d_acquisition_gnss_synchro;

                // perform a correlation step
                do_correlation_step(in);
                save_correlation_results();

                // check lock status
                if (!cn0_and_tracking_lock_status(d_code_period * static_cast<double>(trk_parameters.extend_correlation_symbols)))
                    {
                        clear_tracking_vars();
                        d_state = 0;  // loss-of-lock detected
                    }
                else
                    {
                        run_dll_pll();
                        update_tracking_vars();

                        // ########### Output the tracking results to Telemetry block ##########
                        if (interchange_iq)
                            {
                                if (trk_parameters.track_pilot)
                                    {
                                        // Note that data and pilot components are in quadrature. I and Q are interchanged
                                        current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt_Data).imag());
                                        current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt_Data).real());
                                    }
                                else
                                    {
                                        current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt).imag());
                                        current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt).real());
                                    }
                            }
                        else
                            {
                                if (trk_parameters.track_pilot)
                                    {
                                        // Note that data and pilot components are in quadrature. I and Q are interchanged
                                        current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt_Data).real());
                                        current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt_Data).imag());
                                    }
                                else
                                    {
                                        current_synchro_data.Prompt_I = static_cast<double>((*d_Prompt).real());
                                        current_synchro_data.Prompt_Q = static_cast<double>((*d_Prompt).imag());
                                    }
                            }
                        current_synchro_data.Code_phase_samples = d_rem_code_phase_samples;
                        current_synchro_data.Carrier_phase_rads = d_acc_carrier_phase_rad;
                        current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
                        current_synchro_data.CN0_dB_hz = d_CN0_SNV_dB_Hz;
                        current_synchro_data.Flag_valid_symbol_output = true;
                        current_synchro_data.correlation_length_ms = d_correlation_length_ms;
                        // enable write dump file this cycle (valid DLL/PLL cycle)
                        log_data(false);
                        // reset extended correlator
                        d_VE_accu = gr_complex(0.0, 0.0);
                        d_E_accu = gr_complex(0.0, 0.0);
                        d_P_accu = gr_complex(0.0, 0.0);
                        d_L_accu = gr_complex(0.0, 0.0);
                        d_VL_accu = gr_complex(0.0, 0.0);
                        if (d_enable_extended_integration)
                            {
                                d_state = 3;  // new coherent integration (correlation time extension) cycle
                            }
                    }
            }
        }
    consume_each(d_current_prn_length_samples);
    d_sample_counter += static_cast<uint64_t>(d_current_prn_length_samples);
    if (current_synchro_data.Flag_valid_symbol_output)
        {
            current_synchro_data.fs = static_cast<int64_t>(trk_parameters.fs_in);
            current_synchro_data.Tracking_sample_counter = d_sample_counter;
            *out[0] = current_synchro_data;
            return 1;
        }
    return 0;
}
