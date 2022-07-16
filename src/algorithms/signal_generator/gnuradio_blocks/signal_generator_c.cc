/*!
 * \file signal_generator_c.cc
 * \brief GNU Radio source block that generates synthesized GNSS signal.
 * \author Marc Molina, 2013. marc.molina.pena@gmail.com
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "signal_generator_c.h"
#include "GLONASS_L1_L2_CA.h"
#include "GPS_L1_CA.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "Galileo_E5b.h"
#include "Galileo_E6.h"
#include "galileo_e1_signal_replica.h"
#include "galileo_e5_signal_replica.h"
#include "galileo_e6_signal_replica.h"
#include "glonass_l1_signal_replica.h"
#include "gps_sdr_signal_replica.h"
#include <gnuradio/io_signature.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <utility>


/*
 * Create a new instance of signal_generator_c and return
 * a boost shared_ptr. This is effectively the public constructor.
 */
signal_generator_c_sptr
signal_make_generator_c(const std::vector<std::string> &signal1, const std::vector<std::string> &system, const std::vector<unsigned int> &PRN,
    const std::vector<float> &CN0_dB, const std::vector<float> &doppler_Hz,
    const std::vector<unsigned int> &delay_chips, const std::vector<unsigned int> &delay_sec, bool data_flag, bool noise_flag,
    unsigned int fs_in, unsigned int vector_length, float BW_BB)
{
    return gnuradio::get_initial_sptr(new signal_generator_c(signal1, system, PRN, CN0_dB, doppler_Hz, delay_chips, delay_sec,
        data_flag, noise_flag, fs_in, vector_length, BW_BB));
}


/*
 * The private constructor
 */
signal_generator_c::signal_generator_c(std::vector<std::string> signal1,
    std::vector<std::string> system,
    const std::vector<unsigned int> &PRN,
    std::vector<float> CN0_dB,
    std::vector<float> doppler_Hz,
    std::vector<unsigned int> delay_chips,
    std::vector<unsigned int> delay_sec,
    bool data_flag,
    bool noise_flag,
    unsigned int fs_in,
    unsigned int vector_length,
    float BW_BB) : gr::block("signal_gen_cc", gr::io_signature::make(0, 0, sizeof(gr_complex)), gr::io_signature::make(1, 1, static_cast<int>(sizeof(gr_complex) * vector_length))),
                   signal_(std::move(signal1)),
                   system_(std::move(system)),
                   CN0_dB_(std::move(CN0_dB)),
                   doppler_Hz_(std::move(doppler_Hz)),
                   PRN_(PRN),
                   delay_chips_(std::move(delay_chips)),
                   delay_sec_(std::move(delay_sec)),
                   BW_BB_(BW_BB * static_cast<float>(fs_in) / 2.0F),
                   fs_in_(fs_in),
                   num_sats_(PRN.size()),
                   vector_length_(vector_length),
                   data_flag_(data_flag),
                   noise_flag_(noise_flag)
{
    init();
    generate_codes();
}


void signal_generator_c::init()
{
    work_counter_ = 0;

    complex_phase_ = std::vector<gr_complex>(vector_length_);
    start_phase_rad_.reserve(num_sats_);
    current_data_bit_int_.reserve(num_sats_);
    ms_counter_.reserve(num_sats_);
    data_modulation_.reserve(num_sats_);
    pilot_modulation_.reserve(num_sats_);
    samples_per_code_.reserve(num_sats_);
    num_of_codes_per_vector_.reserve(num_sats_);
    data_bit_duration_ms_.reserve(num_sats_);

    // True if Galileo satellites are present
    bool galileo_signal = std::find(system_.begin(), system_.end(), "E") != system_.end();

    for (unsigned int sat = 0; sat < num_sats_; sat++)
        {
            start_phase_rad_.push_back(0);
            current_data_bit_int_.push_back(1);
            current_data_bits_.emplace_back(1, 0);
            ms_counter_.push_back(0);
            data_modulation_.push_back((GALILEO_E5A_I_SECONDARY_CODE[0] == '0' ? 1 : -1));
            pilot_modulation_.push_back((GALILEO_E5A_Q_SECONDARY_CODE[PRN_[sat]][0] == '0' ? 1 : -1));

            if (system_[sat] == "G")
                {
                    samples_per_code_.push_back(round(static_cast<float>(fs_in_) / (GPS_L1_CA_CODE_RATE_CPS / GPS_L1_CA_CODE_LENGTH_CHIPS)));

                    num_of_codes_per_vector_.push_back(galileo_signal ? 4 * static_cast<int>(GALILEO_E1_C_SECONDARY_CODE_LENGTH) : 1);
                    data_bit_duration_ms_.push_back(1e3 / GPS_CA_TELEMETRY_RATE_BITS_SECOND);
                }
            else if (system_[sat] == "R")
                {
                    samples_per_code_.push_back(round(static_cast<float>(fs_in_) / (GLONASS_L1_CA_CODE_RATE_CPS / GLONASS_L1_CA_CODE_LENGTH_CHIPS)));

                    num_of_codes_per_vector_.push_back(galileo_signal ? 4 * static_cast<int>(GALILEO_E1_C_SECONDARY_CODE_LENGTH) : 1);
                    data_bit_duration_ms_.push_back(1e3 / GLONASS_GNAV_TELEMETRY_RATE_BITS_SECOND);
                }
            else if (system_[sat] == "E")
                {
                    if (signal_[sat].at(0) == '5')
                        {
                            int codelen = static_cast<int>(GALILEO_E5A_CODE_LENGTH_CHIPS);
                            samples_per_code_.push_back(round(static_cast<float>(fs_in_) / (GALILEO_E5A_CODE_CHIP_RATE_CPS / codelen)));
                            num_of_codes_per_vector_.push_back(1);

                            data_bit_duration_ms_.push_back(1e3 / GALILEO_E5A_SYMBOL_RATE_BPS);
                        }
                    else if (signal_[sat].at(0) == '7')
                        {
                            int codelen = static_cast<int>(GALILEO_E5B_CODE_LENGTH_CHIPS);
                            samples_per_code_.push_back(round(static_cast<float>(fs_in_) / (GALILEO_E5B_CODE_CHIP_RATE_CPS / codelen)));
                            num_of_codes_per_vector_.push_back(1);

                            data_bit_duration_ms_.push_back(1e3 / GALILEO_E5B_SYMBOL_RATE_BPS);
                        }
                    else if (signal_[sat].at(1) == '6')
                        {
                            samples_per_code_.push_back(round(static_cast<float>(fs_in_) / (GALILEO_E6_B_CODE_CHIP_RATE_CPS / GALILEO_E6_B_CODE_LENGTH_CHIPS)));

                            num_of_codes_per_vector_.push_back(static_cast<int>(GALILEO_E6_C_SECONDARY_CODE_LENGTH_CHIPS));
                            data_bit_duration_ms_.push_back(1);
                        }
                    else
                        {
                            samples_per_code_.push_back(round(static_cast<float>(fs_in_) / (GALILEO_E1_CODE_CHIP_RATE_CPS / GALILEO_E1_B_CODE_LENGTH_CHIPS)));

                            num_of_codes_per_vector_.push_back(static_cast<int>(GALILEO_E1_C_SECONDARY_CODE_LENGTH));
                            data_bit_duration_ms_.push_back(1e3 / GALILEO_E1_B_SYMBOL_RATE_BPS);
                        }
                }
        }
}


void signal_generator_c::generate_codes()
{
    sampled_code_data_ = std::vector<std::vector<gr_complex>>(num_sats_, std::vector<gr_complex>(vector_length_));
    sampled_code_pilot_ = std::vector<std::vector<gr_complex>>(num_sats_, std::vector<gr_complex>(vector_length_));

    for (unsigned int sat = 0; sat < num_sats_; sat++)
        {
            std::array<gr_complex, 64000> code{};

            if (system_[sat] == "G")
                {
                    // Generate one code-period of 1C signal
                    gps_l1_ca_code_gen_complex_sampled(code, PRN_[sat], fs_in_,
                        static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS) - delay_chips_[sat]);

                    // Obtain the desired CN0 assuming that Pn = 1.
                    if (noise_flag_)
                        {
                            for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                {
                                    code[i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_);
                                }
                        }

                    // Concatenate "num_of_codes_per_vector_" codes
                    for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            std::copy_n(code.data(), samples_per_code_[sat],
                                &(sampled_code_data_[sat][i * samples_per_code_[sat]]));
                        }
                }
            else if (system_[sat] == "R")
                {
                    // Generate one code-period of 1G signal
                    glonass_l1_ca_code_gen_complex_sampled(code, /*PRN_[sat],*/ fs_in_,
                        static_cast<int>(GLONASS_L1_CA_CODE_LENGTH_CHIPS) - delay_chips_[sat]);

                    // Obtain the desired CN0 assuming that Pn = 1.
                    if (noise_flag_)
                        {
                            for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                {
                                    code[i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_);
                                }
                        }

                    // Concatenate "num_of_codes_per_vector_" codes
                    for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            std::copy_n(code.data(), samples_per_code_[sat],
                                &(sampled_code_data_[sat][i * samples_per_code_[sat]]));
                        }
                }
            else if (system_[sat] == "E")
                {
                    if (signal_[sat].at(0) == '5')
                        {
                            std::array<char, 3> signal = {{'5', 'X', '\0'}};

                            galileo_e5_a_code_gen_complex_sampled(sampled_code_data_[sat], PRN_[sat], signal, fs_in_,
                                static_cast<int>(GALILEO_E5A_CODE_LENGTH_CHIPS) - delay_chips_[sat]);
                            // noise
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < vector_length_; i++)
                                        {
                                            sampled_code_data_[sat][i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }
                        }
                    else if (signal_[sat].at(0) == '7')
                        {
                            std::array<char, 3> signal = {{'7', 'X', '\0'}};

                            galileo_e5_b_code_gen_complex_sampled(sampled_code_data_[sat], PRN_[sat], signal, fs_in_,
                                static_cast<int>(GALILEO_E5B_CODE_LENGTH_CHIPS) - delay_chips_[sat]);
                            // noise
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < vector_length_; i++)
                                        {
                                            sampled_code_data_[sat][i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }
                        }
                    else if (signal_[sat].at(1) == '6')
                        {
                            // Generate one code-period of E&B signal
                            galileo_e6_b_code_gen_complex_sampled(code, PRN_[sat], fs_in_,
                                static_cast<int>(GALILEO_E6_B_CODE_LENGTH_CHIPS) - delay_chips_[sat]);
                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                        {
                                            code[i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }
                            // Concatenate "num_of_codes_per_vector_" codes
                            for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    std::copy_n(code.data(), samples_per_code_[sat],
                                        &(sampled_code_data_[sat][i * samples_per_code_[sat]]));
                                }
                            // Generate E6C signal (100 code-periods, with secondary code)
                            galileo_e6_c_code_gen_complex_sampled(sampled_code_pilot_[sat], PRN_[sat], fs_in_,
                                static_cast<int>(GALILEO_E6_C_CODE_LENGTH_CHIPS) - delay_chips_[sat]);
                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < vector_length_; i++)
                                        {
                                            sampled_code_pilot_[sat][i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }
                        }
                    else
                        {
                            // Generate one code-period of E1B signal
                            bool cboc = true;
                            std::array<char, 3> signal = {{'1', 'B', '\0'}};

                            galileo_e1_code_gen_complex_sampled(code, signal, cboc, PRN_[sat], fs_in_,
                                static_cast<int>(GALILEO_E1_B_CODE_LENGTH_CHIPS) - delay_chips_[sat]);

                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                        {
                                            code[i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }

                            // Concatenate "num_of_codes_per_vector_" codes
                            for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    std::copy_n(code.data(), samples_per_code_[sat],
                                        &(sampled_code_data_[sat][i * samples_per_code_[sat]]));
                                }

                            // Generate E1C signal (25 code-periods, with secondary code)

                            std::array<char, 3> signal_1C = {{'1', 'C', '\0'}};

                            galileo_e1_code_gen_complex_sampled(sampled_code_pilot_[sat], signal_1C, cboc, PRN_[sat], fs_in_,
                                static_cast<int>(GALILEO_E1_B_CODE_LENGTH_CHIPS) - delay_chips_[sat], true);

                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < vector_length_; i++)
                                        {
                                            sampled_code_pilot_[sat][i] *= std::sqrt(std::pow(10.0F, CN0_dB_[sat] / 10.0F) / BW_BB_ / 2.0F);
                                        }
                                }
                        }
                }
        }
}


int signal_generator_c::general_work(int noutput_items __attribute__((unused)),
    gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items __attribute__((unused)),
    gr_vector_void_star &output_items)
{
    auto *out = reinterpret_cast<gr_complex *>(output_items[0]);

    work_counter_++;

    std::default_random_engine e1(r());
    std::default_random_engine e2(r());
    unsigned int out_idx = 0;
    unsigned int i = 0;
    unsigned int k = 0;
    // the intermediate frequency must be set by the user
    unsigned int freq = 4e6;

    for (out_idx = 0; out_idx < vector_length_; out_idx++)
        {
            out[out_idx] = gr_complex(0.0, 0.0);
        }

    for (unsigned int sat = 0; sat < num_sats_; sat++)
        {
            float phase_step_rad = -static_cast<float>(TWO_PI) * doppler_Hz_[sat] / static_cast<float>(fs_in_);
            std::array<float, 1> _phase{};
            _phase[0] = -start_phase_rad_[sat];
            volk_gnsssdr_s32f_sincos_32fc(complex_phase_.data(), -phase_step_rad, _phase.data(), vector_length_);
            start_phase_rad_[sat] += static_cast<float>(vector_length_) * phase_step_rad;

            out_idx = 0;

            if (system_[sat] == "G")
                {
                    auto delay_samples = static_cast<unsigned int>((delay_chips_[sat] % static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS)) * samples_per_code_[sat] / GPS_L1_CA_CODE_LENGTH_CHIPS);

                    for (i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            for (k = 0; k < delay_samples; k++)
                                {
                                    out[out_idx] += sampled_code_data_[sat][out_idx] * current_data_bits_[sat] * complex_phase_[out_idx];
                                    out_idx++;
                                }

                            if (ms_counter_[sat] == 0 && data_flag_)
                                {
                                    // New random data bit
                                    current_data_bits_[sat] = gr_complex((uniform_dist(e1) % 2) == 0 ? 1 : -1, 0);
                                }

                            for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                {
                                    out[out_idx] += sampled_code_data_[sat][out_idx] * current_data_bits_[sat] * complex_phase_[out_idx];
                                    out_idx++;
                                }

                            ms_counter_[sat] = (ms_counter_[sat] + static_cast<int>(round(1e3 * GPS_L1_CA_CODE_PERIOD_S))) % data_bit_duration_ms_[sat];
                        }
                }

            else if (system_[sat] == "R")
                {
                    phase_step_rad = -static_cast<float>(TWO_PI) * (static_cast<float>(freq) + (static_cast<float>(DFRQ1_GLO) * GLONASS_PRN.at(PRN_[sat])) + doppler_Hz_[sat]) / static_cast<float>(fs_in_);
                    // std::cout << "sat " << PRN_[sat] << " SG - Freq = " << (freq + (DFRQ1_GLO * GLONASS_PRN.at(PRN_[sat]))) << " Doppler = " << doppler_Hz_[sat] << '\n';
                    _phase[0] = -start_phase_rad_[sat];
                    volk_gnsssdr_s32f_sincos_32fc(complex_phase_.data(), -phase_step_rad, _phase.data(), vector_length_);

                    auto delay_samples = static_cast<unsigned int>((delay_chips_[sat] % static_cast<int>(GLONASS_L1_CA_CODE_LENGTH_CHIPS)) * samples_per_code_[sat] / GLONASS_L1_CA_CODE_LENGTH_CHIPS);

                    for (i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            for (k = 0; k < delay_samples; k++)
                                {
                                    out[out_idx] += sampled_code_data_[sat][out_idx] * current_data_bits_[sat] * complex_phase_[out_idx];
                                    out_idx++;
                                }

                            if (ms_counter_[sat] == 0 && data_flag_)
                                {
                                    // New random data bit
                                    current_data_bits_[sat] = gr_complex((uniform_dist(e1) % 2) == 0 ? 1 : -1, 0);
                                }

                            for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                {
                                    out[out_idx] += sampled_code_data_[sat][out_idx] * current_data_bits_[sat] * complex_phase_[out_idx];
                                    out_idx++;
                                }

                            ms_counter_[sat] = (ms_counter_[sat] + static_cast<int>(round(1e3 * GLONASS_L1_CA_CODE_PERIOD_S))) % data_bit_duration_ms_[sat];
                        }
                }

            else if (system_[sat] == "E")
                {
                    if (signal_[sat].at(0) == '5')
                        {
                            // EACH WORK outputs 1 modulated primary code
                            int codelen = static_cast<int>(GALILEO_E5A_CODE_LENGTH_CHIPS);
                            unsigned int delay_samples = (delay_chips_[sat] % codelen) * samples_per_code_[sat] / codelen;
                            for (k = 0; k < delay_samples; k++)
                                {
                                    out[out_idx] += (gr_complex(sampled_code_data_[sat][out_idx].real() * data_modulation_[sat],
                                                        sampled_code_data_[sat][out_idx].imag() * pilot_modulation_[sat])) *
                                                    complex_phase_[out_idx];
                                    out_idx++;
                                }

                            if (ms_counter_[sat] % data_bit_duration_ms_[sat] == 0 && data_flag_)
                                {
                                    // New random data bit
                                    current_data_bit_int_[sat] = (uniform_dist(e1) % 2) == 0 ? 1 : -1;
                                }
                            data_modulation_[sat] = current_data_bit_int_[sat] * (GALILEO_E5A_I_SECONDARY_CODE[(ms_counter_[sat] + delay_sec_[sat]) % 20] == '0' ? 1 : -1);
                            pilot_modulation_[sat] = (GALILEO_E5A_Q_SECONDARY_CODE[PRN_[sat] - 1][((ms_counter_[sat] + delay_sec_[sat]) % 100)] == '0' ? 1 : -1);

                            ms_counter_[sat] = ms_counter_[sat] + static_cast<int>(round(1e3 * GALILEO_E5A_CODE_PERIOD_S));

                            for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                {
                                    out[out_idx] += (gr_complex(sampled_code_data_[sat][out_idx].real() * data_modulation_[sat],
                                                        sampled_code_data_[sat][out_idx].imag() * pilot_modulation_[sat])) *
                                                    complex_phase_[out_idx];
                                    out_idx++;
                                }
                        }
                    else if (signal_[sat].at(0) == '7')
                        {
                            // EACH WORK outputs 1 modulated primary code
                            int codelen = static_cast<int>(GALILEO_E5B_CODE_LENGTH_CHIPS);
                            unsigned int delay_samples = (delay_chips_[sat] % codelen) * samples_per_code_[sat] / codelen;
                            for (k = 0; k < delay_samples; k++)
                                {
                                    out[out_idx] += (gr_complex(sampled_code_data_[sat][out_idx].real() * data_modulation_[sat],
                                                        sampled_code_data_[sat][out_idx].imag() * pilot_modulation_[sat])) *
                                                    complex_phase_[out_idx];
                                    out_idx++;
                                }

                            if (ms_counter_[sat] % data_bit_duration_ms_[sat] == 0 && data_flag_)
                                {
                                    // New random data bit
                                    current_data_bit_int_[sat] = (uniform_dist(e1) % 2) == 0 ? 1 : -1;
                                }
                            data_modulation_[sat] = current_data_bit_int_[sat] * (GALILEO_E5B_I_SECONDARY_CODE[((ms_counter_[sat] + delay_sec_[sat]) % 4)] == '0' ? 1 : -1);
                            pilot_modulation_[sat] = (GALILEO_E5B_Q_SECONDARY_CODE[PRN_[sat] - 1][((ms_counter_[sat] + delay_sec_[sat]) % 100)] == '0' ? 1 : -1);

                            ms_counter_[sat] = ms_counter_[sat] + static_cast<int>(round(1e3 * GALILEO_E5B_CODE_PERIOD_S));

                            for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                {
                                    out[out_idx] += (gr_complex(sampled_code_data_[sat][out_idx].real() * data_modulation_[sat],
                                                        sampled_code_data_[sat][out_idx].imag() * pilot_modulation_[sat])) *
                                                    complex_phase_[out_idx];
                                    out_idx++;
                                }
                        }
                    else if (signal_[sat].at(1) == '6')
                        {
                            // EACH WORK outputs 1 modulated primary code
                            int codelen = static_cast<int>(GALILEO_E6_C_CODE_LENGTH_CHIPS);
                            unsigned int delay_samples = (delay_chips_[sat] % codelen) * samples_per_code_[sat] / codelen;
                            for (i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    for (k = 0; k < delay_samples; k++)
                                        {
                                            out[out_idx] += (sampled_code_data_[sat][out_idx] * current_data_bits_[sat] - sampled_code_pilot_[sat][out_idx]) * complex_phase_[out_idx];
                                            out_idx++;
                                        }

                                    if (ms_counter_[sat] == 0 && data_flag_)
                                        {
                                            // New random data bit
                                            current_data_bits_[sat] = gr_complex((uniform_dist(e1) % 2) == 0 ? 1 : -1, 0);
                                        }

                                    for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                        {
                                            out[out_idx] += (sampled_code_data_[sat][out_idx] * current_data_bits_[sat] - sampled_code_pilot_[sat][out_idx]) * complex_phase_[out_idx];
                                            out_idx++;
                                        }
                                    ms_counter_[sat] = (ms_counter_[sat] + 1) % data_bit_duration_ms_[sat];
                                }
                        }
                    else
                        {
                            auto delay_samples = static_cast<unsigned int>((delay_chips_[sat] % static_cast<int>(GALILEO_E1_B_CODE_LENGTH_CHIPS)) * samples_per_code_[sat] / GALILEO_E1_B_CODE_LENGTH_CHIPS);

                            for (i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    for (k = 0; k < delay_samples; k++)
                                        {
                                            out[out_idx] += (sampled_code_data_[sat][out_idx] * current_data_bits_[sat] - sampled_code_pilot_[sat][out_idx]) * complex_phase_[out_idx];
                                            out_idx++;
                                        }

                                    if (ms_counter_[sat] == 0 && data_flag_)
                                        {
                                            // New random data bit
                                            current_data_bits_[sat] = gr_complex((uniform_dist(e1) % 2) == 0 ? 1 : -1, 0);
                                        }

                                    for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                        {
                                            out[out_idx] += (sampled_code_data_[sat][out_idx] * current_data_bits_[sat] - sampled_code_pilot_[sat][out_idx]) * complex_phase_[out_idx];
                                            out_idx++;
                                        }

                                    ms_counter_[sat] = (ms_counter_[sat] + static_cast<int>(round(1e3 * GALILEO_E1_CODE_PERIOD_S))) % data_bit_duration_ms_[sat];
                                }
                        }
                }
        }

    if (noise_flag_)
        {
            for (out_idx = 0; out_idx < vector_length_; out_idx++)
                {
                    out[out_idx] += gr_complex(normal_dist(e1), normal_dist(e2));
                }
        }

    // Tell runtime system how many output items we produced.
    return 1;
}
