/*!
* \file signal_generator_c.cc
* \brief GNU Radio source block that generates synthesized GNSS signal.
* \author Marc Molina, 2013. marc.molina.pena@gmail.com
*
* -------------------------------------------------------------------------
*
* Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
*
* GNSS-SDR is a software defined Global Navigation
* Satellite Systems receiver
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
*
* -------------------------------------------------------------------------
*/

#include "signal_generator_c.h"
#include "gps_sdr_signal_processing.h"
#include "glonass_l1_signal_processing.h"
#include "galileo_e1_signal_processing.h"
#include "galileo_e5_signal_processing.h"
#include "Galileo_E1.h"
#include "Galileo_E5a.h"
#include "GPS_L1_CA.h"
#include "GLONASS_L1_L2_CA.h"
#include <gnuradio/io_signature.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <fstream>


/*
* Create a new instance of signal_generator_c and return
* a boost shared_ptr. This is effectively the public constructor.
*/
signal_generator_c_sptr
signal_make_generator_c(std::vector<std::string> signal1, std::vector<std::string> system, const std::vector<unsigned int> &PRN,
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
signal_generator_c::signal_generator_c(std::vector<std::string> signal1, std::vector<std::string> system, const std::vector<unsigned int> &PRN,
    const std::vector<float> &CN0_dB, const std::vector<float> &doppler_Hz,
    const std::vector<unsigned int> &delay_chips, const std::vector<unsigned int> &delay_sec, bool data_flag, bool noise_flag,
    unsigned int fs_in, unsigned int vector_length, float BW_BB) : gr::block("signal_gen_cc", gr::io_signature::make(0, 0, sizeof(gr_complex)),
                                                                       gr::io_signature::make(1, 1, sizeof(gr_complex) * vector_length)),
                                                                   signal_(signal1),
                                                                   system_(system),
                                                                   PRN_(PRN),
                                                                   CN0_dB_(CN0_dB),
                                                                   doppler_Hz_(doppler_Hz),
                                                                   delay_chips_(delay_chips),
                                                                   delay_sec_(delay_sec),
                                                                   data_flag_(data_flag),
                                                                   noise_flag_(noise_flag),
                                                                   fs_in_(fs_in),
                                                                   num_sats_(PRN.size()),
                                                                   vector_length_(vector_length),
                                                                   BW_BB_(BW_BB * static_cast<float>(fs_in) / 2.0)
{
    init();
    generate_codes();
}

void signal_generator_c::init()
{
    work_counter_ = 0;

    complex_phase_ = static_cast<gr_complex *>(volk_gnsssdr_malloc(vector_length_ * sizeof(gr_complex), volk_gnsssdr_get_alignment()));

    // True if Galileo satellites are present
    bool galileo_signal = std::find(system_.begin(), system_.end(), "E") != system_.end();

    for (unsigned int sat = 0; sat < num_sats_; sat++)
        {
            start_phase_rad_.push_back(0);
            current_data_bit_int_.push_back(1);
            current_data_bits_.push_back(gr_complex(1, 0));
            ms_counter_.push_back(0);
            data_modulation_.push_back((Galileo_E5a_I_SECONDARY_CODE.at(0) == '0' ? 1 : -1));
            pilot_modulation_.push_back((Galileo_E5a_Q_SECONDARY_CODE[PRN_[sat]].at(0) == '0' ? 1 : -1));

            if (system_[sat] == "G")
                {
                    samples_per_code_.push_back(round(static_cast<float>(fs_in_) / (GPS_L1_CA_CODE_RATE_HZ / GPS_L1_CA_CODE_LENGTH_CHIPS)));

                    num_of_codes_per_vector_.push_back(galileo_signal ? 4 * static_cast<int>(Galileo_E1_C_SECONDARY_CODE_LENGTH) : 1);
                    data_bit_duration_ms_.push_back(1e3 / GPS_CA_TELEMETRY_RATE_BITS_SECOND);
                }
            else if (system_[sat] == "R")
                {
                    samples_per_code_.push_back(round(static_cast<float>(fs_in_) / (GLONASS_L1_CA_CODE_RATE_HZ / GLONASS_L1_CA_CODE_LENGTH_CHIPS)));

                    num_of_codes_per_vector_.push_back(galileo_signal ? 4 * static_cast<int>(Galileo_E1_C_SECONDARY_CODE_LENGTH) : 1);
                    data_bit_duration_ms_.push_back(1e3 / GLONASS_GNAV_TELEMETRY_RATE_BITS_SECOND);
                }
            else if (system_[sat] == "E")
                {
                    if (signal_[sat].at(0) == '5')
                        {
                            int codelen = static_cast<int>(Galileo_E5a_CODE_LENGTH_CHIPS);
                            samples_per_code_.push_back(round(static_cast<float>(fs_in_) / (Galileo_E5a_CODE_CHIP_RATE_HZ / codelen)));
                            num_of_codes_per_vector_.push_back(1);

                            data_bit_duration_ms_.push_back(1e3 / Galileo_E5a_SYMBOL_RATE_BPS);
                        }
                    else
                        {
                            samples_per_code_.push_back(round(static_cast<float>(fs_in_) / (Galileo_E1_CODE_CHIP_RATE_HZ / Galileo_E1_B_CODE_LENGTH_CHIPS)));

                            num_of_codes_per_vector_.push_back(static_cast<int>(Galileo_E1_C_SECONDARY_CODE_LENGTH));
                            data_bit_duration_ms_.push_back(1e3 / Galileo_E1_B_SYMBOL_RATE_BPS);
                        }
                }
        }
    random_ = new gr::random();
    std::default_random_engine e1(r());
    std::uniform_int_distribution<int> uniform_dist(0, RAND_MAX);
}


void signal_generator_c::generate_codes()
{
    sampled_code_data_.reset(new gr_complex *[num_sats_]);
    sampled_code_pilot_.reset(new gr_complex *[num_sats_]);

    for (unsigned int sat = 0; sat < num_sats_; sat++)
        {
            sampled_code_data_[sat] = static_cast<gr_complex *>(std::malloc(vector_length_ * sizeof(gr_complex)));

            gr_complex code[64000];  //[samples_per_code_[sat]];

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
                                    code[i] *= sqrt(pow(10, CN0_dB_[sat] / 10) / BW_BB_);
                                }
                        }

                    // Concatenate "num_of_codes_per_vector_" codes
                    for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            memcpy(&(sampled_code_data_[sat][i * samples_per_code_[sat]]),
                                code, sizeof(gr_complex) * samples_per_code_[sat]);
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
                                    code[i] *= sqrt(pow(10, CN0_dB_[sat] / 10) / BW_BB_);
                                }
                        }

                    // Concatenate "num_of_codes_per_vector_" codes
                    for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                        {
                            memcpy(&(sampled_code_data_[sat][i * samples_per_code_[sat]]),
                                code, sizeof(gr_complex) * samples_per_code_[sat]);
                        }
                }
            else if (system_[sat] == "E")
                {
                    if (signal_[sat].at(0) == '5')
                        {
                            char signal[3];
                            strcpy(signal, "5X");

                            galileo_e5_a_code_gen_complex_sampled(sampled_code_data_[sat], signal, PRN_[sat], fs_in_,
                                static_cast<int>(Galileo_E5a_CODE_LENGTH_CHIPS) - delay_chips_[sat]);
                            //noise
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < vector_length_; i++)
                                        {
                                            sampled_code_data_[sat][i] *= sqrt(pow(10, CN0_dB_[sat] / 10) / BW_BB_ / 2);
                                        }
                                }
                        }
                    else
                        {
                            // Generate one code-period of E1B signal
                            bool cboc = true;
                            char signal[3];
                            strcpy(signal, "1B");

                            galileo_e1_code_gen_complex_sampled(code, signal, cboc, PRN_[sat], fs_in_,
                                static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS) - delay_chips_[sat]);

                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < samples_per_code_[sat]; i++)
                                        {
                                            code[i] *= sqrt(pow(10, CN0_dB_[sat] / 10) / BW_BB_ / 2);
                                        }
                                }

                            // Concatenate "num_of_codes_per_vector_" codes
                            for (unsigned int i = 0; i < num_of_codes_per_vector_[sat]; i++)
                                {
                                    memcpy(&(sampled_code_data_[sat][i * samples_per_code_[sat]]),
                                        code, sizeof(gr_complex) * samples_per_code_[sat]);
                                }

                            // Generate E1C signal (25 code-periods, with secondary code)
                            sampled_code_pilot_[sat] = static_cast<gr_complex *>(std::malloc(vector_length_ * sizeof(gr_complex)));

                            strcpy(signal, "1C");

                            galileo_e1_code_gen_complex_sampled(sampled_code_pilot_[sat], signal, cboc, PRN_[sat], fs_in_,
                                static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS) - delay_chips_[sat], true);

                            // Obtain the desired CN0 assuming that Pn = 1.
                            if (noise_flag_)
                                {
                                    for (unsigned int i = 0; i < vector_length_; i++)
                                        {
                                            sampled_code_pilot_[sat][i] *= sqrt(pow(10, CN0_dB_[sat] / 10) / BW_BB_ / 2);
                                        }
                                }
                        }
                }
        }
}


signal_generator_c::~signal_generator_c()
{
    /*  for (unsigned int sat = 0; sat < num_sats_; sat++)
        {
            std::free(sampled_code_data_[sat]);
            if (system_[sat] == "E" && signal_[sat].at(0) != '5')
                {
                    std::free(sampled_code_pilot_[sat]);
                }
        } */
    volk_gnsssdr_free(complex_phase_);
    delete random_;
}


int signal_generator_c::general_work(int noutput_items __attribute__((unused)),
    gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items __attribute__((unused)),
    gr_vector_void_star &output_items)
{
    gr_complex *out = reinterpret_cast<gr_complex *>(output_items[0]);

    work_counter_++;

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
            float phase_step_rad = -static_cast<float>(GPS_TWO_PI) * doppler_Hz_[sat] / static_cast<float>(fs_in_);
            float _phase[1];
            _phase[0] = -start_phase_rad_[sat];
            volk_gnsssdr_s32f_sincos_32fc(complex_phase_, -phase_step_rad, _phase, vector_length_);
            start_phase_rad_[sat] += vector_length_ * phase_step_rad;

            out_idx = 0;

            if (system_[sat] == "G")
                {
                    unsigned int delay_samples = (delay_chips_[sat] % static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS)) * samples_per_code_[sat] / GPS_L1_CA_CODE_LENGTH_CHIPS;

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

                            ms_counter_[sat] = (ms_counter_[sat] + static_cast<int>(round(1e3 * GPS_L1_CA_CODE_PERIOD))) % data_bit_duration_ms_[sat];
                        }
                }

            else if (system_[sat] == "R")
                {
                    phase_step_rad = -static_cast<float>(GPS_TWO_PI) * (freq + (DFRQ1_GLO * GLONASS_PRN.at(PRN_[sat])) + doppler_Hz_[sat]) / static_cast<float>(fs_in_);
                    // std::cout << "sat " << PRN_[sat] << " SG - Freq = " << (freq + (DFRQ1_GLO * GLONASS_PRN.at(PRN_[sat]))) << " Doppler = " << doppler_Hz_[sat] << std::endl;
                    _phase[0] = -start_phase_rad_[sat];
                    volk_gnsssdr_s32f_sincos_32fc(complex_phase_, -phase_step_rad, _phase, vector_length_);

                    unsigned int delay_samples = (delay_chips_[sat] % static_cast<int>(GLONASS_L1_CA_CODE_LENGTH_CHIPS)) * samples_per_code_[sat] / GLONASS_L1_CA_CODE_LENGTH_CHIPS;

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

                            ms_counter_[sat] = (ms_counter_[sat] + static_cast<int>(round(1e3 * GLONASS_L1_CA_CODE_PERIOD))) % data_bit_duration_ms_[sat];
                        }
                }

            else if (system_[sat] == "E")
                {
                    if (signal_[sat].at(0) == '5')
                        {
                            // EACH WORK outputs 1 modulated primary code
                            int codelen = static_cast<int>(Galileo_E5a_CODE_LENGTH_CHIPS);
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
                            data_modulation_[sat] = current_data_bit_int_[sat] * (Galileo_E5a_I_SECONDARY_CODE.at((ms_counter_[sat] + delay_sec_[sat]) % 20) == '0' ? 1 : -1);
                            pilot_modulation_[sat] = (Galileo_E5a_Q_SECONDARY_CODE[PRN_[sat] - 1].at((ms_counter_[sat] + delay_sec_[sat]) % 100) == '0' ? 1 : -1);

                            ms_counter_[sat] = ms_counter_[sat] + static_cast<int>(round(1e3 * GALILEO_E5a_CODE_PERIOD));

                            for (k = delay_samples; k < samples_per_code_[sat]; k++)
                                {
                                    out[out_idx] += (gr_complex(sampled_code_data_[sat][out_idx].real() * data_modulation_[sat],
                                                        sampled_code_data_[sat][out_idx].imag() * pilot_modulation_[sat])) *
                                                    complex_phase_[out_idx];
                                    out_idx++;
                                }
                        }
                    else
                        {
                            unsigned int delay_samples = (delay_chips_[sat] % static_cast<int>(Galileo_E1_B_CODE_LENGTH_CHIPS)) * samples_per_code_[sat] / Galileo_E1_B_CODE_LENGTH_CHIPS;

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

                                    ms_counter_[sat] = (ms_counter_[sat] + static_cast<int>(round(1e3 * Galileo_E1_CODE_PERIOD))) % data_bit_duration_ms_[sat];
                                }
                        }
                }
        }

    if (noise_flag_)
        {
            for (out_idx = 0; out_idx < vector_length_; out_idx++)
                {
                    out[out_idx] += gr_complex(random_->gasdev(), random_->gasdev());
                }
        }

    // Tell runtime system how many output items we produced.
    return 1;
}
